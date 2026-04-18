#include "GeofenceModule.h"

#include <algorithm>
#include "FSCommon.h"
#include "NodeDB.h"
#include "RTC.h"
#include "buzz/buzz.h"
#include "graphics/SharedUIDisplay.h"
#include "main.h"

GeofenceModule *geofenceModule;

namespace
{
constexpr uint32_t geofenceAlarmBeepIntervalMs = 1200;
constexpr uint32_t geofenceArmMaxTargetAgeSeconds = 5 * 60;

#if defined(USE_RF95) && defined(PIN_BUZZER) && (PIN_BUZZER == 19)
constexpr bool geofenceBuzzerAllowed = false;
#else
constexpr bool geofenceBuzzerAllowed = true;
#endif

const uint16_t radiusPresetsMeters[] = {25, 50, 100, 200, 500};
constexpr size_t radiusPresetCount = sizeof(radiusPresetsMeters) / sizeof(radiusPresetsMeters[0]);
}

const char *GeofenceModule::stateFilePath()
{
    return "/prefs/geofence.bin";
}

GeofenceModule::GeofenceModule()
    : ProtobufModule("Geofence", meshtastic_PortNum_POSITION_APP, &meshtastic_Position_msg),
      concurrency::OSThread("Geofence")
{
    isPromiscuous = true;
    geofenceModule = this;

    loadState();

#if HAS_SCREEN
#if !MESHTASTIC_EXCLUDE_INPUTBROKER
    if (inputBroker) {
        inputObserver.observe(inputBroker);
    }
#endif
#endif

    setInterval(100); // Fast polling for responsive UI
}

bool GeofenceModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p)
{
    if (!p) {
        return false;
    }

    if (getFrom(&mp) != state.targetNode) {
        return false;
    }

    // Don't update monitoring here - let the thread handle it
    // Accessing NodeDB during radio packet processing can cause conflicts
    // with RF95Interface setStandby() operations
    return false;
}

int32_t GeofenceModule::runOnce()
{
    const uint32_t nowMs = millis();
    
    // Only actively monitor if armed or alarm is active
    // This prevents constant NodeDB access which causes radio driver conflicts
    if (state.armed || runtime.alarmActive) {
        static uint32_t lastMonitorUpdateMs = 0;
        
        // Update monitoring every 500ms when actively running
        if (nowMs - lastMonitorUpdateMs >= 500) {
            updateMonitoringFromNodeDB();
            lastMonitorUpdateMs = nowMs;
        }
        
        // Handle alarm beeping
        if (runtime.alarmActive && !state.alarmSilenced) {
            if (runtime.lastAlarmBeepMs == 0 || (nowMs - runtime.lastAlarmBeepMs) >= geofenceAlarmBeepIntervalMs) {
#if defined(PIN_BUZZER)
                if (geofenceBuzzerAllowed) {
                    playLongBeep();
                }
#endif
                runtime.lastAlarmBeepMs = nowMs;
            }
        }
        
        return 100; // Active monitoring: check every 100ms
    }
    
    // Idle state: sleep the thread to avoid unnecessary radio conflicts
    // Return a long interval so thread mostly sleeps
    return 100; // Idle: check every 100ms
}

void GeofenceModule::updateMonitoringFromNodeDB()
{
    runtime.lastDistanceMeters = -1;
    runtime.lastTargetPositionAgeSec = 0;

    // Quick exit if not actively monitoring
    if (!state.armed && !runtime.alarmActive) {
        runtime.timeoutConsecutiveCount = 0;
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    if (!state.targetNode) {
        runtime.timeoutConsecutiveCount = 0;
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    // Safely access NodeDB with guard checks
    if (!nodeDB) {
        return;
    }

    meshtastic_NodeInfoLite *targetNode = nodeDB->getMeshNode(state.targetNode);
    if (!targetNode) {
        runtime.timeoutConsecutiveCount = 0;
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    runtime.lastTargetPositionAgeSec = sinceLastSeen(targetNode);

    // Offline alarm: target has not been seen for 5 minutes.
    if (state.armed && runtime.lastTargetPositionAgeSec > geofenceArmMaxTargetAgeSeconds) {
        if (runtime.timeoutConsecutiveCount < UINT8_MAX) {
            runtime.timeoutConsecutiveCount++;
        }
        if (runtime.timeoutConsecutiveCount >= state.timeoutThreshold) {
            triggerAlarm(GeofenceModule::AlarmType::Offline);
        }
    } else {
        runtime.timeoutConsecutiveCount = 0;
    }

    if (runtime.lastTargetPositionAgeSec > geofenceArmMaxTargetAgeSeconds) {
        return;
    }

    // Skip distance check if not armed
    if (!state.armed) {
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    // Check if target has valid position and we have a center
    if (!nodeDB->hasValidPosition(targetNode) || !state.hasCenter) {
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    GeoCoord fenceCenter(state.centerLat, state.centerLon, 0);
    GeoCoord targetPosition(targetNode->position.latitude_i, targetNode->position.longitude_i, 0);
    runtime.lastDistanceMeters = fenceCenter.distanceTo(targetPosition);

    // Distance breach detection with hysteresis
    if (runtime.lastDistanceMeters > state.radiusMeters) {
        if (runtime.outsideConsecutiveCount < UINT8_MAX) {
            runtime.outsideConsecutiveCount++;
        }
        if (runtime.outsideConsecutiveCount >= state.outsideThreshold) {
            triggerAlarm(GeofenceModule::AlarmType::Geofence);
        }
    } else {
        runtime.outsideConsecutiveCount = 0;
        // Clear alarm if we've returned to the safe zone
        if (runtime.alarmActive) {
            clearAlarm(true);
        }
    }
}

bool GeofenceModule::refreshCenterFromTargetPosition()
{
    if (!state.targetNode) {
        return false;
    }

    meshtastic_NodeInfoLite *targetNode = nodeDB->getMeshNode(state.targetNode);
    if (!targetNode || !nodeDB->hasValidPosition(targetNode)) {
        return false;
    }

    state.centerLat = targetNode->position.latitude_i;
    state.centerLon = targetNode->position.longitude_i;
    state.hasCenter = true;
    return true;
}

void GeofenceModule::disarm()
{
    state.armed = false;
    state.hasCenter = false;
    runtime.outsideConsecutiveCount = 0;
    runtime.timeoutConsecutiveCount = 0;
    clearAlarm();
    saveState(true);
}

void GeofenceModule::triggerAlarm(AlarmType alarmType)
{
    // If user acknowledged/silenced the alarm, don't re-trigger until state changes
    // (e.g. arm/reset clears alarmSilenced).
    if (state.alarmSilenced) {
        return;
    }

    if (runtime.alarmActive) {
        return;
    }

    runtime.alarmActive = true;
    runtime.alarmType = alarmType;
    runtime.lastAlarmBeepMs = 0;
#if HAS_SCREEN
    if (screen && !state.alarmSilenced) {
        screen->startAlert(alarmType == AlarmType::Offline ? "Offline Alarm" : "Geofence Alarm");
    }
#endif
}

void GeofenceModule::clearAlarm(bool keepSilenced)
{
    runtime.alarmActive = false;
    runtime.alarmType = AlarmType::None;
    runtime.lastAlarmBeepMs = 0;
    if (!keepSilenced) {
        state.alarmSilenced = false;
    }
#if HAS_SCREEN
    if (screen) {
        screen->endAlert();
    }
#endif
}

void GeofenceModule::loadState()
{
#ifdef FSCom
    auto file = FSCom.open(stateFilePath(), FILE_O_READ);
    if (!file) {
        return;
    }

    PersistedState loaded = {};
    if (file.read((uint8_t *)&loaded, sizeof(loaded)) == sizeof(loaded) && loaded.magic == GEOFENCE_MAGIC &&
        loaded.version == GEOFENCE_STATE_VERSION) {
        state = loaded;
    }
    file.close();
#endif
}

void GeofenceModule::saveState(bool force)
{
#ifdef FSCom
    const uint32_t nowMs = millis();
    if (!force && runtime.lastSavedMs != 0 && (nowMs - runtime.lastSavedMs) < 2000) {
        return;
    }

    FSCom.mkdir("/prefs");
    auto file = FSCom.open(stateFilePath(), FILE_O_WRITE);
    if (!file) {
        return;
    }

    file.write((const uint8_t *)&state, sizeof(state));
    file.flush();
    file.close();
    runtime.lastSavedMs = nowMs;
#else
    (void)force;
#endif
}

#if HAS_SCREEN
void GeofenceModule::notifyScreenUpdate(UIFrameEvent::Action action, bool focusFrame)
{
    if (focusFrame) {
        requestFocus();
    }

    UIFrameEvent event;
    event.action = action;
    notifyObservers(&event);
}

void GeofenceModule::setTargetNode(uint32_t nodenum)
{
    if (!nodenum || !nodeDB) {
        return;
    }

    meshtastic_NodeInfoLite *targetNode = nodeDB->getMeshNode(nodenum);
    if (!targetNode) {
        if (screen) {
            screen->showSimpleBanner("Target not\nfound", 2000);
        }
        return;
    }

    state.targetNode = nodenum;

    if (state.armed && (!nodeDB->hasValidPosition(targetNode) || sinceLastSeen(targetNode) > geofenceArmMaxTargetAgeSeconds)) {
        disarm();
        notifyScreenUpdate(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
        if (screen) {
            screen->showSimpleBanner(!nodeDB->hasValidPosition(targetNode) ? "Geofence off\nno target GPS"
                                                                           : "Geofence off\ntarget offline",
                                     2500);
        }
        return;
    }

    saveState(true);
    notifyScreenUpdate(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
}

void GeofenceModule::setRadiusMeters(uint16_t meters)
{
    for (size_t index = 0; index < radiusPresetCount; ++index) {
        if (radiusPresetsMeters[index] == meters) {
            state.radiusMeters = meters;
            saveState(true);
            notifyScreenUpdate(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
            return;
        }
    }
}

void GeofenceModule::armOrReset()
{
    if (!state.targetNode) {
        if (screen) {
            screen->showSimpleBanner("Select target\nfirst", 2000);
        }
        return;
    }

    meshtastic_NodeInfoLite *targetNode = nodeDB ? nodeDB->getMeshNode(state.targetNode) : nullptr;
    if (!targetNode) {
        if (screen) {
            screen->showSimpleBanner("Target not\nfound", 2000);
        }
        return;
    }

    if (!nodeDB->hasValidPosition(targetNode)) {
        if (screen) {
            screen->showSimpleBanner("Target has\nno position", 2000);
        }
        return;
    }

    if (sinceLastSeen(targetNode) > geofenceArmMaxTargetAgeSeconds) {
        if (screen) {
            screen->showSimpleBanner("Target stale\n(>5 min)", 2500);
        }
        return;
    }

    if (!refreshCenterFromTargetPosition()) {
        if (screen) {
            screen->showSimpleBanner("Target has\nno position", 2000);
        }
        return;
    }

    state.armed = true;
    state.alarmSilenced = false;
    runtime.timeoutConsecutiveCount = 0;
    runtime.outsideConsecutiveCount = 0;
    clearAlarm();
    saveState(true);
    notifyScreenUpdate(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
}

void GeofenceModule::disarmFromMenu()
{
    disarm();
    notifyScreenUpdate(UIFrameEvent::Action::REGENERATE_FRAMESET, true);
}

bool GeofenceModule::isModuleFrame(const MeshModule *module) const
{
    return module == static_cast<const MeshModule *>(this);
}

static bool isAlarmAcknowledgeEvent(const InputEvent *event)
{
    switch (event->inputEvent) {
    case INPUT_BROKER_SELECT:
    case INPUT_BROKER_SELECT_LONG:
    case INPUT_BROKER_UP:
    case INPUT_BROKER_DOWN:
    case INPUT_BROKER_LEFT:
    case INPUT_BROKER_RIGHT:
    case INPUT_BROKER_UP_LONG:
    case INPUT_BROKER_DOWN_LONG:
    case INPUT_BROKER_CANCEL:
    case INPUT_BROKER_BACK:
    case INPUT_BROKER_USER_PRESS:
    case INPUT_BROKER_ALT_PRESS:
    case INPUT_BROKER_ALT_LONG:
    case INPUT_BROKER_ANYKEY:
        return true;
    default:
        return false;
    }
}

int GeofenceModule::handleInputEvent(const InputEvent *event)
{
    // Acknowledge/silence active alarm from the alert or status screen.
    if (runtime.alarmActive && isAlarmAcknowledgeEvent(event)) {
        state.alarmSilenced = true;
        clearAlarm(true);
        runtime.outsideConsecutiveCount = 0;
        runtime.timeoutConsecutiveCount = 0;
        saveState(true);
        notifyScreenUpdate(UIFrameEvent::Action::REDRAW_ONLY, false);
        return 1;
    }

    return 0;
}

void GeofenceModule::drawStatusLine(OLEDDisplay *display, int16_t x, int16_t y, int row, const char *label, const char *value)
{
    const int lineHeight = FONT_HEIGHT_SMALL;
    const int yPos = y + 12 + (row * lineHeight);
    char line[64];
    snprintf(line, sizeof(line), "%s: %s", label, value);
    display->drawString(x + 1, yPos, line);
}

void GeofenceModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *stateUi, int16_t x, int16_t y)
{
    (void)stateUi;

    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(FONT_SMALL);

    graphics::drawCommonHeader(display, x, y, "Geofence");

    drawStatusLine(display, x, y, 0, "Mode", state.armed ? "Armed" : "Disarmed");

    char targetStr[24];
    if (state.targetNode) {
        const meshtastic_NodeInfoLite *targetNode = nodeDB ? nodeDB->getMeshNode(state.targetNode) : nullptr;
        const char *targetName = nullptr;
        if (targetNode && targetNode->has_user) {
            if (targetNode->user.long_name[0]) {
                targetName = targetNode->user.long_name;
            } else if (targetNode->user.short_name[0]) {
                targetName = targetNode->user.short_name;
            }
        }

        if (targetName) {
            snprintf(targetStr, sizeof(targetStr), "%s", targetName);
        } else {
            snprintf(targetStr, sizeof(targetStr), "!%08x", state.targetNode);
        }
    } else {
        strncpy(targetStr, "none", sizeof(targetStr));
    }
    drawStatusLine(display, x, y, 1, "Target", targetStr);

    char radiusStr[24];
    snprintf(radiusStr, sizeof(radiusStr), "%um", state.radiusMeters);
    drawStatusLine(display, x, y, 2, "Radius", radiusStr);

    char distStr[24];
    if (runtime.lastDistanceMeters >= 0) {
        snprintf(distStr, sizeof(distStr), "%dm", runtime.lastDistanceMeters);
    } else {
        strncpy(distStr, "n/a", sizeof(distStr));
    }
    drawStatusLine(display, x, y, 3, "Distance", distStr);

    char alarmStr[24];
    if (runtime.alarmActive) {
        const char *alarmLabel = runtime.alarmType == AlarmType::Offline ? "OFFLINE" : "ALARM";
        snprintf(alarmStr, sizeof(alarmStr), state.alarmSilenced ? "%s (muted)" : "%s", alarmLabel);
    } else {
        strncpy(alarmStr, "ok", sizeof(alarmStr));
    }
    drawStatusLine(display, x, y, 4, "Alarm", alarmStr);
}
#endif
