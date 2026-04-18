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

#if defined(USE_RF95) && defined(PIN_BUZZER) && (PIN_BUZZER == 19)
constexpr bool geofenceBuzzerAllowed = false;
#else
constexpr bool geofenceBuzzerAllowed = true;
#endif

const uint16_t radiusPresetsMeters[] = {25, 50, 100, 200, 500};

#if HAS_SCREEN
const char *mainMenuItems[] = {"Select target", "Set radius", "Arm/Reset", "Disarm", "Back"};
#endif
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

    // Check timeout condition only if armed
    if (state.armed && state.timeoutSeconds > 0 && runtime.lastTargetPositionAgeSec > state.timeoutSeconds) {
        if (runtime.timeoutConsecutiveCount < UINT8_MAX) {
            runtime.timeoutConsecutiveCount++;
        }
        if (runtime.timeoutConsecutiveCount >= state.timeoutThreshold) {
            triggerAlarm();
        }
    } else {
        runtime.timeoutConsecutiveCount = 0;
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
            triggerAlarm();
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

void GeofenceModule::triggerAlarm()
{
    if (runtime.alarmActive) {
        return;
    }

    runtime.alarmActive = true;
    runtime.lastAlarmBeepMs = 0;
#if HAS_SCREEN
    if (screen) {
        screen->startAlert("Geofence Alarm");
    }
#endif
}

void GeofenceModule::clearAlarm(bool keepSilenced)
{
    runtime.alarmActive = false;
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
void GeofenceModule::setUiState(UiState nextState)
{
    uiState = nextState;
    menuScrollOffset = 0;
    targetMenuScrollOffset = 0;
    radiusMenuScrollOffset = 0;
    
    if (uiState == UI_STATE_MAIN_MENU) {
        menuIndex = 0;
    } else if (uiState == UI_STATE_TARGET_MENU) {
        targetMenuIndex = 0;
        rebuildTargetCandidates();
    } else if (uiState == UI_STATE_RADIUS_MENU) {
        targetMenuIndex = 0;
        for (int i = 0; i < radiusPresetCount; ++i) {
            if (radiusPresetsMeters[i] == state.radiusMeters) {
                targetMenuIndex = i + 1;
                break;
            }
        }
    }

    requestFocus();
    UIFrameEvent event;
    event.action = UIFrameEvent::Action::REGENERATE_FRAMESET;
    notifyObservers(&event);
}

void GeofenceModule::rebuildTargetCandidates()
{
    targetCandidates.clear();
    if (!nodeDB || !nodeDB->meshNodes) {
        return;
    }

    for (const auto &node : *nodeDB->meshNodes) {
        if (node.num == 0 || node.num == nodeDB->getNodeNum() || !node.has_position || !nodeDB->hasValidPosition(&node)) {
            continue;
        }
        targetCandidates.push_back(node.num);
    }

    if (targetCandidates.size() > 4) {
        targetCandidates.resize(4);
    }

    std::sort(targetCandidates.begin(), targetCandidates.end());

    selectedNodeIndex = 0;
    for (size_t i = 0; i < targetCandidates.size(); ++i) {
        if (targetCandidates[i] == state.targetNode) {
            selectedNodeIndex = static_cast<int>(i);
            break;
        }
    }
}

bool GeofenceModule::isUpEvent(const InputEvent *event) const
{
    return event->inputEvent == INPUT_BROKER_UP || event->inputEvent == INPUT_BROKER_ALT_PRESS ||
           event->inputEvent == INPUT_BROKER_LEFT;
}

bool GeofenceModule::isDownEvent(const InputEvent *event) const
{
    return event->inputEvent == INPUT_BROKER_DOWN || event->inputEvent == INPUT_BROKER_USER_PRESS ||
           event->inputEvent == INPUT_BROKER_RIGHT;
}

bool GeofenceModule::isSelectEvent(const InputEvent *event) const
{
    return event->inputEvent == INPUT_BROKER_SELECT;
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
        saveState();
#if HAS_SCREEN
        if (screen) {
            screen->endAlert();
        }
#endif
    UIFrameEvent uiEvent;
    uiEvent.action = UIFrameEvent::Action::REDRAW_ONLY;
    notifyObservers(&uiEvent);
        return 1;
    }

    if (uiState == UI_STATE_STATUS) {
        if (isSelectEvent(event)) {
            setUiState(UI_STATE_MAIN_MENU);
            return 1;
        }
        return 0;
    }

    if (isUpEvent(event)) {
        if (uiState == UI_STATE_MAIN_MENU) {
            menuIndex = (menuIndex - 1 + mainMenuCount) % mainMenuCount;
            // Update scroll offset to keep selection visible
            if (menuIndex < menuScrollOffset) {
                menuScrollOffset = menuIndex;
            } else if (menuIndex >= menuScrollOffset + maxVisibleMenuItems) {
                menuScrollOffset = menuIndex - maxVisibleMenuItems + 1;
            }
        } else if (uiState == UI_STATE_TARGET_MENU) {
            const int itemCount = static_cast<int>(targetCandidates.size()) + 1;
            if (itemCount > 0) {
                targetMenuIndex = (targetMenuIndex - 1 + itemCount) % itemCount;
                if (targetMenuIndex < targetMenuScrollOffset) {
                    targetMenuScrollOffset = targetMenuIndex;
                } else if (targetMenuIndex >= targetMenuScrollOffset + maxVisibleMenuItems) {
                    targetMenuScrollOffset = targetMenuIndex - maxVisibleMenuItems + 1;
                }
            }
        } else if (uiState == UI_STATE_RADIUS_MENU) {
            const int itemCount = radiusPresetCount + 1;
            targetMenuIndex = (targetMenuIndex - 1 + itemCount) % itemCount;
            if (targetMenuIndex < radiusMenuScrollOffset) {
                radiusMenuScrollOffset = targetMenuIndex;
            } else if (targetMenuIndex >= radiusMenuScrollOffset + maxVisibleMenuItems) {
                radiusMenuScrollOffset = targetMenuIndex - maxVisibleMenuItems + 1;
            }
        }
        UIFrameEvent uiEvent;
        uiEvent.action = UIFrameEvent::Action::REDRAW_ONLY;
        notifyObservers(&uiEvent);
        return 1;
    }

    if (isDownEvent(event)) {
        if (uiState == UI_STATE_MAIN_MENU) {
            menuIndex = (menuIndex + 1) % mainMenuCount;
            if (menuIndex < menuScrollOffset) {
                menuScrollOffset = menuIndex;
            } else if (menuIndex >= menuScrollOffset + maxVisibleMenuItems) {
                menuScrollOffset = menuIndex - maxVisibleMenuItems + 1;
            }
        } else if (uiState == UI_STATE_TARGET_MENU) {
            const int itemCount = static_cast<int>(targetCandidates.size()) + 1;
            if (itemCount > 0) {
                targetMenuIndex = (targetMenuIndex + 1) % itemCount;
                if (targetMenuIndex < targetMenuScrollOffset) {
                    targetMenuScrollOffset = targetMenuIndex;
                } else if (targetMenuIndex >= targetMenuScrollOffset + maxVisibleMenuItems) {
                    targetMenuScrollOffset = targetMenuIndex - maxVisibleMenuItems + 1;
                }
            }
        } else if (uiState == UI_STATE_RADIUS_MENU) {
            const int itemCount = radiusPresetCount + 1;
            targetMenuIndex = (targetMenuIndex + 1) % itemCount;
            if (targetMenuIndex < radiusMenuScrollOffset) {
                radiusMenuScrollOffset = targetMenuIndex;
            } else if (targetMenuIndex >= radiusMenuScrollOffset + maxVisibleMenuItems) {
                radiusMenuScrollOffset = targetMenuIndex - maxVisibleMenuItems + 1;
            }
        }
        UIFrameEvent uiEvent;
        uiEvent.action = UIFrameEvent::Action::REDRAW_ONLY;
        notifyObservers(&uiEvent);
        return 1;
    }

    if (isSelectEvent(event)) {
        if (uiState == UI_STATE_MAIN_MENU) {
            handleMainMenuSelect();
        } else if (uiState == UI_STATE_TARGET_MENU) {
            handleTargetMenuSelect();
        } else if (uiState == UI_STATE_RADIUS_MENU) {
            handleRadiusMenuSelect();
        }
        return 1;
    }

    return 0;
}

void GeofenceModule::handleMainMenuSelect()
{
    switch (menuIndex) {
    case 0:
        setUiState(UI_STATE_TARGET_MENU);
        break;
    case 1:
        setUiState(UI_STATE_RADIUS_MENU);
        break;
    case 2:
        if (refreshCenterFromTargetPosition()) {
            state.armed = true;
            state.alarmSilenced = false;
            runtime.timeoutConsecutiveCount = 0;
            runtime.outsideConsecutiveCount = 0;
            clearAlarm();
            saveState(true);
        }
        setUiState(UI_STATE_STATUS);
        break;
    case 3:
        disarm();
        setUiState(UI_STATE_STATUS);
        break;
    case 4:
        setUiState(UI_STATE_STATUS);
        break;
    default:
        break;
    }
}

void GeofenceModule::handleTargetMenuSelect()
{
    if (targetMenuIndex == 0) {
        setUiState(UI_STATE_MAIN_MENU);
        return;
    }

    const int candidateIndex = targetMenuIndex - 1;
    if (candidateIndex >= 0 && candidateIndex < static_cast<int>(targetCandidates.size())) {
        state.targetNode = targetCandidates[candidateIndex];
        saveState(true);
    }
    setUiState(UI_STATE_MAIN_MENU);
}

void GeofenceModule::handleRadiusMenuSelect()
{
    if (targetMenuIndex == 0) {
        setUiState(UI_STATE_MAIN_MENU);
        return;
    }

    const int presetIndex = targetMenuIndex - 1;
    if (presetIndex >= 0 && presetIndex < radiusPresetCount) {
        state.radiusMeters = radiusPresetsMeters[presetIndex];
        saveState(true);
    }
    setUiState(UI_STATE_MAIN_MENU);
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

    if (uiState == UI_STATE_STATUS) {
        drawStatusLine(display, x, y, 0, "Mode", state.armed ? "Armed" : "Disarmed");

        char targetStr[24];
        if (state.targetNode) {
            snprintf(targetStr, sizeof(targetStr), "!%08x", state.targetNode);
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
            snprintf(alarmStr, sizeof(alarmStr), state.alarmSilenced ? "ALARM (muted)" : "ALARM");
        } else {
            strncpy(alarmStr, "ok", sizeof(alarmStr));
        }
        drawStatusLine(display, x, y, 4, "Alarm", alarmStr);

        return;
    }

    if (uiState == UI_STATE_MAIN_MENU) {
        int startIdx = menuScrollOffset;
        int endIdx = std::min(startIdx + maxVisibleMenuItems, mainMenuCount);
        
        for (int i = startIdx; i < endIdx; ++i) {
            const int displayRow = i - startIdx;
            char line[32];
            snprintf(line, sizeof(line), "%c %s", (i == menuIndex ? '>' : ' '), mainMenuItems[i]);
            display->drawString(x + 1, y + 12 + (displayRow * FONT_HEIGHT_SMALL), line);
        }
        return;
    }

    if (uiState == UI_STATE_TARGET_MENU) {
        int startIdx = targetMenuScrollOffset;
        int itemCount = static_cast<int>(targetCandidates.size()) + 1; // +1 for "Back"
        int endIdx = std::min(startIdx + maxVisibleMenuItems, itemCount);
        
        for (int i = startIdx; i < endIdx; ++i) {
            const int displayRow = i - startIdx;
            char line[32];
            
            if (i == 0) {
                // Back option
                snprintf(line, sizeof(line), "%c Back", (targetMenuIndex == 0 ? '>' : ' '));
            } else {
                const int candidateIdx = i - 1;
                if (candidateIdx < static_cast<int>(targetCandidates.size())) {
                    snprintf(line, sizeof(line), "%c !%08x", (targetMenuIndex == i ? '>' : ' '), 
                             targetCandidates[candidateIdx]);
                }
            }
            display->drawString(x + 1, y + 12 + (displayRow * FONT_HEIGHT_SMALL), line);
        }
        return;
    }

    if (uiState == UI_STATE_RADIUS_MENU) {
        int startIdx = radiusMenuScrollOffset;
        int itemCount = radiusPresetCount + 1; // +1 for "Back"
        int endIdx = std::min(startIdx + maxVisibleMenuItems, itemCount);
        
        for (int i = startIdx; i < endIdx; ++i) {
            const int displayRow = i - startIdx;
            char line[32];
            
            if (i == 0) {
                // Back option
                snprintf(line, sizeof(line), "%c Back", (targetMenuIndex == 0 ? '>' : ' '));
            } else {
                const int presetIdx = i - 1;
                if (presetIdx < radiusPresetCount) {
                    snprintf(line, sizeof(line), "%c %um", (targetMenuIndex == i ? '>' : ' '), 
                             radiusPresetsMeters[presetIdx]);
                }
            }
            display->drawString(x + 1, y + 12 + (displayRow * FONT_HEIGHT_SMALL), line);
        }
    }
}
#endif
