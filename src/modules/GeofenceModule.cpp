#include "GeofenceModule.h"

#include "FSCommon.h"
#include "NodeDB.h"
#include "RTC.h"
#include "buzz/buzz.h"
#include "main.h"

GeofenceModule *geofenceModule;

namespace
{
constexpr uint32_t geofenceAlarmBeepIntervalMs = 1200;

const uint16_t radiusPresetsMeters[] = {25, 50, 100, 200, 500};

#if HAS_SCREEN
const char *mainMenuItems[] = {"Back", "Select target", "Set radius", "Arm/Reset", "Disarm"};
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

    setInterval(1000);
}

bool GeofenceModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p)
{
    if (!p) {
        return false;
    }

    if (getFrom(&mp) != state.targetNode) {
        return false;
    }

    updateMonitoringFromNodeDB();
    return false;
}

int32_t GeofenceModule::runOnce()
{
    updateMonitoringFromNodeDB();

    if (runtime.alarmActive && !state.alarmSilenced) {
        const uint32_t nowMs = millis();
        if (runtime.lastAlarmBeepMs == 0 || (nowMs - runtime.lastAlarmBeepMs) >= geofenceAlarmBeepIntervalMs) {
            playLongBeep();
            runtime.lastAlarmBeepMs = nowMs;
        }
    }

    return 1000;
}

void GeofenceModule::updateMonitoringFromNodeDB()
{
    runtime.lastDistanceMeters = -1;
    runtime.lastTargetPositionAgeSec = 0;

    if (!state.targetNode) {
        runtime.timeoutConsecutiveCount = 0;
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    meshtastic_NodeInfoLite *targetNode = nodeDB->getMeshNode(state.targetNode);
    if (!targetNode) {
        runtime.timeoutConsecutiveCount = 0;
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    runtime.lastTargetPositionAgeSec = sinceLastSeen(targetNode);

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

    if (!nodeDB->hasValidPosition(targetNode) || !state.hasCenter) {
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    GeoCoord fenceCenter(state.centerLat, state.centerLon, 0);
    GeoCoord targetPosition(targetNode->position.latitude_i, targetNode->position.longitude_i, 0);
    runtime.lastDistanceMeters = fenceCenter.distanceTo(targetPosition);

    if (!state.armed) {
        runtime.outsideConsecutiveCount = 0;
        return;
    }

    if (runtime.lastDistanceMeters > state.radiusMeters) {
        if (runtime.outsideConsecutiveCount < UINT8_MAX) {
            runtime.outsideConsecutiveCount++;
        }
        if (runtime.outsideConsecutiveCount >= state.outsideThreshold) {
            triggerAlarm();
        }
    } else {
        runtime.outsideConsecutiveCount = 0;
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

int GeofenceModule::handleInputEvent(const InputEvent *event)
{
    if (runtime.alarmActive && isSelectEvent(event)) {
        state.alarmSilenced = true;
        saveState();
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
        } else if (uiState == UI_STATE_TARGET_MENU) {
            const int itemCount = static_cast<int>(targetCandidates.size()) + 1;
            if (itemCount > 0) {
                targetMenuIndex = (targetMenuIndex - 1 + itemCount) % itemCount;
            }
        } else if (uiState == UI_STATE_RADIUS_MENU) {
            const int itemCount = radiusPresetCount + 1;
            targetMenuIndex = (targetMenuIndex - 1 + itemCount) % itemCount;
        }
        return 1;
    }

    if (isDownEvent(event)) {
        if (uiState == UI_STATE_MAIN_MENU) {
            menuIndex = (menuIndex + 1) % mainMenuCount;
        } else if (uiState == UI_STATE_TARGET_MENU) {
            const int itemCount = static_cast<int>(targetCandidates.size()) + 1;
            if (itemCount > 0) {
                targetMenuIndex = (targetMenuIndex + 1) % itemCount;
            }
        } else if (uiState == UI_STATE_RADIUS_MENU) {
            const int itemCount = radiusPresetCount + 1;
            targetMenuIndex = (targetMenuIndex + 1) % itemCount;
        }
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
        setUiState(UI_STATE_STATUS);
        break;
    case 1:
        setUiState(UI_STATE_TARGET_MENU);
        break;
    case 2:
        setUiState(UI_STATE_RADIUS_MENU);
        break;
    case 3:
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
    case 4:
        disarm();
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

    display->drawString(x + 1, y + 2, uiState == UI_STATE_MAIN_MENU ? "Geofence Menu" :
                                           (uiState == UI_STATE_TARGET_MENU ? "Select Target" : "Set Radius"));

    if (uiState == UI_STATE_MAIN_MENU) {
        for (int i = 0; i < mainMenuCount; ++i) {
            char line[32];
            snprintf(line, sizeof(line), "%c %s", (i == menuIndex ? '>' : ' '), mainMenuItems[i]);
            display->drawString(x + 1, y + 12 + (i * FONT_HEIGHT_SMALL), line);
        }
        return;
    }

    if (uiState == UI_STATE_TARGET_MENU) {
        char line[32];
        snprintf(line, sizeof(line), "%c Back", targetMenuIndex == 0 ? '>' : ' ');
        display->drawString(x + 1, y + 12, line);

        for (size_t i = 0; i < targetCandidates.size() && i < 4; ++i) {
            const int idx = static_cast<int>(i) + 1;
            char nodeLine[32];
            snprintf(nodeLine, sizeof(nodeLine), "%c !%08x", (targetMenuIndex == idx ? '>' : ' '), targetCandidates[i]);
            display->drawString(x + 1, y + 12 + (idx * FONT_HEIGHT_SMALL), nodeLine);
        }
        return;
    }

    if (uiState == UI_STATE_RADIUS_MENU) {
        char line[32];
        snprintf(line, sizeof(line), "%c Back", targetMenuIndex == 0 ? '>' : ' ');
        display->drawString(x + 1, y + 12, line);

        for (int i = 0; i < radiusPresetCount && i < 4; ++i) {
            const int idx = i + 1;
            char radiusLine[32];
            snprintf(radiusLine, sizeof(radiusLine), "%c %um", targetMenuIndex == idx ? '>' : ' ', radiusPresetsMeters[i]);
            display->drawString(x + 1, y + 12 + (idx * FONT_HEIGHT_SMALL), radiusLine);
        }
    }
}
#endif
