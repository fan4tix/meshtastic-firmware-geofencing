#pragma once

#include "NodeDB.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "gps/GeoCoord.h"

#if HAS_SCREEN
#include "Observer.h"
#include "input/InputBroker.h"
#endif

class GeofenceModule : public ProtobufModule<meshtastic_Position>, private concurrency::OSThread
#if HAS_SCREEN
                     , public Observable<const UIFrameEvent *>
#endif
{
  public:
    GeofenceModule();

  protected:
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p) override;
    virtual int32_t runOnce() override;

#if HAS_SCREEN
    virtual bool wantUIFrame() override { return true; }
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
    virtual Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
    virtual bool interceptingKeyboardInput() override { return uiState != UI_STATE_STATUS; }

    int handleInputEvent(const InputEvent *event);
#endif

  private:
    static constexpr uint32_t GEOFENCE_MAGIC = 0x47464E43; // GFNC
    static constexpr uint16_t GEOFENCE_STATE_VERSION = 1;

    static constexpr uint32_t defaultRadiusMeters = 100;
    static constexpr uint32_t defaultTimeoutSeconds = 120;
    static constexpr uint8_t defaultOutsideThreshold = 3;
    static constexpr uint8_t defaultTimeoutThreshold = 3;

    struct PersistedState {
        uint32_t magic;
        uint16_t version;
        uint16_t radiusMeters;
        uint16_t timeoutSeconds;
        uint8_t outsideThreshold;
        uint8_t timeoutThreshold;
        bool armed;
        bool hasCenter;
        bool alarmSilenced;
        uint32_t targetNode;
        int32_t centerLat;
        int32_t centerLon;
    };

    struct RuntimeState {
        bool alarmActive = false;
        uint8_t outsideConsecutiveCount = 0;
        uint8_t timeoutConsecutiveCount = 0;
        int32_t lastDistanceMeters = -1;
        uint32_t lastTargetPositionAgeSec = 0;
        uint32_t lastAlarmBeepMs = 0;
        uint32_t lastSavedMs = 0;
    };

#if HAS_SCREEN
    enum UiState : uint8_t {
        UI_STATE_STATUS,
        UI_STATE_MAIN_MENU,
        UI_STATE_TARGET_MENU,
        UI_STATE_RADIUS_MENU,
    };

    static constexpr int mainMenuCount = 5;
    static constexpr int radiusPresetCount = 5;
    static constexpr int maxVisibleMenuItems = 4;

    UiState uiState = UI_STATE_STATUS;
    int menuIndex = 0;
    int menuScrollOffset = 0;
    int targetMenuIndex = 0;
    int targetMenuScrollOffset = 0;
    int radiusMenuScrollOffset = 0;
    int selectedNodeIndex = 0;
    std::vector<uint32_t> targetCandidates;

    CallbackObserver<GeofenceModule, const InputEvent *> inputObserver =
        CallbackObserver<GeofenceModule, const InputEvent *>(this, &GeofenceModule::handleInputEvent);
#endif

    PersistedState state = {
        .magic = GEOFENCE_MAGIC,
        .version = GEOFENCE_STATE_VERSION,
        .radiusMeters = defaultRadiusMeters,
        .timeoutSeconds = defaultTimeoutSeconds,
        .outsideThreshold = defaultOutsideThreshold,
        .timeoutThreshold = defaultTimeoutThreshold,
        .armed = false,
        .hasCenter = false,
        .alarmSilenced = false,
        .targetNode = 0,
        .centerLat = 0,
        .centerLon = 0,
    };

    RuntimeState runtime;

    void loadState();
    void saveState(bool force = false);

    void updateMonitoringFromNodeDB();
    bool refreshCenterFromTargetPosition();
    void disarm();
    void triggerAlarm();
    void clearAlarm(bool keepSilenced = false);

    static const char *stateFilePath();

#if HAS_SCREEN
    void setUiState(UiState nextState);
    void rebuildTargetCandidates();
    void drawStatusLine(OLEDDisplay *display, int16_t x, int16_t y, int row, const char *label, const char *value);
    bool isUpEvent(const InputEvent *event) const;
    bool isDownEvent(const InputEvent *event) const;
    bool isSelectEvent(const InputEvent *event) const;
    void handleMainMenuSelect();
    void handleTargetMenuSelect();
    void handleRadiusMenuSelect();
#endif
};

extern GeofenceModule *geofenceModule;
