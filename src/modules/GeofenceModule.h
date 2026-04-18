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

#if HAS_SCREEN
        void setTargetNode(uint32_t nodenum);
        void setRadiusMeters(uint16_t meters);
        void armOrReset();
        void disarmFromMenu();
    bool isModuleFrame(const MeshModule *module) const;
        uint16_t getRadiusMeters() const { return state.radiusMeters; }
        bool isArmed() const { return state.armed; }
#endif

  protected:
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p) override;
    virtual int32_t runOnce() override;

#if HAS_SCREEN
    virtual bool wantUIFrame() override { return true; }
    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
    virtual Observable<const UIFrameEvent *> *getUIFrameObservable() override { return this; }
    virtual bool interceptingKeyboardInput() override { return false; }

    int handleInputEvent(const InputEvent *event);
#endif

  private:
        enum class AlarmType : uint8_t {
                None,
                Geofence,
                Offline,
        };

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
        AlarmType alarmType = AlarmType::None;
        uint8_t outsideConsecutiveCount = 0;
        uint8_t timeoutConsecutiveCount = 0;
        int32_t lastDistanceMeters = -1;
        uint32_t lastTargetPositionAgeSec = 0;
        uint32_t lastAlarmBeepMs = 0;
        uint32_t lastSavedMs = 0;
    };

#if HAS_SCREEN
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
    void triggerAlarm(AlarmType alarmType);
    void clearAlarm(bool keepSilenced = false);

    static const char *stateFilePath();

#if HAS_SCREEN
    void notifyScreenUpdate(UIFrameEvent::Action action, bool focusFrame);
    void drawStatusLine(OLEDDisplay *display, int16_t x, int16_t y, int row, const char *label, const char *value);
#endif
};

extern GeofenceModule *geofenceModule;
