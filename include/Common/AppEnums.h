#ifndef APPENUMS_H
#define APPENUMS_H

#include <QObject>
#include <QMap>

class AppEnums : public QObject
{
    Q_OBJECT
    Q_ENUMS(APP_STATE)
    Q_ENUMS(EVENT_t)
    Q_ENUMS(VIEW_SCREEN)

    Q_ENUMS(DETECT_MODEL)

public:
    enum EVENT_t {
        EVT_NONE = 0,   // do not add enum above this
        EVT_CLICK_HOME_SCREEN,
        EVT_CLICK_PROCESS_SCREEN,
        EVT_CLICK_CHOOSE_ROSBAG,
        EVT_CLICK_CHOOSE_CONFIG,
        EVT_CLICK_RUN,
        EVT_CLICK_CAMERA_RUN,
        EVT_CLICK_SFM_RUN,
        EVT_CLICK_SIMPLESFM_RUN,
        EVT_CLICK_STOP,
        EVT_CLICK_OPENGL_RENDER,
        EVT_CLICK_RUN_SfM,
        EVT_CLICK_RESET,
        EVT_MAX, // do not add enum under this
    };

    enum VIEW_SCREEN {
        HOME_SCREEN = 0,
        PROCESS_SCREEN,
        MAX_SCREEN,
    };

    enum APP_STATE {
        STATE_NONE = 0,
        STATE_RUNNING ,
        STATE_STOP,
        STATE_RESET,
        STATE_MAX,
    };

    enum DETECT_MODEL {
        YOLO = 0,
        SSD,
        MODEL_MAX,
    };

    static QMap<int, QString> MODEL_ZOO;

private:
    AppEnums(const AppEnums& _other) = delete;
    void operator =(const AppEnums& _other) = delete;
};

#endif // APPENUMS_H
