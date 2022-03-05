#ifndef APPENUMS_H
#define APPENUMS_H

#include <QObject>

class AppEnums : public QObject
{
    Q_OBJECT
    Q_ENUMS(APP_STATE)
    Q_ENUMS(EVENT_t)
    Q_ENUMS(VIEW_SCREEN)

public:
    enum EVENT_t{
        EVT_NONE = 0,   // do not add enum above this
        EVT_CLICK_IMAGE_SCREEN,
        EVT_CLICK_RUN,
        EVT_CLICK_STOP,
        EVT_CLICK_OPENGL_RENDER,
        EVT_CLICK_RUN_SfM,
        EVT_CLICK_RESET,
        EVT_MAX, // do not add enum under this
    };

    enum VIEW_SCREEN{
            HOME = 0,
        };

    enum APP_STATE{
        STATE_NONE = 0,
        STATE_RUNNING ,
        STATE_STOP,
        STATE_RESET,
    };


private:
    AppEnums(const AppEnums& _other) = delete;
    void operator =(const AppEnums& _other) = delete;
};

#endif // APPENUMS_H
