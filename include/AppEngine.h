#ifndef APPENGINE_H
#define APPENGINE_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "AppEnums.h"
#include "AppModel.h"

class AppEngine : public QQmlApplicationEngine
{
    Q_OBJECT
public:
    QQmlContext* m_rootContext;

public:
    explicit AppEngine();
    ~AppEngine();

    void initEngine();
    void startEngine();

    void pointCloudRenderScreenRun(AppEngine *engine);
public slots:
    void slotReceiveEvent(int event);
signals:
};

#endif // APPENGINE_H
