#include <QGuiApplication>

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QDebug>
#include <QSurfaceFormat>

#include "openGL/QPointCloudViewer.h"
#include "openGL/QPointCloudUnderlay.h"



int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);

    QPointCloudViewer w;
    w.resize(600, 600);
    w.show();

    return app.exec();
}
