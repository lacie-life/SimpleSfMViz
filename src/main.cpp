#include <QGuiApplication>

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <qqml.h>
#include <QDebug>
#include <QSurfaceFormat>

#if WITH_OPENGL
#include "openGL/QPointCloudViewer.h"
#include "openGL/QPointCloudUnderlay.h"
#endif

#if WITH_PCL
#include "pcl/QPointCloud.h"
#include "pcl/QPointCloudGeometry.h"
#include "pcl/QPointField.h"
#include "pcl/QPointCloudLoader.h"
#endif

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);
#if WITH_OPENGL
    QPointCloudViewer w;
    w.resize(600, 600);
    w.show();
#endif

#if WITH_PCL
    QSurfaceFormat fmt;
    fmt.setVersion(1, 4);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    qmlRegisterType<QPointCloudLoader>("pcl", 1, 0, "PointCloudLoader");
    qmlRegisterType<QPointCloud>("pcl", 1, 0, "PointCloud");
    qmlRegisterType<QPointCloudGeometry>("pcl", 1, 0, "PointCloudGeometry");
    qmlRegisterUncreatableType<QPointField>("pcl", 1, 0, "PointField", "Can not yet be created in qml, use PointCloudLoader.");
    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qml/qml/pointcloud.qml")));
#endif
    return app.exec();
}
