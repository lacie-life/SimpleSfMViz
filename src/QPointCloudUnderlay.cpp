#include "QPointCloudUnderlay.h"
#include "QPointCloudRenderer.h"
#include "QCameraControl.h"

#include <QSurfaceFormat>
#include <QQmlContext>

QPointCloudUnderlay::QPointCloudUnderlay(QWindow *parent)
    : QQuickView(parent)
    , m_camera(new QCameraControl(this))
    , m_renderer(new QPointCloudRenderer(this))
{
    QSurfaceFormat format;
    format.setMajorVersion(3);
    format.setMinorVersion(3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);
    setFormat(format);

    connect(this, &QQuickWindow::sceneGraphInitialized,
            this, &QPointCloudUnderlay::initializeUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::beforeSynchronizing,
            this, &QPointCloudUnderlay::synchronizeUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::beforeRendering,
            this, &QPointCloudUnderlay::renderUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::sceneGraphInvalidated,
            this, &QPointCloudUnderlay::invalidateUnderlay,
            Qt::DirectConnection);

    connect(m_camera, &QCameraControl::positionChanged, this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::xRotationChanged, this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::yRotationChanged, this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::zRotationChanged, this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::frontClippingPlaneDistanceChanged, this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::rearClippingDistanceChanged, this, &QQuickWindow::update);

    setClearBeforeRendering(false);
    setPersistentOpenGLContext(true);

    setResizeMode(SizeRootObjectToView);
    rootContext()->setContextProperty("_camera", m_camera);
    setSource(QUrl("/home/jun/Github/GreenHouseAR/assest/main.qml"));

}

void QPointCloudUnderlay::initializeUnderlay()
{
    QString plyPath = "/home/jun/Github/GreenHouseAR/assest/bunny.ply";

    m_renderer->initialize(plyPath);
    m_camera->setPosition(QVector3D(0, -0.1, -0.2));
    m_camera->rotate(0, 50, 0);
    resetOpenGLState();
}

void QPointCloudUnderlay::synchronizeUnderlay()
{
    m_renderer->setPosition(m_camera->position());

    m_renderer->setxRotation(m_camera->xRotation());
    m_renderer->setyRotation(m_camera->yRotation());
    m_renderer->setzRotation(m_camera->zRotation());

    m_renderer->setFrontClippingPlaneDistance(m_camera->frontClippingPlaneDistance());
    m_renderer->setRearClippingDistance(m_camera->rearClippingDistance());
}

void QPointCloudUnderlay::renderUnderlay()
{
    m_renderer->render();
    resetOpenGLState();
}

void QPointCloudUnderlay::invalidateUnderlay()
{
    m_renderer->invalidate();
    resetOpenGLState();
}
