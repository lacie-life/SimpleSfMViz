#include "QPointCloudViewer.h"
#include "QPointCloudRenderer.h"
#include "QCameraControl.h"

#include <QSurfaceFormat>
#include <QQmlContext>

QPointCloudViewer::QPointCloudViewer(QWindow *parent)
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
            this, &QPointCloudViewer::initializeUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::beforeSynchronizing,
            this, &QPointCloudViewer::synchronizeUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::beforeRendering,
            this, &QPointCloudViewer::renderUnderlay,
            Qt::DirectConnection);

    connect(this, &QQuickWindow::sceneGraphInvalidated,
            this, &QPointCloudViewer::invalidateUnderlay,
            Qt::DirectConnection);

    connect(m_camera, &QCameraControl::positionChanged,
            this, &QQuickWindow::update);

    connect(m_camera, &QCameraControl::xRotationChanged,
            this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::yRotationChanged,
            this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::zRotationChanged,
            this, &QQuickWindow::update);

    connect(m_camera, &QCameraControl::frontClippingPlaneDistanceChanged,
            this, &QQuickWindow::update);
    connect(m_camera, &QCameraControl::rearClippingDistanceChanged,
            this, &QQuickWindow::update);

    setClearBeforeRendering(false);
    setPersistentOpenGLContext(true);

    setResizeMode(SizeRootObjectToView);
    rootContext()->setContextProperty("_camera", m_camera);
    setSource(QUrl("qrc:///qml/main.qml"));
}

void QPointCloudViewer::initializeUnderlay()
{
    QString plyPath = "/home/jun/Github/GreenHouseAR/assest/bunny.ply";
    m_renderer->initialize(plyPath);
    resetOpenGLState();
}

void QPointCloudViewer::synchronizeUnderlay()
{
    m_renderer->setPosition(m_camera->position());

    m_renderer->setxRotation(m_camera->xRotation());
    m_renderer->setyRotation(m_camera->yRotation());
    m_renderer->setzRotation(m_camera->zRotation());

    m_renderer->setFrontClippingPlaneDistance(m_camera->frontClippingPlaneDistance());
    m_renderer->setRearClippingDistance(m_camera->rearClippingDistance());
}

void QPointCloudViewer::renderUnderlay()
{
    m_renderer->render();
    resetOpenGLState();
}

void QPointCloudViewer::invalidateUnderlay()
{
    m_renderer->invalidate();
    resetOpenGLState();
}


