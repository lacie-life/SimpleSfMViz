#include "QPointCloudViewer.h"
#include "QPointCloudRenderer.h"
#include "QCameraControl.h"
#include "Constant.h"

#include <QSurfaceFormat>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

#include <QQuickWindow>
#include <QQuickRenderControl>
#include <QQuickItem>

#include <QQmlEngine>
#include <QQmlComponent>
#include <QQmlContext>

#include <QTimer>

QPointCloudViewer::QPointCloudViewer(QWindow *parent)
    : QWindow(parent)
    , m_context(0)
    , m_renderer(0)
    , m_renderControl(0)
    , m_quickWindow(0)
    , m_qmlComponent(0)
    , m_rootItem(0)
{
    // set the window up
    setSurfaceType(QSurface::OpenGLSurface);

    QSurfaceFormat format;
    format.setMajorVersion(3);
    format.setMinorVersion(3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setSamples(4);

    setFormat(format);
    create();

    QString plyPath = "/home/jun/Github/GreenHouseAR/assest/bunny.ply";

    // create the GL context

    m_context = new QOpenGLContext(this);
    m_context->setFormat(format);
    if (!m_context->create())
        qFatal("Unable to create context");

    m_context->makeCurrent(this);

    // set up our stuff

    m_renderer = new QPointCloudRenderer(this);
    m_renderer->initialize(plyPath);

    m_camera = new QCameraControl(this);

    m_camera->setPosition(QVector3D(0, 0.2, 0.2));
    m_camera->rotate(0, 50, 0);

    // set up QtQuick

    m_renderControl = new QQuickRenderControl(this);
    m_quickWindow = new QQuickWindow(m_renderControl);
    m_quickWindow->setClearBeforeRendering(false);

    // try to "batch" multiple scene changed signals in one sync
    QTimer *sceneSyncTimer = new QTimer(this);
    sceneSyncTimer->setInterval(5);
    sceneSyncTimer->setSingleShot(true);
    connect(sceneSyncTimer, &QTimer::timeout,
            this, &QPointCloudViewer::syncScene);

    connect(m_renderControl, &QQuickRenderControl::sceneChanged,
            sceneSyncTimer, static_cast<void (QTimer::*)()>(&QTimer::start));

    connect(m_renderControl, &QQuickRenderControl::renderRequested,
            this, &QPointCloudViewer::draw);

    m_renderControl->initialize(m_context);


    // load a QML scene "manually"
    QQmlEngine *engine = new QQmlEngine(this);

    if (!engine->incubationController())
        engine->setIncubationController(m_quickWindow->incubationController());

    engine->rootContext()->setContextProperty("_camera", m_camera);
//    engine->rootContext()->setContextProperty("QmlConstant", DEFS);
    m_qmlComponent = new QQmlComponent(engine, this);

    connect(m_qmlComponent, &QQmlComponent::statusChanged,
            this, &QPointCloudViewer::onQmlComponentLoadingComplete);

    m_qmlComponent->loadUrl(QUrl("/home/jun/Github/GreenHouseAR/assest/main.qml"));

    // also, just for the sake of it, trigger a redraw every 500 ms no matter what
    QTimer *redrawTimer = new QTimer(this);
    connect(redrawTimer, &QTimer::timeout, this, &QPointCloudViewer::draw);
    redrawTimer->start(500);
}

QPointCloudViewer::~QPointCloudViewer()
{
    m_context->makeCurrent(this);

    m_renderer->invalidate();
    delete m_renderer;

    delete m_rootItem;
    delete m_qmlComponent;
    delete m_renderControl;
    delete m_quickWindow;

    m_context->doneCurrent();
    delete m_context;
}

void QPointCloudViewer::syncScene()
{
    m_renderControl->polishItems();

    m_renderer->setPosition(m_camera->position());

    m_renderer->setxRotation(m_camera->xRotation());
    m_renderer->setyRotation(m_camera->yRotation());
    m_renderer->setzRotation(m_camera->zRotation());

    m_renderer->setFrontClippingPlaneDistance(m_camera->frontClippingPlaneDistance());
    m_renderer->setRearClippingDistance(m_camera->rearClippingDistance());

    m_renderControl->sync();
    draw();
}

void QPointCloudViewer::draw()
{
    if (!isExposed())
        return;
    m_context->makeCurrent(this);
    m_context->functions()->glViewport(0, 0, width() * devicePixelRatio(), height() * devicePixelRatio());

    m_renderer->render();
    m_quickWindow->resetOpenGLState();

    m_renderControl->render();

    m_context->swapBuffers(this);
}

void QPointCloudViewer::onQmlComponentLoadingComplete()
{
    if (m_qmlComponent->isLoading())
        return;
    if (m_qmlComponent->isError()) {
        const QList<QQmlError> errorList = m_qmlComponent->errors();
        foreach (const QQmlError &error, errorList)
            qWarning() << error.url() << error.line() << error;

        qFatal("Unable to load QML file");
    }

    QObject *rootObject = m_qmlComponent->create();
    m_rootItem = qobject_cast<QQuickItem *>(rootObject);
    if (!m_rootItem)
        qFatal("Did not load a Qt Quick scene");

    m_rootItem->setParentItem(m_quickWindow->contentItem());
}

void QPointCloudViewer::resizeEvent(QResizeEvent *e)
{
    // Simulate the "resize root item to follow window"
    updateRootItemSize();
    QWindow::resizeEvent(e);
}

void QPointCloudViewer::updateRootItemSize()
{
    if (m_rootItem) {
        m_rootItem->setWidth(width());
        m_rootItem->setHeight(height());
    }

    m_quickWindow->setHeight(height());
    m_quickWindow->setWidth(width());
}

void QPointCloudViewer::mousePressEvent(QMouseEvent *e)
{
    qApp->sendEvent(m_quickWindow, e);
    if (!e->isAccepted())
        QWindow::mousePressEvent(e);
}

void QPointCloudViewer::mouseMoveEvent(QMouseEvent *e)
{
    qApp->sendEvent(m_quickWindow, e);
    if (!e->isAccepted())
        QWindow::mousePressEvent(e);
}

void QPointCloudViewer::mouseReleaseEvent(QMouseEvent *e)
{
    qApp->sendEvent(m_quickWindow, e);
    if (!e->isAccepted())
        QWindow::mousePressEvent(e);
}
