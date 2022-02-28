#ifndef QPOINTCLOUDVIEWER_H
#define QPOINTCLOUDVIEWER_H

#include <QObject>
#include <QWindow>

#include "AppConstant.h"
#include "openGL/QOpenGLCamera.h"
#include "AppEngine.h"

class QPointCloudRenderer;
class QOpenGLContext;
class QQuickWindow;
class QQuickRenderControl;
class QQmlComponent;
class QQuickItem;

class QPointCloudViewer : public QWindow
{
public:
    QPointCloudViewer(AppEngine *engine = nullptr, QWindow *parent = 0);
    ~QPointCloudViewer();


protected:
    void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;

    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *e) Q_DECL_OVERRIDE;

private:
    void syncScene();
    void draw();

    void onQmlComponentLoadingComplete();
    void updateRootItemSize();

    QPoint m_prevMousePosition;

    QOpenGLContext *m_context;

    QPointCloudRenderer *m_renderer;
    QOpenGLCamera *m_openGLcamera;

    QQuickRenderControl *m_renderControl;
    QQuickWindow *m_quickWindow;
    QQmlComponent *m_qmlComponent;
    QQuickItem *m_rootItem;

//    AppConstant *m_constant;
};

#endif // QPOINTCLOUDVIEWER_H
