#ifndef QPOINTCLOUDVIEWER_H
#define QPOINTCLOUDVIEWER_H

#include <QWindow>
#include <QQuickView>

class QPointCloudRenderer;
class QCameraControl;
class QOpenGLContext;
class QQuickWindow;
class QQuickRenderControl;
class QQmlComponent;
class QQuickItem;


class QPointCloudViewer : public QWindow
{
    Q_OBJECT

public:
    explicit  QPointCloudViewer(QWindow *parent = 0);
    ~QPointCloudViewer();

protected:
    void resizeEvent(QResizeEvent *e) Q_DECL_OVERRIDE;

    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE;

private:
    void syncScene();
    void draw();

    void onQmlComponentLoadingComplete();
    void updateRootItemSize();

    QOpenGLContext *m_context;

    QPointCloudRenderer *m_renderer;
    QCameraControl *m_camera;

    QQuickRenderControl *m_renderControl;
    QQuickWindow *m_quickWindow;
    QQmlComponent *m_qmlComponent;
    QQuickItem *m_rootItem;

};


#endif // QPOINTCLOUDVIEWER_H
