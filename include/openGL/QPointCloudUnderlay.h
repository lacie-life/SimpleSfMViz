#ifndef QPOINTCLOUDUNDERLAY_H
#define QPOINTCLOUDUNDERLAY_H

#include <QObject>
#include <QQuickView>

class QPointCloudRenderer;
class QCameraControl;

class QPointCloudUnderlay : public QQuickView
{
    Q_OBJECT
public:
    QPointCloudUnderlay(QWindow *parent = 0);

private:
    void initializeUnderlay();
    void synchronizeUnderlay();
    void renderUnderlay();
    void invalidateUnderlay();

    QCameraControl *m_camera;
    QPointCloudRenderer *m_renderer;
};

#endif // QPOINTCLOUDUNDERLAY_H
