#ifndef QPOINTCLOUDVIEWER_H
#define QPOINTCLOUDVIEWER_H

#include <QQuickView>


class QPointCloudViewer : public QQuickView
{
    Q_OBJECT

public:
    explicit  QPointCloudViewer(QWindow *parent = 0);

};


#endif // QPOINTCLOUDVIEWER_H
