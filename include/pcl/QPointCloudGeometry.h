#ifndef QPOINTCLOUDGEOMETRY_H
#define QPOINTCLOUDGEOMETRY_H

#include "pcl/QPointCloud.h"
#include <Qt3DRender/qgeometry.h>

class QPointCloudGeometryPrivate;

class QPointCloudGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
    Q_PROPERTY(QPointCloud *pointCloud READ pointCloud WRITE setPointCloud NOTIFY pointCloudChanged)

public:
    explicit QPointCloudGeometry(QNode *parent = NULL);
    ~QPointCloudGeometry();
    void updateVertices();

    QPointCloud *pointCloud() const;

public slots:
    void setPointCloud(QPointCloud *pointcloud);

private slots:
    void updateAttributes();

signals:
    void pointCloudChanged(QPointCloud *pointcloud);

private:
    QPointCloudGeometryPrivate *m_p;
};

#endif // QPOINTCLOUDGEOMETRY_H
