#ifndef QPCLVISUAL_H
#define QPCLVISUAL_H

#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include <QObject>
#include <sophus/se3.hpp>
#include <boost/shared_ptr.hpp>

enum PCL_VISUAL_MODE {
    INTERACTIVE_MODE,
    RENDER_MODE
};

class QPCLVisual : public QObject
{
    Q_OBJECT
public:
    explicit QPCLVisual(QObject *parent = nullptr);
    ~QPCLVisual();

    void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

    double curTime();

public slots:
    void renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

signals:
    void signalShowPtsFinished();

public:
    // pcl visualize

    pcl::visualization::PCLVisualizer::Ptr viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_allPts;

    std::size_t m_count;

public:
    PCL_VISUAL_MODE m_mode = PCL_VISUAL_MODE::RENDER_MODE;

};

#endif // QPCLVISUAL_H
