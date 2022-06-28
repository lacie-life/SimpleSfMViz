#ifndef PCL_VISUAL_H
#define PCL_VISUAL_H

#include "pcl-1.12/pcl/filters/voxel_grid.h"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include <QObject>
#include <sophus/se3.hpp>

enum PCL_VISUAL_MODE {
    INTERACTIVE_MODE,
    RENDER_MODE
};

class PCLVisual : public QObject {
    Q_OBJECT
  public:
    explicit PCLVisual(QObject *parent = nullptr);

    void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

    double curTime();

  public slots:
    void renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

  signals:
    void signalShowPtsFinished();

  public:
    // pcl visualize

    pcl::visualization::PCLVisualizer::Ptr viewer;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _allPts;

    std::size_t _count;

  public:
    PCL_VISUAL_MODE _mode = PCL_VISUAL_MODE::RENDER_MODE;
};

#endif // PCL_VISUAL_H
