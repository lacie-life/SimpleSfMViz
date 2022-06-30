#ifndef QOPTIMIZE_H
#define QOPTIMIZE_H

#include "QConfigDialog.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/surface/gp3.h"
#include "pcl/surface/impl/mls.hpp"
#include "pcl/surface/mls.h"
#include "pcl/surface/surfel_smoothing.h"
#include "pcl/TextureMesh.h"
#include <QGridLayout>
#include <QLabel>
#include <QObject>
#include <QWidget>
#include <opencv2/core.hpp>
#include <pcl/registration/icp.h>
#include <sophus/se3.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointXYZRGBNormal SurfelT;
typedef pcl::PointCloud<SurfelT> SurfelCloud;
typedef pcl::PointCloud<SurfelT>::Ptr SurfelCloudPtr;

class QOptimize
{
public:
    static SurfelCloudPtr reconstructionSurface(const PointCloudPtr &input,
                                                float radius = 0.05,
                                                int polynomial_order = 2);
    static pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels);
    static pcl::TextureMeshPtr MashTextured(const pcl::PolygonMeshPtr &mesh);

public:
    static bool ICPTrack(const PointCloudPtr &source, const PointCloudPtr &target, Sophus::SE3f &Tts);
};

#endif // QOPTIMIZE_H
