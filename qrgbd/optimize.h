#pragma once

#include "configdialog.h"
#include "pcl-1.12/pcl/filters/voxel_grid.h"
#include "pcl-1.12/pcl/io/pcd_io.h"
#include "pcl-1.12/pcl/kdtree/kdtree_flann.h"
#include "pcl-1.12/pcl/point_cloud.h"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/surface/gp3.h"
#include "pcl-1.12/pcl/surface/impl/mls.hpp"
#include "pcl-1.12/pcl/surface/mls.h"
#include "pcl-1.12/pcl/surface/surfel_smoothing.h"
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

class Optimize {
  public:
    static SurfelCloudPtr reconstructionSurface(const PointCloudPtr &input,
                                                float radius = 0.05,
                                                int polynomial_order = 2);
    static pcl::PolygonMeshPtr triangulateMesh(const SurfelCloudPtr &surfels);
    static pcl::TextureMeshPtr MashTextured(const pcl::PolygonMeshPtr &mesh);

  public:
    static bool ICPTrack(const PointCloudPtr &source, const PointCloudPtr &target, Sophus::SE3f &Tts);
};
