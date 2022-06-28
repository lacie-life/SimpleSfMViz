#include "pcl_visual.h"
#include "QThread"
#include "optimize.h"
#include "timer.h"
#include <QDebug>

PCLVisual::PCLVisual(QObject *parent)
    : QObject{parent} {

    // pcl win

    this->_allPts = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    this->viewer = std::make_shared<pcl::visualization::PCLVisualizer>();
    //
    viewer->removeAllPointClouds();
    //
    this->viewer->setSize(1000, 640);
    this->viewer->setBackgroundColor(0.9f, 0.9f, 0.9f);
    this->_count = 0;

    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
    origin.pretranslate(Eigen::Vector3f::Zero());
    this->viewer->addCoordinateSystem(0.5, Eigen::Affine3f(origin.affine()), "origin");

    viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    viewer->spinOnce();
}

void PCLVisual::showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw) {
    if (this->_mode == PCL_VISUAL_MODE::RENDER_MODE) {
        this->renderMode(frameCloud, Tcw);
    } else if (this->_mode == PCL_VISUAL_MODE::INTERACTIVE_MODE) {
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> filter;
            filter.setLeafSize(0.02, 0.02, 0.02);
            filter.setInputCloud(_allPts);
            filter.filter(*_allPts);
            viewer->removeAllPointClouds();
            viewer->addPointCloud(this->_allPts, "cloud");
        }
        {
            //            ns_timer::Timer timer;
            //            timer.reStart();
            //            SurfelCloudPtr _allPtsSur = Optimize::reconstructionSurface(_allPts, 0.05);
            //            qDebug() << QString::fromStdString(timer.last_elapsed<ns_timer::DurationType::MS>("reconstructionSurface"));
            //            timer.reStart();
            //            pcl::PolygonMeshPtr _allPtsPoMesh = Optimize::triangulateMesh(_allPtsSur);
            //            qDebug() << QString::fromStdString(timer.last_elapsed("triangulateMesh"));
            //            this->viewer->addPolylineFromPolygonMesh(*_allPtsPoMesh, "PolylineFromPolygonMesh");
            //            this->viewer->addPolygonMesh(*_allPtsPoMesh, "PolygonMesh");
        }
        //
        while (!this->viewer->wasStopped() && this->_mode == PCL_VISUAL_MODE::INTERACTIVE_MODE) {
            this->viewer->spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        {
            //            this->viewer->removePolygonMesh("PolygonMesh");
        }
        emit this->signalShowPtsFinished();
    }
}

double PCLVisual::curTime() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::time_point_cast<std::chrono::duration<double>>(now)
        .time_since_epoch()
        .count();
}

void PCLVisual::renderMode(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw) {
    ++this->_count;
    if (Tcw.log() == Sophus::SE3f().log()) {
        // remove pts
        this->_allPts->clear();
        this->_allPts->reserve(0);
        // camera
        viewer->setCameraPosition(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        viewer->spinOnce();
        emit this->signalShowPtsFinished();
        return;
    }
    auto Twc = Tcw.inverse();
    auto quat = Twc.unit_quaternion();
    auto trans = Twc.translation();
    // add new point cloud
    this->_allPts->resize(this->_allPts->size() + frameCloud->size());
    std::copy(frameCloud->cbegin(), frameCloud->cend(), this->_allPts->end() - frameCloud->size());

    if (this->_count % 20 == 0) {
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setLeafSize(0.02, 0.02, 0.02);
        filter.setInputCloud(_allPts);
        filter.filter(*_allPts);
        viewer->removeAllPointClouds();
        viewer->addPointCloud(this->_allPts, "cloud");
    } else {
        viewer->addPointCloud(frameCloud, "cloud_" + std::to_string(this->_count));
    }
    // set camera pos
    Eigen::Vector3f pos(0.0f, 0.0f, -3.0f);
    Eigen::Vector3f view(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f up(0.0f, -1.0f, -3.0f);
    pos = Twc * pos;
    view = Twc * view;
    up = Twc * up;
    up(0) = up(0) - pos(0), up(1) = up(1) - pos(1), up(2) = up(2) - pos(2);
    viewer->setCameraPosition(pos(0), pos(1), pos(2),
                              view(0), view(1), view(2),
                              up(0), up(1), up(2));

    Eigen::Isometry3f coord(quat);
    coord.pretranslate(trans);
    viewer->removeCoordinateSystem("Camera");
    viewer->addCoordinateSystem(0.2, Eigen::Affine3f(coord.affine()), "Camera");
    viewer->spinOnce();
    emit this->signalShowPtsFinished();
}
