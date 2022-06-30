#ifndef QREBUILDER_H
#define QREBUILDER_H

#include "QConfigDialog.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "QPCLVisual.h"
#include <QGridLayout>
#include <QLabel>
#include <QObject>
#include <QThread>
#include <QWidget>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace Ui {
class QRebuilder;
}

class QRebuilder : public QObject
{
    Q_OBJECT
public:
    explicit QRebuilder(QObject *parent = nullptr);
    ~QRebuilder();

    void changeToInterMode();

    double curTime();

    void changeToRenderMode();

public slots:
    void processNewDepthFrame(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f Tcw);

    void init(QConfigDialog *config);

signals:

    void signalProcessNewFrameFinished(cv::Mat im);

    void signalPCLShowPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frameCloud, Sophus::SE3f Tcw);

public:
    Ui::QRebuilder *ui;

    float fx, fy, cx, cy, depthScale;

    std::shared_ptr<PCLVisual> _pcl_visual;

    QThread _thread;

    Sophus::SE3f _curTcw;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pts;

    bool _pcl_visual_need_data = true;

public:
    const std::string _cvWinName = "Rebuilder";

};

#endif // QREBUILDER_H
