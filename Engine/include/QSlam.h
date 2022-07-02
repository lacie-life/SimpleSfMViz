#ifndef QSLAM_H
#define QSLAM_H

// attention here
#undef HAVE_CUDA
#include "System.h"
#include "QConfigDialog.h"

#include <QDebug>
#include <QObject>
#include <QThread>
#include <memory>
#include <opencv2/core/core.hpp>

class QSlam : public QObject
{
    Q_OBJECT
public:
    explicit QSlam(QObject *parent = nullptr);

    void shutdown();

    std::shared_ptr<ORB_SLAM3::System> getSlamSystem();

signals:
    void createSlamSystemFinished(float scale);

    void processNewFrameFinished(Sophus::SE3f pose);

public slots:
    void createSlamSystem(QConfigDialog *config);

    void processNewFrame(cv::Mat colorImg, cv::Mat depthImg, double tframe);

private:
    QConfigDialog *m_config;

    std::shared_ptr<ORB_SLAM3::System> m_slamSystem;

};

#endif // QSLAM_H
