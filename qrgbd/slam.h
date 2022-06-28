#ifndef SLAMTHREAD_H
#define SLAMTHREAD_H

// attention here
#undef HAVE_CUDA
#include "System.h"
#include "configdialog.h"
#include <QDebug>
#include <QObject>
#include <QThread>
#include <memory>
#include <opencv2/core/core.hpp>

class Slam : public QObject
{
    Q_OBJECT
public:
    explicit Slam(QObject *parent = nullptr);

    void shutdown();

    std::shared_ptr<ORB_SLAM3::System> getSlamSystem();

signals:
    void createSlamSystemFinished(float scale);

    void processNewFrameFinished(Sophus::SE3f pose);

public slots:
    void createSlamSystem(ConfigDialog *config);

    void processNewFrame(cv::Mat colorImg, cv::Mat depthImg, double tframe);

private:
    ConfigDialog *_config;

    std::shared_ptr<ORB_SLAM3::System> _slamSystem;
};

#endif // SLAMTHREAD_H
