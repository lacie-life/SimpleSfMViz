#include "QSlam.h"

QSlam::QSlam(QObject *parent)
{
}

void QSlam::shutdown()
{
    this->_slamSystem->Shutdown();
    //    this->_slamSystem->mpViewer->RequestFinish();
    return;
}

std::shared_ptr<ORB_SLAM3::System> QSlam::getSlamSystem()
{
    return this->_slamSystem;
}

void QSlam::createSlamSystem(QConfigDialog *config)
{
    qDebug() << "Slam thread: " << QThread::currentThread();

    this->_config = config;

    // create a slam system
    this->_slamSystem = std::make_shared<ORB_SLAM3::System>(
        this->_config->_vocPath.toStdString(),
        this->_config->_settingPath.toStdString(),
        ORB_SLAM3::System::RGBD,
        true);
    // finished
    emit this->createSlamSystemFinished(this->_slamSystem->GetImageScale());

    //  emit this->createSlamSystemFinished(1.0f);
}

void QSlam::processNewFrame(cv::Mat colorImg, cv::Mat depthImg, double tframe)
{
    // track a new frame

    Sophus::SE3f pose = this->_slamSystem->TrackRGBD(colorImg, depthImg, tframe);

    emit this->processNewFrameFinished(pose);
}
