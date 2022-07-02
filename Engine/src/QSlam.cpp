#include "QSlam.h"

QSlam::QSlam(QObject *parent)
{
}

void QSlam::shutdown()
{
    this->m_slamSystem->Shutdown();
    //    this->_slamSystem->mpViewer->RequestFinish();
    return;
}

std::shared_ptr<ORB_SLAM3::System> QSlam::getSlamSystem()
{
    return this->m_slamSystem;
}

void QSlam::createSlamSystem(QConfigDialog *config)
{
    qDebug() << "Slam thread: " << QThread::currentThread();

    this->m_config = config;

    // create a slam system
    this->m_slamSystem = std::make_shared<ORB_SLAM3::System>(
        this->m_config->m_vocPath.toStdString(),
        this->m_config->m_settingPath.toStdString(),
        ORB_SLAM3::System::RGBD,
        true);
    // finished
    emit this->createSlamSystemFinished(this->m_slamSystem->GetImageScale());

    //  emit this->createSlamSystemFinished(1.0f);
}

void QSlam::processNewFrame(cv::Mat colorImg, cv::Mat depthImg, double tframe)
{
    // track a new frame

    Sophus::SE3f pose = this->m_slamSystem->TrackRGBD(colorImg, depthImg, tframe);

    emit this->processNewFrameFinished(pose);
}
