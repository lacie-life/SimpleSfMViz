#include "Camera/QCameraCapture.h"
#include "AppConstant.h"
#include <QDebug>
#include <QTime>
#include <QtConcurrent/QtConcurrent>

QCameraCapture::QCameraCapture(QObject *parent)
{
    running = false;
    fps_calculating = false;
    fps = 0.0;

    frame_width = frame_height = 0;
}

QCameraCapture::QCameraCapture(int camera, QMutex *lock)
    : running(false), cameraID(camera), data_lock(lock)
{
    fps_calculating = false;
    fps = 0.0;

    frame_width = frame_height = 0;
}

QCameraCapture::QCameraCapture(QString videoPath, QMutex *lock)
    : running(false), cameraID(-1), videoPath(videoPath), data_lock(lock)
{
    fps_calculating = false;
    fps = 0.0;

    frame_width = frame_height = 0;
}

QCameraCapture::~QCameraCapture()
{

}

void QCameraCapture::initCamera(QString _videoPath, QMutex *lock)
{
    CONSOLE << _videoPath;
    cameraID = -1;
    videoPath = _videoPath;
    data_lock = lock;
}

void QCameraCapture::setRunning(bool run) {
    running = run;
}

void QCameraCapture::startCalcFPS() {
    fps_calculating = true;
}

void QCameraCapture::calculateFPS(cv::VideoCapture &cap)
{
    const int count_to_read = 100;
    cv::Mat tmp_frame;
    QTime timer;
    timer.start();
    for(int i = 0; i < count_to_read; i++) {
            cap >> tmp_frame;
    }
    int elapsed_ms = timer.elapsed();
    fps = count_to_read / (elapsed_ms / 1000.0);
    fps_calculating = false;
    emit fpsChanged(fps);
}

void QCameraCapture::run()
{
    running = true;

    cv::VideoCapture cap;

    if (cameraID == -1){
        CONSOLE << "Streamming mode";
        // cap = cv::VideoCapture(videoPath.toStdString(), cv::CAP_GSTREAMER);
        CONSOLE << videoPath;
        cap = cv::VideoCapture(this->videoPath.toStdString());
    }
    else{
        CONSOLE << "USB/Webcam mode";
        cap = cv::VideoCapture(cameraID);
    }

    cv::Mat tmp_frame;

    frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    while (running) {
        cap >> tmp_frame;

        if(tmp_frame.empty())
        {
            break;
        }

        cv::cvtColor(tmp_frame, tmp_frame, cv::COLOR_BGR2RGB);

        data_lock->lock();
        frame = tmp_frame;
        data_lock->unlock();
        // cv::imshow("Video", frame);
        emit frameCaptured(&frame);
        if(fps_calculating) {
            calculateFPS(cap);
        }

    }

    cap.release();
    running = false;

}
