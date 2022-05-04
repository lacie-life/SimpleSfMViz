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

    connect(&tUpdate, &QTimer::timeout, this, &QCameraCapture::stream);
}


QCameraCapture::~QCameraCapture()
{
    cap.release();
    tUpdate.stop();
    threadStreamer->requestInterruption();
}

void QCameraCapture::initCamera(QString _videoPath)
{
    CONSOLE << _videoPath;
    cameraID = -1;
    videoPath = _videoPath;


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

    QCameraCapture* worker = new QCameraCapture();

    worker->moveToThread(threadStreamer);
    QObject::connect(threadStreamer,&QThread::started,worker,&QCameraCapture::streamerThreadSlot);
    QObject::connect(worker,&QCameraCapture::emitThreadImage,this,&QCameraCapture::catchFrame);
    threadStreamer->start();

    double fps = cap.get(cv::CAP_PROP_FPS);

    CONSOLE << "FPS: " << fps;

    tUpdate.start(1000/fps);
}

void QCameraCapture::streamerThreadSlot()
{
    cv::Mat tempFrame;

    while(1)
    {
        cap >> tempFrame;

        if(tempFrame.data)
        {
            cv::cvtColor(tempFrame, tempFrame, cv::COLOR_BGR2RGB);

            emit emitThreadImage(tempFrame);
        }

        if(QThread::currentThread()->isInterruptionRequested())
        {
            cap.release();
            return;
        }
    }
}


void QCameraCapture::stream()
{
    if(frame.data)
    {
        QImage img = QImage(frame.data,frame.cols,frame.rows,QImage::Format_RGB888).rgbSwapped();
        emit newImage(img);
    }

}

void QCameraCapture::catchFrame(cv::Mat emittedFrame)
{
    frame = emittedFrame;
}
