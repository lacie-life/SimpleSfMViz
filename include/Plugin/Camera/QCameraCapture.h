#ifndef QCAMERACAPTURE_H
#define QCAMERACAPTURE_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QImage>
#include <QTimer>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

// TODO: Read video/camera

using namespace std;

static cv::VideoCapture cap;

class QCameraCapture : public QObject
{
    Q_OBJECT
public:
    QCameraCapture(QObject *parent = nullptr);
    ~QCameraCapture();

public:
    void stream();
    QThread* threadStreamer = new QThread();
    void catchFrame(cv::Mat emittedFrame);

public slots:
    void initCamera(QString videoPath);
    void streamerThreadSlot();

signals:
    void newImage(QImage &);
    void emitThreadImage(cv::Mat frameThread);

private:
    bool running;
    int cameraID;
    QString videoPath;
    QMutex *data_lock;
    cv::Mat frame;

    QTimer tUpdate;

    // FPS calculating
    bool fps_calculating;
    float fps;

    // video
    int frame_width, frame_height;
};

#endif // QCAMERACAPTURE_H
