#ifndef QCAMERACAPTURE_H
#define QCAMERACAPTURE_H

#include <QObject>
#include <QThread>
#include <QMutex>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

// TODO: Read video/camera

using namespace std;

class QCameraCapture : public QThread
{
    Q_OBJECT
public:
    QCameraCapture(QObject *parent = nullptr);
    QCameraCapture(int camera, QMutex *lock);
    QCameraCapture(QString videoPath, QMutex *lock);
    ~QCameraCapture();
    void initCamera(QString videoPath, QMutex *lock);
    void setRunning(bool run);
    void startCalcFPS();

signals:
    void frameCaptured(cv::Mat *data);
    void fpsChanged(float fps);

private:
    void calculateFPS(cv::VideoCapture &cap);

protected:
    void run() override;

private:
    bool running;
    int cameraID;
    QString videoPath;
    QMutex *data_lock;
    cv::Mat frame;

    // FPS calculating
    bool fps_calculating;
    float fps;

    // video
    int frame_width, frame_height;
};

#endif // QCAMERACAPTURE_H
