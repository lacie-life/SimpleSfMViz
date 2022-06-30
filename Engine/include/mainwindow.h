#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <memory>

#include "QConfigDialog.h"
#include "QHelpDialog.h"
#include "QRebuilder.h"
#include "QRecognizer.h"
#include "QSlam.h"
#include "QTimerHelper.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void connection();

    void init();

    void closeEvent(QCloseEvent *e) override;

    void loadImages(const std::string &strAssociationFilename);

    void processNewFrame();

    void quitThreads();

    void displayMapInfo();

    void displayMapPoints();

    void displayKeyFrames();

    void createCVWins();

signals:
    void signalCreateSlamSystem(QConfigDialog *config);

    void signalNewFrameToSlamSystem(cv::Mat colorImg, cv::Mat depthImg, double tframe);

    void signalNewColorFrameToRecongnizer(cv::Mat srcImg);

    void signalNewDepthFrameToReBulider(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose);

    void signalInitRebuilder(QConfigDialog *cof);

    void signalInitRecoginzer(QConfigDialog *cof);

private slots:
    void on_actionrun_triggered();

    void on_actionstop_triggered();

    void on_actioncontinue_triggered();

    void on_actionforce_quit_triggered();

    void on_actionhelp_triggered();

    void on_actionback_triggered();

    void on_actionsave_triggered();

    void on_actionquit_triggered();

    void on_actionstart_triggered();

private:
    Ui::MainWindow *ui;

    // the help dialog
    QHelpDialog _helpDig;
    // the configure dialog
    QConfigDialog _configDig;

    // the slam object
    QSlam *_slam;
    // thread to run orb-slam
    QThread _slamThread;

    // the  recongnizer
    QRecognizer *_recognizer;
    // thread to recongnize objects
    QThread _recogThread;

    // rebuilder
    QRebuilder *_rebuilder;
    // thread to run rebulider
    QThread _rebulidThread;

    // timer used to send single shot [image] to slam object
    QTimer _timer;

    // vectors [rgb, depth, timeStamp, timeTrack]
    std::vector<std::string> _vstrImageFilenamesRGB;
    std::vector<std::string> _vstrImageFilenamesD;
    std::vector<double> _vTimestamps;
    std::vector<float> _vTimesTrack;

    // number of images, the scale of image
    int _nImages;
    float _imgScale;

    // images [rgb, depth]
    cv::Mat _imRGB;
    cv::Mat _imD;

    // current frame index in the image sequence
    int _curFrameIdx = -1;

    // for timing
    ns_timer::Timer<> _timing;

    // time of the frame
    double _tframe;

    // the slam thread is running
    bool _isRunning = false;
};
#endif // MAINWINDOW_H
