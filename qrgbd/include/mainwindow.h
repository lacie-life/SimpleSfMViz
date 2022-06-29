#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "configdialog.h"
#include "helpdialog.h"
#include "rebuilder.h"
#include "recognizer.h"
#include "slam.h"
#include "timer.h"
#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void connection();

    void init();

    void closeEvent(QCloseEvent *e) override;

    void loadImages(const string &strAssociationFilename);

    void processNewFrame();

    void quitThreads();

    void displayMapInfo();

    void displayMapPoints();

    void displayKeyFrames();

    void createCVWins();

  signals:
    void signalCreateSlamSystem(ConfigDialog *config);

    void signalNewFrameToSlamSystem(cv::Mat colorImg, cv::Mat depthImg, double tframe);

    void signalNewColorFrameToRecongnizer(cv::Mat srcImg);

    void signalNewDepthFrameToReBulider(cv::Mat colorImg, cv::Mat depthImg, Sophus::SE3f pose);

    void signalInitRebuilder(ConfigDialog *cof);

    void signalInitRecoginzer(ConfigDialog *cof);

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

  public:
    Ui::MainWindow *ui;
    // the help dialog
    HelpDialog _helpDig;
    // the configure dialog
    ConfigDialog _configDig;

    // the slam object
    Slam *_slam;
    // thread to run orb-slam
    QThread _slamThread;

    // the  recongnizer
    Recognizer *_recognizer;
    // thread to recongnize objects
    QThread _recogThread;

    // rebuilder
    Rebuilder *_rebuilder;
    // thread to run rebulider
    QThread _rebulidThread;

    // timer used to send single shot [image] to slam object
    QTimer _timer;

    // vectors [rgb, depth, timeStamp, timeTrack]
    vector<string> _vstrImageFilenamesRGB;
    vector<string> _vstrImageFilenamesD;
    vector<double> _vTimestamps;
    vector<float> _vTimesTrack;

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
