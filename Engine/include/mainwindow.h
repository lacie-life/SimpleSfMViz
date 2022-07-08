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

    void loadImages(const std::string &strAssociationDFilename, const std::string &strAssociationRGBFilename);

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
    QHelpDialog m_helpDig;
    // the configure dialog
    QConfigDialog m_configDig;

    // the slam object
    QSlam *m_slam;
    // thread to run orb-slam
    QThread m_slamThread;

    // the  recongnizer
    QRecognizer *m_recognizer;
    // thread to recongnize objects
    QThread m_recogThread;

    // rebuilder
    QRebuilder *m_rebuilder;
    // thread to run rebulider
    QThread m_rebulidThread;

    // timer used to send single shot [image] to slam object
    QTimer m_timer;

    // vectors [rgb, depth, timeStamp, timeTrack]
    std::vector<std::string> m_vstrImageFilenamesRGB;
    std::vector<std::string> m_vstrImageFilenamesD;
    std::vector<double> m_vTimestamps;
    std::vector<float> m_vTimesTrack;

    // number of images, the scale of image
    int m_nImages;
    float m_imgScale;

    // images [rgb, depth]
    cv::Mat m_imRGB;
    cv::Mat m_imD;

    // current frame index in the image sequence
    int m_curFrameIdx = -1;

    // for timing
    ns_timer::Timer<> m_timing;

    // time of the frame
    double m_tframe;

    // the slam thread is running
    bool m_isRunning = false;
};
#endif // MAINWINDOW_H
