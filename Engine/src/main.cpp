#include "mainwindow.h"

#include <QApplication>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<Sophus::SE3f>("Sophus::SE3f");
    MainWindow w;
    w.show();
    return a.exec();
}
