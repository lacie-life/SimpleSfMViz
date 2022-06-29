#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#include "configdialog.h"
#include "yolo.h"
#include "yolo_v2_class.hpp"
#include <QGridLayout>
#include <QLabel>
#include <QWidget>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace Ui
{
    class Recognizer;
}

class Recognizer : public QObject
{
    Q_OBJECT

public:
    explicit Recognizer(QObject *parent = nullptr);
    ~Recognizer();

public slots:
    void processNewColorFrame(cv::Mat srcImg);

    void init(ConfigDialog *cof);

signals:
    void signalProcessNewFrameFinished(cv::Mat img);

private:
    Ui::Recognizer *ui;

    std::vector<std::string> _classesVec;
    std::shared_ptr<Detector> _detector;

public:
    const std::string _cvWinName = "Recognizer";
};

#endif // RECOGNIZER_H
