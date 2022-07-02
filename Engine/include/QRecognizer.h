#ifndef QRECOGNIZER_H
#define QRECOGNIZER_H

#include "QConfigDialog.h"
#include "yolo.h"
#include "yolo_v2_class.hpp"

#include <QGridLayout>
#include <QLabel>
#include <QWidget>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <QObject>

namespace Ui
{
    class QRecognizer;
}

class QRecognizer : public QObject
{
    Q_OBJECT
public:
    explicit QRecognizer(QObject *parent = nullptr);

    ~QRecognizer();

    public slots:
        void processNewColorFrame(cv::Mat srcImg);

        void init(QConfigDialog *cof);

    signals:
        void signalProcessNewFrameFinished(cv::Mat img);

    private:
        Ui::QRecognizer *ui;

        std::vector<std::string> m_classesVec;
        std::shared_ptr<Detector> m_detector;

    public:
        const std::string m_cvWinName = "Recognizer";

};

#endif // QRECOGNIZER_H
