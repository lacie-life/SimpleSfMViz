#include "QRecognizer.h"
#include "QEmbeddedWindow.h"

#include <QThread>
#include <opencv2/highgui/highgui.hpp>

QRecognizer::QRecognizer(QObject *parent)
    : QObject{parent}
{

}

QRecognizer::~QRecognizer()
{
    delete ui;
}

void QRecognizer::processNewColorFrame(cv::Mat srcImg) {
    cv::Mat dstImg = srcImg.clone();

    // do process

    //yoloDetector(dstImg, this->_classesVec, *(this->_detector));

    // show image
    emit this->signalProcessNewFrameFinished(dstImg);
}

void QRecognizer::init(QConfigDialog *cof) {
//    qDebug() << "Recognizer thread: " << QThread::currentThread();

    std::ifstream ifs(cof->m_classes.toStdString().c_str());
    std::string line;
    while (std::getline(ifs, line))
        this->m_classesVec.push_back(line);
    this->m_detector = std::make_shared<Detector>(cof->m_modelConfig.toStdString(),
                                                 cof->m_modelWeights.toStdString(), 0);
}
