#include "recognizer.h"
#include "QDebug"
#include "QThread"
#include "embed.h"
#include "opencv2/highgui/highgui.hpp"

Recognizer::Recognizer(QObject *parent)
    : QObject(parent) {
}

Recognizer::~Recognizer() {
    delete ui;
}

void Recognizer::processNewColorFrame(cv::Mat srcImg) {
    cv::Mat dstImg = srcImg.clone();

    // do process

    //yoloDetector(dstImg, this->_classesVec, *(this->_detector));

    // show image
    emit this->signalProcessNewFrameFinished(dstImg);
}

void Recognizer::init(ConfigDialog *cof) {
    qDebug() << "Recognizer thread: " << QThread::currentThread();

    std::ifstream ifs(cof->_classes.toStdString().c_str());
    std::string line;
    while (std::getline(ifs, line))
        this->_classesVec.push_back(line);
    this->_detector = std::make_shared<Detector>(cof->_modelConfig.toStdString(),
                                                 cof->_modelWeights.toStdString(), 0);
}
