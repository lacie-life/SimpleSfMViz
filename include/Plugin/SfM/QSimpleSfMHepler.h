#ifndef QSIMPLESFMHEPLER_H
#define QSIMPLESFMHEPLER_H

#include <QObject>

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Utils/Timer.h"
#include "Utils/Database.h"
#include "Feature/FeatureExtraction.h"

class QSimpleSfMHepler
{
public:
    QSimpleSfMHepler(QString path);

    void init();
    void run();

private:

    QString m_configPath;

    // Config parameters
    QString m_imagesPath;
    QString m_databasePath;

    int m_max_image_size = 3200;
    int m_num_features = 8024;
    int m_normalization_type = 0;
    int m_match_type = 1;
    double m_max_distance = 0.7;
    double m_distance_ratio = 0.8;
    bool m_cross_check = true;
    SimpleSfM::MapBuilder::Parameters params;

    // Worker
    SimpleSfM::Timer m_timer;
    SimpleSfM::Database m_database;
    cv::Ptr<SimpleSfM::FeatureExtractor> m_extractor;
    cv::Ptr<SimpleSfM::FeatureMatcher> m_matcher;
    SimpleSfM::MapBuilder m_mapBuilder;
};

#endif // QSIMPLESFMHEPLER_H
