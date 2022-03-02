#ifndef QSFM_H
#define QSFM_H

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <map>
#include <fstream>
#include <cassert>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <QObject>
#include <QString>

#include "QImageLoader.h"

using namespace std;
using namespace gtsam;
using namespace  cv;

const int IMAGE_DOWNSAMPLE = 1; // downsample the image to speed up processing
const double FOCAL_LENGTH = 4308 / IMAGE_DOWNSAMPLE; // focal length in pixels, after downsampling, guess from jpeg EXIF data
// prior_focal=max(width,heigh)*focal/ccdw
const int MIN_LANDMARK_SEEN = 3; // minimum number of camera views a 3d point (landmark) has to be seen to be used

struct ImagePose;
struct Landmark;

class QSfM : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString imgFolder READ imgFolder WRITE setImgFolder NOTIFY imgFolderChanged)
    Q_PROPERTY(QString pointCloudPath READ pointCloudPath WRITE setPointCloudPath NOTIFY pointCloudPathChanged)

public:
    QSfM(QObject *parent = nullptr);
    ~QSfM();

    void init(QString imgFolder);
    void run();

    QString imgFolder() const;
    QString pointCloudPath() const;

public slots:
    void setImgFolder(QString path);
    void setPointCloudPath(QString path);

signals:
    void imgFolderChanged(QString path);
    void pointCloudPathChanged(QString path);

public:
    QString m_imgFolder;
    QString m_pointCloudPath;

    QImageLoader m_loader;

    std::vector<std::string> m_image_names;

    std::vector<ImagePose> m_img_pose;
    std::vector<Landmark> m_landmark;
};

#endif // QSFM_H
