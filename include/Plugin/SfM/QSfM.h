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
#include <QVector>
#include <QStringList>

#include "AppConstant.h"

using namespace std;
using namespace gtsam;
using namespace  cv;

const int IMAGE_DOWNSAMPLE = 1; // downsample the image to speed up processing
const double FOCAL_LENGTH = 4308 / IMAGE_DOWNSAMPLE; // focal length in pixels, after downsampling, guess from jpeg EXIF data
// prior_focal=max(width,heigh)*focal/ccdw
const int MIN_LANDMARK_SEEN = 3; // minimum number of camera views a 3d point (landmark) has to be seen to be used

struct ImagePose
{
    cv::Mat img; // down sampled image used for display
    cv::Mat desc; // feature descriptor
    std::vector<cv::KeyPoint> kp; // keypoint

    cv::Mat T; // 4x4 pose transformation matrix
    cv::Mat P; // 3x4 projection matrix

    // alias to clarify map usage below
    using kp_idx_t = size_t;
    using landmark_idx_t = size_t;
    using img_idx_t = size_t;

    std::map<kp_idx_t, std::map<img_idx_t, kp_idx_t>> kp_matches; // keypoint matches in other images
    std::map<kp_idx_t, landmark_idx_t> kp_landmark; // seypoint to 3d points

    // helper
    kp_idx_t& kp_match_idx(size_t kp_idx, size_t img_idx) { return kp_matches[kp_idx][img_idx]; };
    bool kp_match_exist(size_t kp_idx, size_t img_idx) { return kp_matches[kp_idx].count(img_idx) > 0; };

    landmark_idx_t& kp_3d(size_t kp_idx) { return kp_landmark[kp_idx]; }
    bool kp_3d_exist(size_t kp_idx) { return kp_landmark.count(kp_idx) > 0; }
};

// 3D point
struct Landmark
{
    cv::Point3f pt;
    int seen = 0; // how many cameras have seen this point
};

class QSfM : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString imgFolder READ imgFolder WRITE setImgFolder NOTIFY imgFolderChanged)
    Q_PROPERTY(QString pointCloudPath READ pointCloudPath WRITE setPointCloudPath NOTIFY pointCloudPathChanged)

public:
    explicit QSfM(QObject *parent = nullptr);
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

private:
    void featureExtract();
    void triangulate();
    void bundleAdjustment();
    void reconstruction();
    void savePCD();

public:
    QString m_imgFolder;
    QString m_pointCloudPath;

    QStringList m_image_names;

    QVector<ImagePose> m_img_pose;
    QVector<Landmark> m_landmark;
};

#endif // QSFM_H
