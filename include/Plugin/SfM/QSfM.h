#ifndef QSFM_H
#define QSFM_H

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <vector>
#include <iostream>
#include <map>
#include <fstream>
#include <cassert>

#include <QObject>
#include <QString>
#include <QVector>
#include <QStringList>
#include <QMap>

#include "AppConstant.h"

using namespace std;
using namespace  cv;

// downsample the image to speed up processing
const int IMAGE_DOWNSAMPLE = 1;

// focal length in pixels, after downsampling, guess from jpeg EXIF data
const double FOCAL_LENGTH = 825 / IMAGE_DOWNSAMPLE;

// prior_focal=max(width,heigh)*focal/ccdw

// minimum number of camera views a 3d point (landmark) has to be seen to be used
const int MIN_LANDMARK_SEEN = 3;

struct SFM_Helper
{
    struct ImagePose
    {
        cv::Mat img; // down sampled image used for display
        cv::Mat desc; // feature descriptor
        QVector<cv::KeyPoint> kp; // keypoint

        cv::Mat T; // 4x4 pose transformation matrix
        cv::Mat P; // 3x4 projection matrix

        // alias to clarify map usage below
        using kp_idx_t = unsigned int;
        using landmark_idx_t = unsigned int;
        using img_idx_t = unsigned int;

        QMap<kp_idx_t, QMap<img_idx_t, kp_idx_t>> kp_matches; // keypoint matches in other images
        QMap<kp_idx_t, landmark_idx_t> kp_landmark; // keypoint to 3d points

        // helper
        kp_idx_t& kp_match_idx(int kp_idx, int img_idx)
        {
            return kp_matches[kp_idx][img_idx];
        }

        bool kp_match_exist(int kp_idx, int img_idx)
        {
            return kp_matches[kp_idx].count(img_idx) > 0;
        }

        landmark_idx_t& kp_3d(int kp_idx)
        {
            return kp_landmark[kp_idx];
        }
        bool kp_3d_exist(int kp_idx)
        {
            return kp_landmark.count(kp_idx) > 0;
        }
    };

    // 3D point
    struct Landmark
    {
        cv::Point3f pt;
        int seen = 0; // how many cameras have seen this point
    };
    QVector<ImagePose> m_img_pose;
    QVector<Landmark> m_landmark;
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

private:
    QString m_imgFolder;
    QString m_pointCloudPath;

    SFM_Helper SFM;

    pcl::PointCloud<pcl::PointXYZ> m_cloud;

    QStringList m_image_names;
};

#endif // QSFM_H
