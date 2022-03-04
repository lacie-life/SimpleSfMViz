#include "SfM/QSfM.h"

#include <QDir>
#include <QFile>
#include <QDebug>
#include <QString>
#include <QStringList>
#include <QVector>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>

QSfM::QSfM(QObject *parent)
    : QObject(parent)
    , m_imgFolder("")
    , m_pointCloudPath("")
{
    CONSOLE << "QSfM init ... ";
    m_image_names.reserve(1);


    CONSOLE << "Fucking crash";
}

QSfM::~QSfM()
{

}

void QSfM::init(QString imgFolder)
{
    CONSOLE << imgFolder;

    setImgFolder(imgFolder);

    QDir directory(imgFolder);

    QStringList images = directory.entryList(QStringList() << "*.jpg" << "*.JPG",QDir::Files);

    CONSOLE << images.size();

    for (int i = 0; i < images.size(); i++){
        QString fileName = images.at(i);

        QString path = directory.path() + "/" + fileName;

        CONSOLE << path;
        try
        {
           m_image_names.append(path);
        }  catch (...)
        {
            CONSOLE << "Fucking ...";
        }
    }

    CONSOLE << "READ DONE";

    CONSOLE << "Number of images: " << m_image_names.size();
}

void QSfM::run()
{
    featureExtract();
    triangulate();
    bundleAdjustment();
    reconstruction();
    // TODO: Point Cloud gennerate
}

QString QSfM::imgFolder() const
{
    return m_imgFolder;
}

QString QSfM::pointCloudPath() const
{
    return m_pointCloudPath;
}

void QSfM::setImgFolder(QString path)
{
    m_imgFolder = path;

    emit imgFolderChanged(path);
}

void QSfM::setPointCloudPath(QString path)
{
    m_pointCloudPath = path;

    emit pointCloudPathChanged(path);
}

void QSfM::featureExtract()
{
    {
        using namespace cv;

        Ptr<AKAZE> feature = AKAZE::create();
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

        // Extract features
        for (auto f : m_image_names) {
            ImagePose a;

            Mat img = imread(f.toStdString());
            std::vector<cv::KeyPoint> kp;
            assert(!img.empty());

            resize(img, img, img.size()/IMAGE_DOWNSAMPLE);
            a.img = img;
            cvtColor(img, img, COLOR_BGR2GRAY);

            feature->detect(img, kp);
            feature->compute(img, kp, a.desc);

            a.kp = QVector<cv::KeyPoint>(kp.begin(), kp.end());

            m_img_pose.append(a);
        }

        // Match features between all images
        for (int i=0; i < m_img_pose.size()-1; i++) {
            auto &img_pose_i = m_img_pose[i];
            for (int j=i+1; j < m_img_pose.size(); j++) {
                auto &img_pose_j = m_img_pose[j];
                vector<vector<DMatch>> matches;
                vector<Point2f> src, dst;
                vector<uchar> mask;
                vector<int> i_kp, j_kp;

                // 2 nearest neighbour match
                matcher->knnMatch(img_pose_i.desc, img_pose_j.desc, matches, 2);

                for (auto &m : matches) {
                    if(m[0].distance < 0.7*m[1].distance) {
                        src.push_back(img_pose_i.kp[m[0].queryIdx].pt);
                        dst.push_back(img_pose_j.kp[m[0].trainIdx].pt);

                        i_kp.push_back(m[0].queryIdx);
                        j_kp.push_back(m[0].trainIdx);
                    }
                }

                // Filter bad matches using fundamental matrix constraint
                findFundamentalMat(src, dst, FM_RANSAC, 3.0, 0.99, mask);

                cv::Mat canvas = img_pose_i.img.clone();
                canvas.push_back(img_pose_j.img.clone());

                for (size_t k=0; k < mask.size(); k++) {
                    if (mask[k]) {
                        img_pose_i.kp_match_idx(i_kp[k], j) = j_kp[k]; // store index value
                        img_pose_j.kp_match_idx(j_kp[k], i) = i_kp[k];

                        line(canvas, src[k], dst[k] + Point2f(0, img_pose_i.img.rows), Scalar(0, 0, 255), 2);
                    }
                }

                int good_matches = sum(mask)[0];
                assert(good_matches >= 10);

                cout << "Feature matching " << i << " " << j << " ==> " << good_matches << "/" << matches.size() << endl;

                //            resize(canvas, canvas, canvas.size()/2);
                waitKey(30);
            }
        }
    }
}

void QSfM::triangulate()
{
    {
        using namespace cv;
        // Setup camera matrix
        // It is assumed here that the center of the image is the main point coordinate.
        // If you have an internal reference file, you can replace it, and the image is a de-distorted image.
        double cx = m_img_pose[0].img.size().width/2;
        double cy = m_img_pose[0].img.size().height/2;

        cv::Point2d pp(cx, cy);

        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);

        K.at<double>(0,0) = FOCAL_LENGTH;
        K.at<double>(1,1) = FOCAL_LENGTH;
        K.at<double>(0,2) = cx;
        K.at<double>(1,2) = cy;

        cout << endl << "initial camera matrix K " << endl << K << endl << endl;

        m_img_pose[0].T = cv::Mat::eye(4, 4, CV_64F);
        m_img_pose[0].P = K*cv::Mat::eye(3, 4, CV_64F);

        CONSOLE << m_img_pose.size();

        for (int i=0; i < m_img_pose.size() - 1; i++) {

            CONSOLE << i;

            auto &prev = m_img_pose[i];
            auto &cur = m_img_pose[i+1];

            std::vector<cv::Point2f> src, dst;
            std::vector<size_t> kp_used;

            CONSOLE << "Prev Keypoint size: " << prev.kp.size();

            for (int k=0; k < prev.kp.size(); k++) {
                if (prev.kp_match_exist(k, i+1)) {
                    size_t match_idx = prev.kp_match_idx(k, i+1);

                    src.push_back(prev.kp[k].pt);
                    dst.push_back(cur.kp[match_idx].pt);

                    kp_used.push_back(k);
                }
            }

            cv::Mat mask;

            // NOTE: pose from dst to src,src as the reference frame
            // The first frame is used as the world coordinate system
            cv::Mat E = findEssentialMat(dst, src, FOCAL_LENGTH, pp, RANSAC, 0.999, 1.0, mask);
            cv::Mat local_R, local_t;

            recoverPose(E, dst, src, local_R, local_t, FOCAL_LENGTH, pp, mask);

            // local tansform
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
            local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

            // accumulate transform
            cur.T = prev.T*T; // T matrix is world to camera

            // make projection matrix
            cv::Mat R = cur.T(cv::Range(0, 3), cv::Range(0, 3));
            cv::Mat t = cur.T(cv::Range(0, 3), cv::Range(3, 4));

            cv::Mat P(3, 4, CV_64F); // P matrix is camera to world

            P(cv::Range(0, 3), cv::Range(0, 3)) = R.t();
            P(cv::Range(0, 3), cv::Range(3, 4)) = -R.t()*t;
            P = K*P;

            cur.P = P;

            cv::Mat points4D;
            triangulatePoints(prev.P, cur.P, src, dst, points4D); // triangulate camera to world P =R*p+C

            // Scale the new 3d points to be similar to the existing 3d points (landmark)
            // Use ratio of distance between pairing 3d points
            // Initialize the scale using the distance ratio of a pair of common 3d points
            if (i > 1) {

                double scale = 0;
                int count = 0;

                cv::Point3f prev_camera;

                prev_camera.x = prev.T.at<double>(0, 3);
                prev_camera.y = prev.T.at<double>(1, 3);
                prev_camera.z = prev.T.at<double>(2, 3);

                std::vector<cv::Point3f> new_pts;
                std::vector<cv::Point3f> existing_pts;
                CONSOLE << mask.size().height << " " << mask.size().width ;
                for (size_t j=0; j < kp_used.size(); j++) {
                    size_t k = kp_used[j];
                    CONSOLE << mask.at<uchar>(j) << " " << prev.kp_match_exist(k, i+1) << " " << prev.kp_3d_exist(k);
                    if (mask.at<uchar>(j) && prev.kp_match_exist(k, i+1) && prev.kp_3d_exist(k)) {
                        cv::Point3f pt3d;

                        pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                        pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                        pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

                        size_t idx = prev.kp_3d(k);
                        cv::Point3f avg_landmark = m_landmark[idx].pt / (m_landmark[idx].seen - 1);

                        new_pts.push_back(pt3d);
                        existing_pts.push_back(avg_landmark);

                        CONSOLE << "Fucking index: " << new_pts.size();
                    }
                }


                // ratio of distance for all possible point pairing
                // probably an over kill! can probably just pick N random pairs

                for (size_t j=0; j < new_pts.size()-1; j++) {
                    for (size_t k=j+1; k< new_pts.size(); k++) {
                        double s = norm(existing_pts[j] - existing_pts[k]) / norm(new_pts[j] - new_pts[k]);

                        scale += s;
                        count++;
                    }
                }

                assert(count > 0);

                scale /= count;

                CONSOLE << "image " << (i+1) << " ==> " << i << " scale=" << scale << " count=" << count;

                // apply scale and re-calculate T and P matrix
                local_t *= scale;

                // local tansform
                cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
                local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
                local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

                // accumulate transform
                cur.T = prev.T*T;

                // make projection ,matrix
                R = cur.T(cv::Range(0, 3), cv::Range(0, 3));
                t = cur.T(cv::Range(0, 3), cv::Range(3, 4));

                cv::Mat P(3, 4, CV_64F);
                P(cv::Range(0, 3), cv::Range(0, 3)) = R.t();
                P(cv::Range(0, 3), cv::Range(3, 4)) = -R.t()*t;
                P = K*P;

                cur.P = P;
                // re-update 3d

                triangulatePoints(prev.P, cur.P, src, dst, points4D);
            }

            // Find good triangulated points
            for (size_t j=0; j < kp_used.size(); j++) {
                if (mask.at<uchar>(j)) {
                    size_t k = kp_used[j];
                    size_t match_idx = prev.kp_match_idx(k, i+1);

                    cv::Point3f pt3d;

                    pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                    pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                    pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

                    if (prev.kp_3d_exist(k)) {
                        // Found a match with an existing landmark
                        cur.kp_3d(match_idx) = prev.kp_3d(k);

                        m_landmark[prev.kp_3d(k)].pt += pt3d;
                        m_landmark[cur.kp_3d(match_idx)].seen++;
                    } else {
                        // Add new 3d point
                        Landmark landmark;

                        landmark.pt = pt3d;
                        landmark.seen = 2;

                        m_landmark.push_back(landmark);

                        prev.kp_3d(k) = m_landmark.size() - 1;
                        cur.kp_3d(match_idx) = m_landmark.size() - 1;
                    }
                }
            }
        }

        // Average out the landmark 3d position
        for (auto &l : m_landmark) {
            if (l.seen >= 3) {
                l.pt /= (l.seen - 1);
            }
        }
    }
}

void QSfM::bundleAdjustment()
{
//    gtsam::Values result;
//    {
//        double cx = m_img_pose[0].img.size().width/2;
//        double cy = m_img_pose[0].img.size().height/2;

//        gtsam::Cal3_S2 K(FOCAL_LENGTH, FOCAL_LENGTH, 0 /* skew */, cx, cy);
//        gtsam::noiseModel::Isotropic::shared_ptr measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 2.0); // pixel error in (x,y)

//        gtsam::NonlinearFactorGraph graph;
//        gtsam::Values initial;

//        // Poses
//        for (int i=0; i < m_img_pose.size(); i++) {
//            auto &img_pose = m_img_pose[i];

//            gtsam::Rot3 R(
//                        img_pose.T.at<double>(0,0),
//                        img_pose.T.at<double>(0,1),
//                        img_pose.T.at<double>(0,2),

//                        img_pose.T.at<double>(1,0),
//                        img_pose.T.at<double>(1,1),
//                        img_pose.T.at<double>(1,2),

//                        img_pose.T.at<double>(2,0),
//                        img_pose.T.at<double>(2,1),
//                        img_pose.T.at<double>(2,2)
//                        );

//            gtsam::Point3 t;

//            t(0) = img_pose.T.at<double>(0,3);
//            t(1) = img_pose.T.at<double>(1,3);
//            t(2) = img_pose.T.at<double>(2,3);

//            gtsam::Pose3 pose(R, t);

//            // Add prior for the first image
//            if (i == 0) {
//                gtsam::noiseModel::Diagonal::shared_ptr pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.1)).finished());
//                graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', 0), pose, pose_noise); // add directly to graph
//            }

//            initial.insert(gtsam::Symbol('x', i), pose);

//            // landmark seen
//            for (int k=0; k < img_pose.kp.size(); k++) {
//                if (img_pose.kp_3d_exist(k)) {
//                    size_t landmark_id = img_pose.kp_3d(k);

//                    if (m_landmark[landmark_id].seen >= MIN_LANDMARK_SEEN) {
//                        gtsam::Point2 pt;

//                        pt(0) = img_pose.kp[k].pt.x;
//                        pt(1) = img_pose.kp[k].pt.y;

//                        graph.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>(pt, measurement_noise, gtsam::Symbol('x', i), gtsam::Symbol('l', landmark_id), gtsam::Symbol('K', 0));
//                    }
//                }
//            }
//        }

//        // Add a prior on the calibration.
//        initial.insert(gtsam::Symbol('K', 0), K);

//        gtsam::noiseModel::Diagonal::shared_ptr cal_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 100, 100, 0.01 /*skew*/, 100, 100).finished());
//        graph.emplace_shared<gtsam::PriorFactor<gtsam::Cal3_S2>>(gtsam::Symbol('K', 0), K, cal_noise);

//        // Initialize estimate for landmarks
//        bool init_prior = false;

//        for (int i=0; i < m_landmark.size(); i++) {
//            if (m_landmark[i].seen >= MIN_LANDMARK_SEEN) {
//                cv::Point3f &p = m_landmark[i].pt;

//                initial.insert<gtsam::Point3>(gtsam::Symbol('l', i), gtsam::Point3(p.x, p.y, p.z));

//                if (!init_prior) {
//                    init_prior = true;

//                    gtsam::noiseModel::Isotropic::shared_ptr point_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
//                    gtsam::Point3 p(m_landmark[i].pt.x, m_landmark[i].pt.y, m_landmark[i].pt.z);
//                    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(gtsam::Symbol('l', i), p, point_noise);
//                }
//            }
//        }

//        result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

//        cout << endl;
//        cout << "initial graph error = " << graph.error(initial) << endl;
//        cout << "final graph error = " << graph.error(result) << endl;
//    }
}

void QSfM::reconstruction()
{
    CONSOLE << "Run reconstruction";

    savePCD();
}

void QSfM::savePCD()
{
    CONSOLE << "Save point cloud in pcd format";
}
