#include "SfM/QSfM.h"

#include <QDir>
#include <QFile>
#include <QDebug>

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

QSfM::QSfM(QObject *parent)
    : QObject(parent)
{

}

QSfM::~QSfM()
{

}

void QSfM::init(QString imgFolder)
{
    CONSOLE << imgFolder;

    QDir directory(imgFolder);

    QStringList images = directory.entryList(QStringList() << "*.jpg" << "*.JPG",QDir::Files);

    foreach (QString filename, images)
    {
        CONSOLE << "Image name : " << filename;
        m_image_names.push_back(imgFolder + "/" + filename);

    }

    CONSOLE << "READ DONE";

    CONSOLE << "Number of images: " << m_image_names.size();
}

void QSfM::run()
{
    featureExtract();
    triangulate();
    bundleAdjustment();
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
    Ptr<AKAZE> feature = AKAZE::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    namedWindow("img", WINDOW_NORMAL);

    // Extract features
    for (auto f : m_image_names) {
        ImagePose a;

        Mat img = imread(f.toStdString());
        assert(!img.empty());

        resize(img, img, img.size()/IMAGE_DOWNSAMPLE);
        a.img = img;
        cvtColor(img, img, COLOR_BGR2GRAY);

        feature->detect(img, a.kp);
        feature->compute(img, a.kp, a.desc);

        m_img_pose.emplace_back(a);
    }

    // Match features between all images
    for (size_t i=0; i < m_img_pose.size()-1; i++) {
        auto &img_pose_i = m_img_pose[i];
        for (size_t j=i+1; j < m_img_pose.size(); j++) {
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

            Mat canvas = img_pose_i.img.clone();
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
            namedWindow("img",cv::WINDOW_NORMAL);

            imshow("img", canvas);
            waitKey(30);
        }
    }
}

void QSfM::triangulate()
{
    // Setup camera matrix
    // It is assumed here that the center of the image is the main point coordinate.
    // If you have an internal reference file, you can replace it, and the image is a de-distorted image.
    double cx = m_img_pose[0].img.size().width/2;
    double cy = m_img_pose[0].img.size().height/2;

    Point2d pp(cx, cy);

    Mat K = Mat::eye(3, 3, CV_64F);

    K.at<double>(0,0) = FOCAL_LENGTH;
    K.at<double>(1,1) = FOCAL_LENGTH;
    K.at<double>(0,2) = cx;
    K.at<double>(1,2) = cy;

    cout << endl << "initial camera matrix K " << endl << K << endl << endl;

    m_img_pose[0].T = Mat::eye(4, 4, CV_64F);
    m_img_pose[0].P = K*Mat::eye(3, 4, CV_64F);

    for (size_t i=0; i < m_img_pose.size() - 1; i++) {
        auto &prev = m_img_pose[i];
        auto &cur = m_img_pose[i+1];

        vector<Point2f> src, dst;
        vector<size_t> kp_used;

        for (size_t k=0; k < prev.kp.size(); k++) {
            if (prev.kp_match_exist(k, i+1)) {
                size_t match_idx = prev.kp_match_idx(k, i+1);

                src.push_back(prev.kp[k].pt);
                dst.push_back(cur.kp[match_idx].pt);

                kp_used.push_back(k);
            }
        }

        Mat mask;

        // NOTE: pose from dst to src,src as the reference frame
        // The first frame is used as the world coordinate system
        Mat E = findEssentialMat(dst, src, FOCAL_LENGTH, pp, RANSAC, 0.999, 1.0, mask);
        Mat local_R, local_t;

        recoverPose(E, dst, src, local_R, local_t, FOCAL_LENGTH, pp, mask);

        // local tansform
        Mat T = Mat::eye(4, 4, CV_64F);
        local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
        local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

        // accumulate transform
        cur.T = prev.T*T; // T matrix is world to camera

        // make projection matrix
        Mat R = cur.T(cv::Range(0, 3), cv::Range(0, 3));
        Mat t = cur.T(cv::Range(0, 3), cv::Range(3, 4));

        Mat P(3, 4, CV_64F); // P matrix is camera to world

        P(cv::Range(0, 3), cv::Range(0, 3)) = R.t();
        P(cv::Range(0, 3), cv::Range(3, 4)) = -R.t()*t;
        P = K*P;

        cur.P = P;

        Mat points4D;
        triangulatePoints(prev.P, cur.P, src, dst, points4D); // triangulate camera to world P =R*p+C

        // Scale the new 3d points to be similar to the existing 3d points (landmark)
        // Use ratio of distance between pairing 3d points
        // Initialize the scale using the distance ratio of a pair of common 3d points
        if (i > 0) {
            double scale = 0;
            int count = 0;

            Point3f prev_camera;

            prev_camera.x = prev.T.at<double>(0, 3);
            prev_camera.y = prev.T.at<double>(1, 3);
            prev_camera.z = prev.T.at<double>(2, 3);

            vector<Point3f> new_pts;
            vector<Point3f> existing_pts;

            for (size_t j=0; j < kp_used.size(); j++) {
                size_t k = kp_used[j];
                if (mask.at<uchar>(j) && prev.kp_match_exist(k, i+1) && prev.kp_3d_exist(k)) {
                    Point3f pt3d;

                    pt3d.x = points4D.at<float>(0, j) / points4D.at<float>(3, j);
                    pt3d.y = points4D.at<float>(1, j) / points4D.at<float>(3, j);
                    pt3d.z = points4D.at<float>(2, j) / points4D.at<float>(3, j);

                    size_t idx = prev.kp_3d(k);
                    Point3f avg_landmark = m_landmark[idx].pt / (m_landmark[idx].seen - 1);

                    new_pts.push_back(pt3d);
                    existing_pts.push_back(avg_landmark);
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

            cout << "image " << (i+1) << " ==> " << i << " scale=" << scale << " count=" << count <<  endl;

            // apply scale and re-calculate T and P matrix
            local_t *= scale;

            // local tansform
            Mat T = Mat::eye(4, 4, CV_64F);
            local_R.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
            local_t.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));

            // accumulate transform
            cur.T = prev.T*T;

            // make projection ,matrix
            R = cur.T(cv::Range(0, 3), cv::Range(0, 3));
            t = cur.T(cv::Range(0, 3), cv::Range(3, 4));

            Mat P(3, 4, CV_64F);
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

                Point3f pt3d;

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

void QSfM::bundleAdjustment()
{
    gtsam::Values result;

    double cx = m_img_pose[0].img.size().width/2;
    double cy = m_img_pose[0].img.size().height/2;

    Cal3_S2 K(FOCAL_LENGTH, FOCAL_LENGTH, 0 /* skew */, cx, cy);
    noiseModel::Isotropic::shared_ptr measurement_noise = noiseModel::Isotropic::Sigma(2, 2.0); // pixel error in (x,y)

    NonlinearFactorGraph graph;
    Values initial;

    // Poses
    for (size_t i=0; i < m_img_pose.size(); i++) {
        auto &img_pose = m_img_pose[i];

        Rot3 R(
                    img_pose.T.at<double>(0,0),
                    img_pose.T.at<double>(0,1),
                    img_pose.T.at<double>(0,2),

                    img_pose.T.at<double>(1,0),
                    img_pose.T.at<double>(1,1),
                    img_pose.T.at<double>(1,2),

                    img_pose.T.at<double>(2,0),
                    img_pose.T.at<double>(2,1),
                    img_pose.T.at<double>(2,2)
                    );

        Point3 t;

        t(0) = img_pose.T.at<double>(0,3);
        t(1) = img_pose.T.at<double>(1,3);
        t(2) = img_pose.T.at<double>(2,3);

        Pose3 pose(R, t);

        // Add prior for the first image
        if (i == 0) {
            noiseModel::Diagonal::shared_ptr pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
            graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), pose, pose_noise); // add directly to graph
        }

        initial.insert(Symbol('x', i), pose);

        // landmark seen
        for (size_t k=0; k < img_pose.kp.size(); k++) {
            if (img_pose.kp_3d_exist(k)) {
                size_t landmark_id = img_pose.kp_3d(k);

                if (m_landmark[landmark_id].seen >= MIN_LANDMARK_SEEN) {
                    Point2 pt;

                    pt(0) = img_pose.kp[k].pt.x;
                    pt(1) = img_pose.kp[k].pt.y;

                    graph.emplace_shared<GeneralSFMFactor2<Cal3_S2>>(pt, measurement_noise, Symbol('x', i), Symbol('l', landmark_id), Symbol('K', 0));
                }
            }
        }
    }

    // Add a prior on the calibration.
    initial.insert(Symbol('K', 0), K);

    noiseModel::Diagonal::shared_ptr cal_noise = noiseModel::Diagonal::Sigmas((Vector(5) << 100, 100, 0.01 /*skew*/, 100, 100).finished());
    graph.emplace_shared<PriorFactor<Cal3_S2>>(Symbol('K', 0), K, cal_noise);

    // Initialize estimate for landmarks
    bool init_prior = false;

    for (size_t i=0; i < m_landmark.size(); i++) {
        if (m_landmark[i].seen >= MIN_LANDMARK_SEEN) {
            cv::Point3f &p = m_landmark[i].pt;

            initial.insert<Point3>(Symbol('l', i), Point3(p.x, p.y, p.z));

            if (!init_prior) {
                init_prior = true;

                noiseModel::Isotropic::shared_ptr point_noise = noiseModel::Isotropic::Sigma(3, 0.1);
                Point3 p(m_landmark[i].pt.x, m_landmark[i].pt.y, m_landmark[i].pt.z);
                graph.emplace_shared<PriorFactor<Point3>>(Symbol('l', i), p, point_noise);
            }
        }
    }

    result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    cout << endl;
    cout << "initial graph error = " << graph.error(initial) << endl;
    cout << "final graph error = " << graph.error(result) << endl;
}

void QSfM::reconstruction()
{

}

void QSfM::savePCD()
{

}
