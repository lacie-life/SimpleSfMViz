#include "SfM/QSimpleSfMHepler.h"
#include <QDebug>
#include "AppConstant.h"

QSimpleSfMHepler::QSimpleSfMHepler(QString path, bool isGPU, bool matchcheck)
{
    m_configPath = path;

    this->isGPU = isGPU;
    this->matchcheck = matchcheck;

    init();
}

void QSimpleSfMHepler::init(){

    cv::FileStorage fs(argv[1], cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout << "YAML file : " << argv[1] << " can't not open!" << std::endl;
        emit sfmInitFailed();
        return;
    }

    // Feature extraction
    fs["images_path"] >> m_imagesPath;
    fs["database_path"] >> m_databasePath;
    fs["SIFTextractor.max_image_size"] >> m_max_image_size;
    fs["SIFTextractor.num_features"] >> m_num_features;
    fs["SIFTextractor.normalization"] >> m_normalization_type;

    assert(m_max_image_size > 0);
    assert(m_num_features > 0);
    assert(m_normalization_type == 0 || m_normalization_type == 1 || m_normalization_type == 2);

    if (m_normalization_type == 0)
    {
        m_extractor = cv::Ptr<SimpleSfM::FeatureExtractorCPU>(new SimpleSfM::FeatureExtractorCPU(m_databasePath, m_imagesPath, m_max_image_size, m_num_features,
                                                                         SimpleSfM::FeatureExtractor::Normalization::L1_ROOT));
    }
    else if (m_normalization_type == 1)
    {
        m_extractor = cv::Ptr<SimpleSfM::FeatureExtractorCPU>(new SimpleSfM::FeatureExtractorCPU(m_databasePath, m_imagesPath, m_max_image_size, m_num_features,
                                                                         SimpleSfM::FeatureExtractor::Normalization::L2));
    }
    else
    {
        m_extractor = cv::Ptr<SimpleSfM::FeatureExtractorCPU>(new SimpleSfM::FeatureExtractorCPU(m_databasePath, m_imagesPath, m_max_image_size, m_num_features,
                                                                         SimpleSfM::FeatureExtractor::Normalization::ROOT_SIFT));
    }

    // Feature matching
    fs["SIFTmatch.match_type"] >> m_match_type;
    fs["SIFTmatch.max_distance"] >> m_max_distance;
    fs["SIFTmatch.distance_ratio"] >> m_distance_ratio;
    fs["SIFTmatch.cross_check"] >> m_cross_check;

    assert(m_match_type == 0 || m_match_type == 1);

    if (m_match_type == 0)
    {
        m_matcher = cv::Ptr<SimpleSfM::SequentialFeatureMatcher>(new SimpleSfM::SequentialFeatureMatcher(m_databasePath));
    }
    else
    {
        m_matcher = cv::Ptr<SimpleSfM::BruteFeatureMatcher>(new SimpleSfM::BruteFeatureMatcher(m_databasePath));
    }

    // Reconstruction
    fs["Reconstruction.Camera.fx"] >> m_params.fx;
    fs["Reconstruction.Camera.fy"] >> m_params.fy;
    fs["Reconstruction.Camera.cx"] >> m_params.cx;
    fs["Reconstruction.Camera.cy"] >> m_params.cy;

    fs["Reconstruction.Camera.k1"] >> m_params.k1;
    fs["Reconstruction.Camera.k2"] >> m_params.k2;
    fs["Reconstruction.Camera.p1"] >> m_params.p1;
    fs["Reconstruction.Camera.p2"] >> m_params.p2;

    fs["Reconstruction.output_path"] >> m_outputPath;
    fs["Reconstruction.is_visualization"] >> m_params.is_visualization;

    m_mapBuilder = SimpleSfM::MapBuilder(m_databasePath, m_params);
}

bool QSimpleSfMHepler::run(){

    CONSOLE << "SimpleSfM started...";

    m_timer.start();
    extractor->RunExtraction();
    m_timer.PrintMinutes();

    m_timer.Start();
    m_matcher->RunMatching();
    m_timer.PrintMinutes();

    if(matchcheck)
    {
        m_database.Open(m_databasePath);

        std::vector<std::pair<image_pair_t, std::vector<cv::DMatch>>> all_matches = m_database.ReadAllMatches();

        for (const auto &matches : all_matches)
        {
            SimpleSfM::image_t image_id1, image_id2;
            SimpleSfM::Database::PairIdToImagePair(matches.first, &image_id1, &image_id2);

            SimpleSfM::Database::Image image1 = m_database.ReadImageById(image_id1);
            SimpleSfM::Database::Image image2 = m_database.ReadImageById(image_id2);

            cv::Mat cv_image1 = cv::imread(UnionPath(images_path, image1.name));
            cv::Mat cv_image2 = cv::imread(UnionPath(images_path, image2.name));

            std::vector<cv::KeyPoint> kpts1 = m_database.ReadKeyPoints(image_id1);
            std::vector<cv::KeyPoint> kpts2 = m_database.ReadKeyPoints(image_id2);

            std::vector<cv::Point2f> pts1, pts2;
            cv::KeyPoint::convert(kpts1, pts1);
            cv::KeyPoint::convert(kpts2, pts2);
            std::cout << image_id1 << " -- " << image_id2 << " : " << matches.second.size() << std::endl;
            SimpleSfM::FeatureUtils::ShowMatches(cv_image1, cv_image2, pts1, pts2, matches.second, "Matching Results", 1);
        }

        database.Close();
    }

    m_mapBuilder.SetUp();
    m_mapBuilder.DoBuild();

    m_mapBuilder.WriteCOLMAP(m_outputPath);
    m_mapBuilder.WriteOpenMVS(m_outputPath);
    m_mapBuilder.WritePLY(Utils::UnionPath(m_outputPath, "points3D.ply"));
    m_mapBuilder.WritePLYBinary(Utils::UnionPath(m_outputPath, "points3D_binary.ply"));

    return true;
}

