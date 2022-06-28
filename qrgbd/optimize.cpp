#include "optimize.h"

SurfelCloudPtr Optimize::reconstructionSurface(
    const PointCloudPtr &input,
    float radius,
    int polynomial_order) {
    pcl::MovingLeastSquares<PointT, SurfelT> mls;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(radius * radius);
    //    mls.setPolynormialFit(polynomial_order > 1);
    mls.setPolynomialOrder(polynomial_order);
    mls.setInputCloud(input);
    SurfelCloudPtr output(new SurfelCloud);
    mls.process(*output);
    return (output);
}

pcl::PolygonMeshPtr Optimize::triangulateMesh(const SurfelCloudPtr &surfels) {

    pcl::search::KdTree<SurfelT>::Ptr tree(new pcl::search::KdTree<SurfelT>);
    tree->setInputCloud(surfels);

    pcl::GreedyProjectionTriangulation<SurfelT> gp3;
    pcl::PolygonMeshPtr triangles(new pcl::PolygonMesh);

    gp3.setSearchRadius(0.1);

    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(M_PI / 3);
    gp3.setNormalConsistency(true);

    gp3.setInputCloud(surfels);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);

    return triangles;
}

pcl::TextureMeshPtr Optimize::MashTextured(const pcl::PolygonMeshPtr &mesh) {
}

bool Optimize::ICPTrack(const PointCloudPtr &source, const PointCloudPtr &target, Sophus::SE3f &Tts) {

    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(0.02, 0.02, 0.02);
    filter.setInputCloud(source);
    filter.filter(*source);
    filter.setInputCloud(target);
    filter.filter(*target);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(30);
    icp.align(*source);

    if (icp.hasConverged()) {
        Eigen::Matrix4f trans = icp.getFinalTransformation();
        auto rot = trans.block(0, 0, 3, 3);
        //

        // normalize rows
        Eigen::Vector3f row1 = rot.row(0);
        Eigen::Vector3f row2 = rot.row(1).normalized();
        Eigen::Vector3f row3 = row1.cross(row2).normalized();
        row1 = row2.cross(row3);
        rot.row(0) = row1;
        rot.row(1) = row2;
        rot.row(2) = row3;

        // normalize cols
        Eigen::Vector3f col1 = rot.col(0);
        Eigen::Vector3f col2 = rot.col(1).normalized();
        Eigen::Vector3f col3 = col1.cross(col2).normalized();
        col1 = col2.cross(col3);
        rot.col(0) = col1;
        rot.col(1) = col2;
        rot.col(2) = col3;

        trans.block(0, 0, 3, 3) = rot;
        //
        Tts = Sophus::SE3f(trans);
        return true;
    }
    return false;
}
