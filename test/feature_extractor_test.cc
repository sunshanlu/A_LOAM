#include <pcl/visualization/pcl_visualizer.h>

#include "FeatureExtractor.h"
#include "KittiHelper.h"

using namespace A_LOAM_MT;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Convert(PointCloudXYZTL::Ptr cloud, uint8_t r, uint8_t g, uint8_t b) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &pt : *cloud) {
        pcl::PointXYZRGB pt_out;
        pt_out.x = pt.x;
        pt_out.y = pt.y;
        pt_out.z = pt.z;
        pt_out.r = r;
        pt_out.g = g;
        pt_out.b = b;
        cloud_out->push_back(pt_out);
    }
    return cloud_out;
}

int main(int argc, char **argv) {
    Mat3 Rlc;
    Rlc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    KittiHelper::Options kitti_options = {"/media/rookie-lu/新加卷/Dataset/KITTI", "00", SE3(Rlc, Vec3(0, 0, 0))};
    KittiHelper kitti_helper(kitti_options);
    pcl::visualization::PCLVisualizer viewer("Feature Extraction");

    auto db = kitti_helper.LoadNext();

    FeatureExtractor extractor;
    extractor.SetTarget(db.pointclouds_);
    extractor.Extract();

    auto less_corner_points = Convert(extractor.less_corner_points_, 255, 0, 0);
    auto less_surf_points = Convert(extractor.less_surf_points_, 0, 255, 0);
    auto corner_points = Convert(extractor.corner_points_, 0, 0, 255);
    auto surf_points = Convert(extractor.surf_points_, 255, 255, 255);

    // viewer.addPointCloud(less_corner_points, "less_corner_points");
    // viewer.addPointCloud(less_surf_points, "less_surf_points");
    viewer.addPointCloud(corner_points, "corner_points");
    viewer.addPointCloud(surf_points, "surf_points");

    while (1){
        cv::imshow("img", db.right_image_);
        cv::waitKey(30);
        viewer.spinOnce();
    }
    cv::destroyAllWindows();

    return 0;
}
