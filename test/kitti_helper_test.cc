#include <pcl/visualization/pcl_visualizer.h>

#include "KittiHelper.h"

using namespace A_LOAM_MT;

/// @note 这里在测试的过程中，没有对点云的时间戳进行测试，因为使用估计的时间戳进行运动畸变的去处意义不大

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

    pcl::visualization::PCLVisualizer viewer("Kitti Helper");

    auto db = kitti_helper.LoadNext();
    uint8_t r = 255, g = 0, b = 0;
    for (int i = 0; i < 51; ++i) {
        auto cloud_out = Convert(db.pointclouds_[i], r, g, b);
        viewer.addPointCloud(cloud_out, "scan_" + std::to_string(i));
        r = r == 255 ? 0 : 255;
        g = g == 255 ? 0 : 255;
        b = b == 255 ? 0 : 255;
    }

    while (1) {
        cv::imshow("img", db.right_image_);
        cv::waitKey(30);
        viewer.spinOnce();
    }
    cv::destroyAllWindows();

    return 0;
}