#include <rclcpp/rclcpp.hpp>

#include "KittiHelper.h"
#include "LidarOdom.h"

using namespace A_LOAM_MT;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    Mat3 Rlc;
    Rlc << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    KittiHelper::Options kitti_options = {"/media/rookie-lu/新加卷/Dataset/KITTI", "00", SE3(Rlc, Vec3(0, 0, 0))};
    DataHelper::Ptr data_loader = std::make_shared<KittiHelper>(kitti_options);
    auto lidar_odom = std::make_shared<LidarOdom>();
    lidar_odom->SetDataLoader(data_loader);
    lidar_odom->Run();

    rclcpp::shutdown();

    return 0;
}