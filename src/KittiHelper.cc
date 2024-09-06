#include <fstream>
#include <iomanip>
#include <sstream>

#include "KittiHelper.h"

NAMESPACE_BEGIN

/// todo 这里的getline();到底是怎么截止的？
KittiHelper::DataBag KittiHelper::LoadNext() {
    static std::ifstream ifs_stamp(stamp_path_);
    static std::ifstream ifs_gt(gt_path_);
    std::string line_stamp, line_gt;
    if (!std::getline(ifs_stamp, line_stamp) || !std::getline(ifs_gt, line_gt))
        return {0, nullptr, cv::Mat(), SE3(), false};

    std::stringstream ss_id, ss_stamp(line_stamp);
    ss_id << std::setw(6) << std::setfill('0') << frame_id_++;
    str_id_ = ss_id.str();

    DataBag db;
    ss_stamp >> db.stamp_;
    db.pointcloud_ = LoadPointCloud();
    db.right_image_ = LoadImage();
    db.ref_pose_ = LoadGTruth(line_gt);
}

PointCloudXYZTL::Ptr* KittiHelper::LoadPointCloud() {
    std::string lidar_file = velo_path_ + str_id_ + ".bin";
    std::ifstream ifs_lidar(lidar_file, std::ios::in | std::ios::binary);
    ifs_lidar.seekg(0, std::ios::end);
    const std::size_t num_element = ifs_lidar.tellg() / sizeof(float);
    ifs_lidar.seekg(0, std::ios::beg);

    std::vector<float> lidar_buffer(num_element);
    ifs_lidar.read(reinterpret_cast<char *>(&lidar_buffer[0]), num_element * sizeof(float));

    /// 转换针对Veloyne64E的数据处理
    PointCloudXYZTL::Ptr pointcloud[51];
    for (int i = 0; i < 51; ++i)
        pointcloud[i] = PointCloudXYZTL::Ptr(new PointCloudXYZTL);

    for (int i = 0; i < num_element; i += 4) {
        float x = lidar_buffer[i];
        float y = lidar_buffer[i + 1];
        float z = lidar_buffer[i + 2];

        if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(lidar_buffer[i + 3]))
            continue;

        float angle = std::atan2(z, std::sqrt(x * x + y * y)) * rad2deg;
        int scan_id = 0;
        if (angle >= -8.83)
            scan_id = int((2 - angle) * 3.0 + 0.5);
        else
            scan_id = 32 + int((-8.83 - angle) * 2.0 + 0.5);

        if (angle > 2 || angle < -24.33 || scan_id > 50 || scan_id < 0)
            continue;

        PointXYZTL point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.ring = scan_id;
        pointcloud[scan_id]->push_back(point);
    }

    std::vector<double> scan_start(51);
    for (int i = 0; i < 51; ++i) {
        auto point_start = pointcloud[i]->points.front();
        scan_start[i] = std::atan2(point_start.y, point_start.x);
        bool pass_negative = false;
        for (auto &point : pointcloud[i]->points) {
            float angle = std::atan2(point.y, point.x);
            if (!pass_negative && angle < 0)
                pass_negative = true;
            if (pass_negative && angle > 0)
                point.time = ((2 * M_PI - angle + scan_start[i]) / (2 * M_PI)) * 0.1;
            else
                point.time = ((scan_start[i] - angle) / (2 * M_PI)) * 0.1;
        }
    }

    return pointcloud;
}

/**
 * @brief 读取左侧图像
 *
 * @return cv::Mat 返回左侧图像的cv::Mat数据
 */
cv::Mat KittiHelper::LoadImage() {
    cv::Mat left_image = cv::imread(img_path_ + str_id_ + ".png", cv::IMREAD_GRAYSCALE);
    return left_image;
}

/**
 * @brief 加载GroundTruth
 * @details
 *      1. 读取的位姿是相机里程计的位姿
 *      2. 因此需要将位姿转换成雷达坐标系下
 * @param pose_str 输入的读取的行数据，包含一个3 * 4的位姿矩阵
 * @return SE3 输出由李群表示的激光里程计位姿
 */
SE3 KittiHelper::LoadGTruth(const std::string &pose_str) {
    std::stringstream ss_pose(pose_str);
    double pose[12];
    for (int i = 0; i < 12; ++i)
        ss_pose >> pose[i];

    Eigen::Map<Mat3_4, Eigen::RowMajor> pose_eig(pose);
    Mat3 R = pose_eig.block<3, 3>(0, 0);
    Vec3 t = pose_eig.block<3, 1>(0, 3);
    return options_.Tlc * SE3(R, t) * options_.Tlc.inverse();
}

NAMESPACE_END
