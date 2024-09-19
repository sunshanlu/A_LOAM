#include <fstream>
#include <iomanip>
#include <sstream>

#include "KittiHelper.h"

NAMESPACE_BEGIN

/**
 * @brief 加载Kitti数据信息
 * @details
 *      1. 帧的时间戳数据
 *      2. 激光雷达数据（分线束信息[0, 50]）
 *      3. 左图图像数据
 *      4. 里程计GroundTruth
 * @return DataHelper::DataBag  定义的Kitti数据包（一帧）
 */
DataHelper::DataBag KittiHelper::LoadNext() {
    static std::ifstream ifs_stamp(stamp_path_);
    static std::ifstream ifs_gt(gt_path_);
    std::string line_stamp, line_gt;
    if (!std::getline(ifs_stamp, line_stamp) || !std::getline(ifs_gt, line_gt))
        return {0, DataBag::PointClouds(), cv::Mat(), SE3(), false};

    std::stringstream ss_id, ss_stamp(line_stamp);
    ss_id << std::setw(6) << std::setfill('0') << frame_id_++;
    str_id_ = ss_id.str();

    DataBag db;
    ss_stamp >> db.stamp_;
    db.pointclouds_ = LoadPointCloud();
    db.right_image_ = LoadImage();
    db.ref_pose_ = LoadGTruth(line_gt);
    return db;
}

/**
 * @brief 加载Kitti的Velodyne64E的点云数据
 * @details
 *      1. 针对atan2(z, std::sqrt(x*x + y*y))判断线束信息
 *      2. Velodyne HDL 64E的旋转特性是顺时针还是逆时针？判断点云的时间戳信息（这种方式不够准确！）
 *          2.1 原因在于，使用这种方式判断Velodyne的数据时，畸变已经存在，因此推算出的时间戳本身就不准确
 *          2.2 由于时间戳本身不准确，那么使用这个时间戳进行点云去畸变就更没有意义了
 *          2.3 除此之外，点云的线束信息也是通过角度进行估计的，这就更加剧了时间戳无意义的情况
 * @return DataHelper::DataBag::PointClouds 里面有51个线束的点云信息
 */
DataHelper::DataBag::PointClouds KittiHelper::LoadPointCloud() {
    std::string lidar_file = velo_path_ + str_id_ + ".bin";
    std::ifstream ifs_lidar(lidar_file, std::ios::in | std::ios::binary);
    ifs_lidar.seekg(0, std::ios::end);
    const std::size_t num_element = ifs_lidar.tellg() / sizeof(float);
    ifs_lidar.seekg(0, std::ios::beg);

    std::vector<float> lidar_buffer(num_element);
    ifs_lidar.read(reinterpret_cast<char *>(&lidar_buffer[0]), num_element * sizeof(float));

    /// 转换针对Veloyne64E的数据处理
    DataBag::PointClouds pointclouds(51);
    for (int i = 0; i < 51; ++i)
        pointclouds[i] = PointCloudXYZTL::Ptr(new PointCloudXYZTL);

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
        pointclouds[scan_id]->push_back(point);
    }

    std::vector<double> scan_start(51);
    for (int i = 0; i < 51; ++i) {
        auto point_start = pointclouds[i]->points.front();
        scan_start[i] = std::atan2(point_start.y, point_start.x);
        // bool pass_negative = false;
        bool pass_positive = false;
        std::vector<float> times;
        for (auto &point : pointclouds[i]->points) {
            double angle = std::atan2(point.y, point.x);
            
            /// 使用顺时针判断时间偏置的时候，会出现负时间戳的情况
            // if (!pass_negative && angle < 0)
            //     pass_negative = true;
            // if (pass_negative && angle > 0)
            //     point.time = ((2 * M_PI - angle + scan_start[i]) / (2 * M_PI)) * 0.1;
            // else
            //     point.time = ((scan_start[i] - angle) / (2 * M_PI)) * 0.1;
            
            /// 由于顺时针估计的时间戳偏差较大，这里尝试使用逆时针的方式判断时间戳
            if (!pass_positive && angle > 0)
                pass_positive = true;
            if (pass_positive && angle < 0)
                point.time = (2 * M_PI - (scan_start[i] - angle)) / (2 * M_PI) * 0.1;
            else
                point.time = (angle - scan_start[i]) / (2 * M_PI) * 0.1;
            times.push_back(point.time);
        }
        // std::cout << "scan " << i << "finished!" << std::endl;
    }

    return pointclouds;
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
    Quat q(R);
    q.normalize();
    return options_.Tlc * SE3(q, t) * options_.Tlc.inverse();
}

NAMESPACE_END
