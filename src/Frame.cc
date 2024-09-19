#include <Frame.h>

NAMESPACE_BEGIN

/**
 * @brief this作为当前帧，用于寻找和last_frame的角点关联关系
 * @details
 *      1. 使用this的强角点和last_frame的弱角点kd_tree进行最近邻搜索
 *      2. 使用该最近邻的角点线束信息，在其[-2, 2]的5条线束内，寻找两一个最近邻
 *      3. 存储对应点信息，并返回
 * @param last_frame 上一帧
 * @param Tlc 上一帧到当前帧的位姿变换
 */
void Frame::FindCornerCorr(Frame::Ptr last_frame, const SE3 &Tlc) {
    line_params_.clear();
    auto last_corner_points = last_frame->less_corner_points_;

    // int corr_num = 0;
    // Vec3 diff_sum(0, 0, 0);
    
    for (int i = 0; i < corner_points_->size(); ++i) {
        // todo 当前点的去畸变处理（针对Tlc可以对corner_points_->points[i]进行去畸变）

        Vec3 p_eig = Vec3(corner_points_->points[i].x, corner_points_->points[i].y, corner_points_->points[i].z);
        p_eig = Tlc * p_eig;
        PointXYZTL curr_point;
        curr_point.x = p_eig(0);
        curr_point.y = p_eig(1);
        curr_point.z = p_eig(2);
        curr_point.time = corner_points_->points[i].time;
        curr_point.ring = corner_points_->points[i].ring;

        /// 寻找最近邻和次近邻
        std::vector<int> search_id;
        std::vector<float> search_dist;
        last_frame->kdtree_corner_->nearestKSearch(curr_point, 1, search_id, search_dist);
        int closest_id = -1, min_id = -1;
        if (search_dist[0] < options_->distance_th) {
            closest_id = search_id[0];
            double min_dist = options_->distance_th;
            int closest_ring = last_corner_points->points[closest_id].ring;

            /// closest_id后，寻找次近邻
            for (int j = closest_id + 1; j < corner_points_->size(); ++j) {
                if (last_corner_points->points[j].ring > closest_ring + options_->surround_scanlines)
                    break;
                double distance = std::sqrt(Distance2(curr_point, last_corner_points->points[j]));
                if (distance < min_dist) {
                    min_dist = distance;
                    min_id = j;
                }
            }

            /// closest_id前，寻找次近邻
            for (int j = closest_id - 1; j >= 0; --j) {
                if (last_corner_points->points[j].ring < closest_ring - options_->surround_scanlines)
                    break;
                double distance = std::sqrt(Distance2(curr_point, last_corner_points->points[j]));
                if (distance < min_dist) {
                    min_dist = distance;
                    min_id = j;
                }
            }
        }

        LineParam line_param;
        if (closest_id > 0 && min_id > 0) {
            auto pcl_pt0 = last_corner_points->points[closest_id];
            auto pcl_pt1 = last_corner_points->points[min_id];
            Vec3 p0 = Vec3(pcl_pt0.x, pcl_pt0.y, pcl_pt0.z);
            Vec3 p1 = Vec3(pcl_pt1.x, pcl_pt1.y, pcl_pt1.z);
            Vec3 d = (p1 - p0).normalized();
            line_param.d = d;
            line_param.p0 = p0;
            line_param.valid = true;

            /// 统计点的信息，大致确定之间的距离
            // diff_sum += (p0 + p1) / 2 - p_eig;
            // diff_sum += p0 - p_eig;
            // ++corr_num;
        } else
            line_param.valid = false;

        line_params_.push_back(line_param);
    }
    // diff_sum /= corr_num;
    // std::cout << diff_sum.transpose() << std::endl;
}

/**
 * @brief this作为当前帧，用于寻找和last_frame的平面点关联关系
 *
 * @param last_frame 上一帧
 * @param Tlc 上一帧到当前帧的位姿变换
 */
void Frame::FindSurfCorr(Frame::Ptr last_frame, const SE3 &Tlc) {
    plane_params_.clear();

    auto last_surf_points = last_frame->less_surf_points_;
    for (int i = 0; i < surf_points_->size(); ++i) {
        /// todo 点云去畸变

        Vec3 p_eig = Vec3(surf_points_->points[i].x, surf_points_->points[i].y, surf_points_->points[i].z);
        p_eig = Tlc * p_eig;
        PointXYZTL curr_point;
        curr_point.x = p_eig(0);
        curr_point.y = p_eig(1);
        curr_point.z = p_eig(2);
        curr_point.time = surf_points_->points[i].time;
        curr_point.ring = surf_points_->points[i].ring;

        /// 寻找最近邻和次近邻
        std::vector<int> search_id;
        std::vector<float> search_dist;
        last_frame->kdtree_surf_->nearestKSearch(curr_point, 1, search_id, search_dist);

        int nearest_id = -1, min_id0 = -1, min_id1 = -1;
        if (search_dist[0] < options_->distance_th) {
            nearest_id = search_id[0];
            int nearest_ring = last_surf_points->points[nearest_id].ring;
            int min_dist0 = options_->distance_th, min_dist1 = options_->distance_th;

            /// nearest_id的后方，寻找次近邻
            for (int j = nearest_id + 1; j < surf_points_->size(); ++j) {
                if (last_surf_points->points[j].ring > nearest_ring + options_->surround_scanlines)
                    break;
                double distance = std::sqrt(Distance2(curr_point, last_surf_points->points[j]));
                if (distance < min_dist0) {
                    min_dist0 = distance;
                    min_id0 = j;
                } else if (distance < min_dist1) {
                    min_dist1 = distance;
                    min_id1 = j;
                }
            }

            /// nearest_id的前方，寻找次近邻
            for (int j = nearest_id - 1; j >= 0; --j) {
                if (last_surf_points->points[j].ring < nearest_ring - options_->surround_scanlines)
                    break;
                double distance = std::sqrt(Distance2(curr_point, last_surf_points->points[j]));
                if (distance < min_dist0) {
                    min_dist0 = distance;
                    min_id0 = j;
                } else if (distance < min_dist1) {
                    min_dist1 = distance;
                    min_id1 = j;
                }
            }
        }

        PlaneParam plane_param;
        if (nearest_id > 0 && min_id0 > 0 && min_id1 > 0) {
            PointXYZTL pcl_p0 = last_surf_points->points[nearest_id];
            PointXYZTL pcl_p1 = last_surf_points->points[min_id0];
            PointXYZTL pcl_p2 = last_surf_points->points[min_id1];
            Vec3 p0 = Vec3(pcl_p0.x, pcl_p0.y, pcl_p0.z);
            Vec3 p1 = Vec3(pcl_p1.x, pcl_p1.y, pcl_p1.z);
            Vec3 p2 = Vec3(pcl_p2.x, pcl_p2.y, pcl_p2.z);

            Mat3_4 A;
            A.row(0).head<3>() = p0.transpose();
            A.row(1).head<3>() = p1.transpose();
            A.row(2).head<3>() = p2.transpose();
            A.col(3) = Vec3::Ones();

            Eigen::JacobiSVD svd(A, Eigen::ComputeFullV);
            Vec4 plane_coeffs = svd.matrixV().col(3);
            plane_coeffs /= plane_coeffs(3);

            plane_param.n = plane_coeffs.block<3, 1>(0, 0);
            double norm = plane_param.n.norm();
            plane_param.n = plane_param.n / norm;
            plane_param.d = 1 / norm;
            plane_param.valid = true;
        } else
            plane_param.valid = false;

        plane_params_.push_back(plane_param);
    }
}

/**
 * @brief 寻找强角点和线特征对应关系、强平面点和平面特征对应关系
 * @details
 *      1. 强角点和线特征对应关系
 *      2. 强平面点和平面特征对应关系
 * @param last_frame 上一帧
 * @param Tlc 上一帧到当前帧的变换
 */
void Frame::FindCorrWithLast(Frame::Ptr last_frame, const SE3 &Tlc) {
    FindCornerCorr(last_frame, Tlc);
    FindSurfCorr(last_frame, Tlc);
}

/**
 * @brief Frame的工厂函数
 *
 * @param laser_scans   输入的点云数据，每个扫描线的数据分开
 * @param timestamp     时间戳数据
 * @param left_image    左目图像，可能不存在，为kitti数据设计
 * @return Frame::Ptr   返回一个Frame的指针
 */
Frame::Ptr Frame::Create(LaserScans laser_scans, double timestamp, cv::Mat left_image) {
    static Options::Ptr options(new Options());
    Frame::Ptr frame(new Frame(options, FeatureExtractor::Options(), laser_scans, timestamp, left_image));
    return frame;
}

/**
 * @brief Frame的唯一构造函数
 *
 * @param frame_options   帧参数配置
 * @param extract_options 特征提取参数配置
 * @param laser_scans     输入的点云数据，每个扫描线的数据分开
 * @param timestamp       时间戳数据
 * @param left_image      左目图像，可能不存在，为kitti数据设计
 */
Frame::Frame(Options::Ptr frame_options, FeatureExtractor::Options extract_options, LaserScans laser_scans,
             double timestamp, cv::Mat left_image)
    : options_(std::move(frame_options))
    , timestamp_(std::move(timestamp))
    , left_image_(std::move(left_image)) {
    FeatureExtractor::Ptr extractor = std::make_shared<FeatureExtractor>(extract_options);
    extractor->SetTarget(laser_scans);
    extractor->Extract();

    less_corner_points_ = extractor->less_corner_points_;
    less_surf_points_ = extractor->less_surf_points_;
    corner_points_ = extractor->corner_points_;
    surf_points_ = extractor->surf_points_;

    kdtree_corner_ = pcl::make_shared<pcl::KdTreeFLANN<PointXYZTL>>();
    kdtree_surf_ = pcl::make_shared<pcl::KdTreeFLANN<PointXYZTL>>();
    kdtree_corner_->setInputCloud(less_corner_points_);
    kdtree_surf_->setInputCloud(less_surf_points_);

    frame_id_ = next_id_++;
}

/// Frame的静态成员变量
std::uint8_t Frame::next_id_ = 0;

NAMESPACE_END