#include "FeatureExtractor.h"

NAMESPACE_BEGIN

/**
 * @brief 提取laser_scans_中的角点和平面点特征
 * @details
 *  1. 针对某一scan，进行曲率计算
 *  2. 针对某个scan，进行平均划分（涉及到划分几份的问题）
 *  3. 针对某一scan的某一部分，特征点提取
 *      (1) 针对某一部分的曲率信息，从大到小提取角点信息（涉及到强角点和弱角点提取数目）
 *      (2) 针对某一部分的曲率信息，从小到大提取平面点信息（涉及到强平面点和弱平面点提取数目）
 *      (3) 对某一提取成功的特征点，要保证该线束的一定范围内，不能被再被提取特征（涉及到提取的范围距离）
 */
void FeatureExtractor::Extract() {
    corner_points_->clear();
    less_corner_points_->clear();
    surf_points_->clear();
    less_corner_points_->clear();

    for (const auto &scan : laser_scans_) {
        /// 计算曲率
        std::vector<double> curvature = ComputeCurvature(scan);
        std::vector<int> curvature_index(curvature.size());
        std::for_each(curvature_index.begin(), curvature_index.end(), [id = 0](int &idx) mutable { idx = id++; });

        std::vector<bool> points_picked(curvature.size(), false); ///< 用于统计点是否被提取
        std::vector<int> point_level(curvature.size(), 0);        ///< 用于统计点的等级（0代表弱平面点）

        int begin_id = 0, end_id = 0;
        for (int i = 0; i < options_.part_num; ++i) {
            begin_id = end_id;
            if (i == options_.part_num - 1)
                end_id = curvature.size();
            else
                end_id = begin_id + (curvature.size() / options_.part_num) + 1;

            auto begin_iter = curvature_index.begin() + begin_id;
            auto end_iter = curvature_index.begin() + end_id;

            /// 针对某一scan的某一部分进行曲率排序
            std::sort(begin_iter, end_iter,
                      [&curvature](const int &a, const int &b) { return curvature[a] < curvature[b]; });

            /// 从大到小曲率，提取角点信息
            int corner_picked_num = 0;
            for (int j = end_id - 1; j >= begin_id; --j) {
                int curvature_id = curvature_index[j];
                if (curvature[curvature_id] < options_.curvature_th)
                    break;

                if (!points_picked[curvature_id] && corner_picked_num < options_.corner_points_num) {
                    corner_points_->push_back(scan->points[curvature_id + 5]);
                    less_corner_points_->push_back(scan->points[curvature_id + 5]);
                    point_level[curvature_id] = 2;
                    points_picked[curvature_id] = true;
                    ++corner_picked_num;
                } else if (!points_picked[curvature_id] && corner_picked_num < options_.less_corner_points_num) {
                    less_corner_points_->push_back(scan->points[curvature_id + 5]);
                    point_level[curvature_id] = 1;
                    points_picked[curvature_id] = true;
                    ++corner_picked_num;
                }

                /// 当特征点被成功提取时，该特征点部分的前后5个点的位置，如果两两之间的距离平方小于0.05，则认为其前后位置的点不可取
                if (points_picked[curvature_id]) {
                    for (int l = 1; l <= 5; ++l) {
                        float diffX = scan->points[curvature_id + l + 5].x - scan->points[curvature_id + l + 5 - 1].x;
                        float diffY = scan->points[curvature_id + l + 5].y - scan->points[curvature_id + l + 5 - 1].y;
                        float diffZ = scan->points[curvature_id + l + 5].z - scan->points[curvature_id + l + 5 - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > options_.min_distance_th2) {
                            break;
                        }
                        if (curvature_id + l > 0 && curvature_id + l < curvature.size())
                            points_picked[curvature_id + l] = true;
                    }
                    for (int l = -1; l >= -5; --l) {
                        float diffX = scan->points[curvature_id + l + 5].x - scan->points[curvature_id + l + 5 + 1].x;
                        float diffY = scan->points[curvature_id + l + 5].y - scan->points[curvature_id + l + 5 + 1].y;
                        float diffZ = scan->points[curvature_id + l + 5].z - scan->points[curvature_id + l + 5 + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > options_.min_distance_th2) {
                            break;
                        }
                        if (curvature_id + l > 0 && curvature_id + l < curvature.size())
                            points_picked[curvature_id + l] = true;
                    }
                }
            }

            /// 从小到大曲率，提取平面点信息
            int surf_picked_num = 0;
            for (int j = begin_id; j < end_id; ++j) {
                int curvature_id = curvature_index[j];
                if (curvature[curvature_id] >= options_.curvature_th)
                    break;

                if (!points_picked[curvature_id] && surf_picked_num < options_.surf_points_num) {
                    surf_points_->push_back(scan->points[curvature_id + 5]);
                    point_level[curvature_id] = -1;
                    points_picked[curvature_id] = true;
                    ++surf_picked_num;
                }
                if (points_picked[curvature_id]) {
                    for (int l = 1; l <= 5; ++l) {
                        float diffX = scan->points[curvature_id + l + 5].x - scan->points[curvature_id + l + 5 - 1].x;
                        float diffY = scan->points[curvature_id + l + 5].y - scan->points[curvature_id + l + 5 - 1].y;
                        float diffZ = scan->points[curvature_id + l + 5].z - scan->points[curvature_id + l + 5 - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > options_.min_distance_th2) {
                            break;
                        }
                        if (curvature_id + l > 0 && curvature_id + l < curvature.size())
                            points_picked[curvature_id + l] = true;
                    }
                    for (int l = -1; l >= -5; --l) {
                        float diffX = scan->points[curvature_id + l + 5].x - scan->points[curvature_id + l + 5 + 1].x;
                        float diffY = scan->points[curvature_id + l + 5].y - scan->points[curvature_id + l + 5 + 1].y;
                        float diffZ = scan->points[curvature_id + l + 5].z - scan->points[curvature_id + l + 5 + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > options_.min_distance_th2) {
                            break;
                        }
                        if (curvature_id + l > 0 && curvature_id + l < curvature.size())
                            points_picked[curvature_id + l] = true;
                    }
                }

                if (surf_picked_num >= options_.surf_points_num)
                    break;
            }

            /// 提取弱角点位置
            for (int i = 0; i < point_level.size(); ++i) {
                if (point_level[i] <= 0)
                    less_surf_points_->push_back(scan->points[i + 5]);
            }
        }
    }
    DownSizeLessSurf();
}

/**
 * @brief 对平面点进行降采样
 *
 */
void FeatureExtractor::DownSizeLessSurf() {
    voxel_filter_.setInputCloud(less_surf_points_);
    voxel_filter_.filter(*less_surf_points_);
}

/**
 * @brief 计算点云曲率信息
 *
 * @param scan 输入的lader扫描数据
 * @return std::vector<double> 输出的扫描数据的曲率信息
 */
std::vector<double> FeatureExtractor::ComputeCurvature(const PointCloudXYZTL::ConstPtr &scan) {
    std::vector<double> curvature(scan->size() - 10, 0);
    for (int i = 5; i < scan->size() - 5; ++i) {
        const auto &point0 = scan->points[i - 5];
        const auto &point1 = scan->points[i - 4];
        const auto &point2 = scan->points[i - 3];
        const auto &point3 = scan->points[i - 2];
        const auto &point4 = scan->points[i - 1];
        const auto &point = scan->points[i];
        const auto &point6 = scan->points[i + 1];
        const auto &point7 = scan->points[i + 2];
        const auto &point8 = scan->points[i + 3];
        const auto &point9 = scan->points[i + 4];
        const auto &point10 = scan->points[i + 5];
        float diff_x = point0.x + point1.x + point2.x + point3.x + point4.x + point6.x + point7.x + point8.x +
                       point9.x + point10.x - 10 * point.x;
        float diff_y = point0.y + point1.y + point2.y + point3.y + point4.y + point6.y + point7.y + point8.y +
                       point9.y + point10.y - 10 * point.y;
        float diff_z = point0.z + point1.z + point2.z + point3.z + point4.z + point6.z + point7.z + point8.z +
                       point9.z + point10.z - 10 * point.z;
        curvature[i - 5] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    }
    return curvature;
}

NAMESPACE_END