#pragma once

#include "Common.hpp"

NAMESPACE_BEGIN

class FeatureExtractor {
public:
    typedef std::shared_ptr<FeatureExtractor> Ptr;
    typedef std::vector<PointCloudXYZTL::Ptr> LaserScans;

    /// @brief 特征提取的配置信息
    struct Options {
        Options()
            : part_num(6)
            , corner_points_num(2)
            , surf_points_num(4)
            , less_corner_points_num(20)
            , min_distance_th2(0.05)
            , curvature_th(0.1)
            , down_size_radius(0.2) {}

        int part_num = 6;                ///< scan被平均分为几部分
        int corner_points_num = 2;       ///< 每部分中强角点数量
        int surf_points_num = 4;         ///< 每部分中强平面点数量
        int less_corner_points_num = 20; ///< 每部分中弱角点数量
        double min_distance_th2 = 0.05;  ///< 特征提取的最小距离阈值
        double curvature_th = 0.1;       ///< 粗略区分角点和平面点的曲率阈值
        double down_size_radius = 0.2;   ///< 体素滤波的滤波半径
    };

    FeatureExtractor(Options options = Options())
        : options_(std::move(options)) {
        voxel_filter_.setLeafSize(options_.down_size_radius, options_.down_size_radius, options_.down_size_radius);
        corner_points_ = pcl::make_shared<PointCloudXYZTL>();
        surf_points_ = pcl::make_shared<PointCloudXYZTL>();
        less_surf_points_ = pcl::make_shared<PointCloudXYZTL>();
        less_corner_points_ = pcl::make_shared<PointCloudXYZTL>();
    }

    /// 设置待特征提取的目标scan
    void SetTarget(LaserScans laser_scans) { laser_scans_ = std::move(laser_scans); }

    /// 开始提取特征
    void Extract();

private:
    /// 计算某个scan的曲率信息
    std::vector<double> ComputeCurvature(const PointCloudXYZTL::ConstPtr &scan);

    /// 对弱平面点进行降采样
    void DownSizeLessSurf();

    Options options_;                         ///< 特征提取的配置信息
    LaserScans laser_scans_;                  ///< 带有线束信息的雷达扫描数据
    pcl::VoxelGrid<PointXYZTL> voxel_filter_; ///< 弱平面点的降采样体素滤波器

public:
    PointCloudXYZTL::Ptr corner_points_;      ///< 强角点集合
    PointCloudXYZTL::Ptr less_corner_points_; ///< 弱角点集合（包含强角点）
    PointCloudXYZTL::Ptr surf_points_;        ///< 强平面点集合
    PointCloudXYZTL::Ptr less_surf_points_;   ///< 弱平面点集合（包含强平面点）
};

NAMESPACE_END