#pragma once

#include "Common.hpp"
#include "FeatureExtractor.h"

NAMESPACE_BEGIN

/// 线参数
struct LineParam {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3 d;     ///< 直线向量的参数
    Vec3 p0;    ///< 直线上任意一点
    bool valid; ///< 参数是否有效
};

struct PlaneParam {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3 n;     ///< 平面法向量
    double d;   ///< 平面偏置
    bool valid; ///< 参数是否有效
};

/// 帧类
class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Frame> Ptr;
    typedef std::vector<PointCloudXYZTL::Ptr> LaserScans;

    struct Options {
        typedef std::shared_ptr<Options> Ptr;
        double distance_th = 25;
        int surround_scanlines = 2;
    };

    Frame(const Frame &frame) = delete;

    Frame &operator=(const Frame &frame) = delete;

    /// this作为当前帧，用于寻找和last_frame的关联关系
    void FindCorrWithLast(Frame::Ptr last_frame, const SE3 &Tlc);

    /// Frame的工厂函数
    static Frame::Ptr Create(LaserScans laser_scans, double timestamp, cv::Mat left_image = cv::Mat());

    /// 设置Todom-curr
    void SetPoseToc(SE3 Toc) { T_odom_curr = std::move(Toc); }

    /// 设置Twc，地图下的位姿
    void SetPoseTwc(SE3 Twc) { T_map_curr = std::move(Twc); }

    /// 获取Todom-curr
    SE3 GetPoseToc() { return T_odom_curr; }

    /// 获取Twc，地图下的位姿
    SE3 GetPoseTwc() { return T_map_curr; }

private:
    /// Frame的构造函数
    Frame(Options::Ptr options, FeatureExtractor::Options extract_options, LaserScans laser_scans, double timestamp,
          cv::Mat left_image);

    /// 寻找角点关联
    void FindCornerCorr(Frame::Ptr last_frame, const SE3 &Tlc);

    /// 寻找平面点关联
    void FindSurfCorr(Frame::Ptr last_frame, const SE3 &Tlc);

    SE3 T_odom_curr;                          ///< 里程计下当前帧位姿
    SE3 T_map_curr;                           ///< 地图下当前帧位姿
    PointCloudXYZTL::Ptr less_corner_points_; ///< 弱角点集合（作为上一帧待寻找关联的依据）
    PointCloudXYZTL::Ptr less_surf_points_;   ///< 弱平面点集合（作为上一帧待寻找关联的依据）
    pcl::KdTreeFLANN<PointXYZTL>::Ptr kdtree_corner_; ///< 角点kd-tree
    pcl::KdTreeFLANN<PointXYZTL>::Ptr kdtree_surf_;   ///< 平面点kd-tree

    Options::Ptr options_;        ///< 帧参数选项
    double timestamp_;            ///< 帧的时间戳信息
    cv::Mat left_image_;          ///< 图像信息
    std::uint8_t frame_id_;       ///< 帧id信息
    static std::uint8_t next_id_; ///< 下一帧id

public:
    PointCloudXYZTL::Ptr corner_points_;   ///< 强角点集合（作为当前帧寻找上一帧的依据）
    PointCloudXYZTL::Ptr surf_points_;     ///< 强平面点集合（作为当前帧寻找上一帧的依据）
    std::vector<LineParam> line_params_;   ///< 线参数
    std::vector<PlaneParam> plane_params_; ///< 面参数
};

NAMESPACE_END
