#pragma once

#include <atomic>

#include "Common.hpp"
#include "DataHelper.h"
#include "Frame.h"

NAMESPACE_BEGIN

class LidarOdom {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<LidarOdom> Ptr;

    enum class OdomStatus { NOT_INIT, RUNNING };

    /// 雷达里程计配置项
    struct Options {
        Options()
            : icp_distance_th(2.5)
            , epoch(5)
            , iterations(10) {}

        double icp_distance_th = 2.5; ///< icp距离阈值
        int epoch = 5;                ///< 优化代数
        int iterations = 10;          ///< 每一代的最大迭代次数
    };

    LidarOdom(Options options = Options())
        : curr_frame_(nullptr)
        , last_frame_(nullptr)
        , request_stop_(false)
        , data_loader_(nullptr)
        , status_(OdomStatus::NOT_INIT)
        , options_(std::move(options)) {}

    /// 里程计线程
    void Run();

    /// 请求线程停止
    void RequestStop() { request_stop_.store(true); }

    /// 设置数据加载器
    void SetDataLoader(DataHelper::Ptr data_loader) { data_loader_ = std::move(data_loader); }

private:
    /// 使用优化的方式，计算Tlc
    void AlignLastFrame();

    /// 优化一个epoch
    void AlignOneEpoch();

    SE3 T_odom_curr_;                       ///< 里程计下的当前帧位姿
    SE3 Tlc_;                               ///< 速度信息，优化的结果Tlc
    Frame::Ptr curr_frame_;                 ///< 当前帧
    Frame::Ptr last_frame_;                 ///< 上一帧
    std::atomic<bool> request_stop_;        ///< 请求停止标志符
    DataHelper::Ptr data_loader_ = nullptr; ///< 数据加载器
    OdomStatus status_;                     ///< 里程计状态
    Options options_;                       ///< 里程计配置项
};

NAMESPACE_END