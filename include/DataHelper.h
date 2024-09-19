#pragma once

#include "Common.hpp"

NAMESPACE_BEGIN

class DataHelper {
public:
    typedef std::shared_ptr<DataHelper> Ptr;
    typedef std::shared_ptr<const DataHelper> ConstPtr;

    /// 数据包
    struct DataBag {
        typedef std::vector<PointCloudXYZTL::Ptr> PointClouds;

        double stamp_;            ///< 时间戳
        PointClouds pointclouds_; ///< 点云数据，分线束存储
        cv::Mat right_image_;     ///< 右侧图像
        SE3 ref_pose_;            ///< 里程计参考位姿
        bool valid_;              ///< 数据是否有效
    };

    DataHelper() = default;

    virtual DataBag LoadNext() = 0;

    virtual ~DataHelper() = default;

private:
};

NAMESPACE_END