#include <memory>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#define NAMESPACE_BEGIN namespace A_LOAM_MT {
#define NAMESPACE_END }

#define deg2rad (M_PI) / (180.0);
#define rad2deg (180.0) / (M_PI);

NAMESPACE_BEGIN
struct PointXYZTL;

typedef Eigen::Matrix<double, 3, 4> Mat3_4;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Vector3d Vec3;

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointXYZTL> PointCloudXYZTL;

struct PointXYZTL {
    PCL_ADD_POINT4D

    std::uint8_t ring = 0; ///< 线束信息
    double time = 0;       ///< 时间offset

    PointXYZTL() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

NAMESPACE_END

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(A_LOAM_MT::PointXYZTL,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (std::uint8_t, ring, ring)
                                 (double, time, time))

// clang-format on