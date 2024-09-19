#pragma once

/// 注意flann和opencv的include的顺序，要保证flann在opencv之前

#include <memory>

#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#define NAMESPACE_BEGIN namespace A_LOAM_MT {
#define NAMESPACE_END }

#define deg2rad ((M_PI) / (180.0));
#define rad2deg ((180.0) / (M_PI));

NAMESPACE_BEGIN
struct PointXYZTL;

typedef Eigen::Matrix<double, 3, 4> Mat3_4;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;

typedef Eigen::Quaterniond Quat;

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointXYZTL> PointCloudXYZTL;
typedef pcl::PointXYZ PointXYZ;

typedef g2o::SparseOptimizer Optimizer;
typedef g2o::BlockSolverX BlockSolver;
typedef g2o::LinearSolverEigen<BlockSolver::PoseMatrixType> LinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg LevenbergMarquardt;

/// 计算p1点和p2点之间的距离平方
// clang-format off
template <typename T> 
double Distance2(const T &p1, const T &p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}
// clang-format on

struct PointXYZTL {
    PCL_ADD_POINT4D

    std::uint8_t ring = 0; ///< 线束信息
    double time = 0;       ///< 时间offset

    PointXYZTL() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

NAMESPACE_END

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(A_LOAM_MT::PointXYZTL,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (std::uint8_t, ring, ring)
                                 (double, time, time))

// clang-format on