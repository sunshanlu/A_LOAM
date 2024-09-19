#include "G2oTypes.h"

NAMESPACE_BEGIN

/**
 * @brief SE3Vertex的顶点更新函数
 * @details 定义v中的顺序为
 *      1. 前三个数为平移向量的更新t
 *      2. 后三个数位旋转向量的更新deltaR，使用李代数右扰动模型
 * @param v 输入的更新向量v
 */
void SE3Vertex::oplusImpl(const double *v) {
    Vec3 update_t, update_r;
    update_r << v[0], v[1], v[2];
    update_t << v[3], v[4], v[5];
    _estimate.translation() += update_t;
    _estimate.so3() = _estimate.so3() * SO3::exp(update_r);
}

/**
 * @brief 点到线icp误差边的残差求解函数
 * @details
 *      1. 定义了残差公式
 *      2. 定义了误差_error的顺序
 */
void LineEdge::computeError() {
    LineParam line_param = measurement();
    vTlc_ = dynamic_cast<SE3Vertex *>(_vertices[0]);
    hat_d_ = SO3::hat(line_param.d);
    _error = hat_d_ * (vTlc_->estimate() * point_ - line_param.p0);
}

/**
 * @brief 点到线的icp误差雅可比矩阵求解函数
 * @details
 *      1. 由SE3Vertex定义确定R,t顺序
 *      2. 由computeError确定误差_error的顺序
 *
 */
void LineEdge::linearizeOplus() {
    SE3 Tlc = vTlc_->estimate();
    _jacobianOplusXi.setZero();

    _jacobianOplusXi.block<3, 3>(0, 0) = -hat_d_ * Tlc.so3().matrix() * SO3::hat(point_); /// de / dr
    _jacobianOplusXi.block<3, 3>(0, 3) = hat_d_;                                          ///< de / dt
}

/**
 * @brief 点到面的icp误差计算函数
 * @details
 *      1. 定义了残差公式
 *      2. 定义了误差顺序
 */
void PlaneEdge::computeError() {
    vTlc_ = dynamic_cast<SE3Vertex *>(_vertices[0]);
    nT_ = _measurement.n.transpose();
    _error = nT_ * (vTlc_->estimate() * point_);
    _error(0, 0) += _measurement.d;
}

/**
 * @brief 点到面的icp雅可比矩阵计算函数
 * @details
 *      1. 由SE3Vertex定义确定R,t顺序
 *      2. 由computeError确定误差_error的顺序
 */
void PlaneEdge::linearizeOplus() {
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<1, 3>(0, 0) = -nT_ * vTlc_->estimate().so3().matrix() * SO3::hat(point_); ///< de / dr
    _jacobianOplusXi.block<1, 3>(0, 3) = nT_;                                                        ///< de / dt
}

NAMESPACE_END