#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include "Common.hpp"
#include "Frame.h"

NAMESPACE_BEGIN

/// SE3顶点
class SE3Vertex : public g2o::BaseVertex<6, SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &) override { return false; }

    bool write(std::ostream &) const override { return false; }

    /// 这里需要定义v向量中代表的含义，这里使用R|t顺序
    void oplusImpl(const double *v) override;

    void setToOriginImpl() override { _estimate = SE3(); }
};

/// 点到线参数icp对应边
class LineEdge : public g2o::BaseUnaryEdge<3, LineParam, SE3Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LineEdge(Vec3 point)
        : point_(std::move(point))
        , vTlc_(nullptr) {}

    bool read(std::istream &) override { return false; }

    bool write(std::ostream &) const override { return false; }

    void computeError() override;

    void linearizeOplus() override;

private:
    Vec3 point_;           ///< curr_frame中的点
    SE3Vertex *vTlc_;      ///< 参与的顶点信息
    LineParam line_param_; ///< 线参数
    Mat3 hat_d_;           ///< 线参数方向的反对称矩阵
};

/// 点到面参数icp对应边
class PlaneEdge : public g2o::BaseUnaryEdge<1, PlaneParam, SE3Vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneEdge(Vec3 point)
        : point_(std::move(point))
        , vTlc_(nullptr) {}

    bool read(std::istream &) override { return false; }

    bool write(std::ostream &) const override { return false; }

    void computeError() override;

    void linearizeOplus() override;

private:
    Vec3 point_;                     ///< curr_frame中的点
    SE3Vertex *vTlc_;                ///< 参与优化的顶点信息
    Eigen::Matrix<double, 1, 3> nT_; ///< n的转置
};

NAMESPACE_END
