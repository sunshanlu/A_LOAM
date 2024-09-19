#include <rclcpp/rclcpp.hpp>

#include "G2oTypes.h"
#include "LidarOdom.h"

NAMESPACE_BEGIN

/**
 * @brief 里程计运行主逻辑函数
 *
 */
void LidarOdom::Run() {
    while (!request_stop_.load() && rclcpp::ok()) {
        auto db = data_loader_->LoadNext();
        curr_frame_ = Frame::Create(db.pointclouds_, db.stamp_, db.right_image_);

        switch (status_) {
        case OdomStatus::NOT_INIT:
            last_frame_ = curr_frame_;
            status_ = OdomStatus::RUNNING;
            break;

        case OdomStatus::RUNNING:
            AlignLastFrame();
            T_odom_curr_ = T_odom_curr_ * Tlc_;
            curr_frame_->SetPoseToc(T_odom_curr_);
            break;
        default:
            throw std::runtime_error("OdomStatus中的未知状态");
            break;
        }
        last_frame_ = curr_frame_;

        /// 输出位姿信息
        std::cout << T_odom_curr_.translation().transpose() << std::endl;
    }
}

/**
 * @brief 优化一个Epoch
 * @details
 *      1. 在运行该函数之前，要保证curr_frame中的关联关系已经建立完成
 *      2. 优化之后，会将优化的结果保存在Tlc_中，以供后续优化操作
 */
void LidarOdom::AlignOneEpoch() {
    /// 构造图优化器和优化算法LM
    Optimizer optimizer;
    optimizer.setVerbose(false);
    auto *lm = new LevenbergMarquardt(std::make_unique<BlockSolver>(std::make_unique<LinearSolver>()));
    optimizer.setAlgorithm(lm);

    /// 创建顶点
    SE3Vertex *vTlc = new SE3Vertex;
    vTlc->setId(0);
    vTlc->setEstimate(Tlc_);
    optimizer.addVertex(vTlc);

    /// 创建点到线icp边
    int line_num = 0;
    auto corner_points = curr_frame_->corner_points_;
    auto line_params = curr_frame_->line_params_;
    for (int i = 0; i < corner_points->size(); ++i) {
        const auto &point = corner_points->points[i];
        const auto &param = line_params[i];
        if (!param.valid)
            continue;

        Vec3 point_eig(point.x, point.y, point.z);
        auto line_edge = new LineEdge(point_eig);
        line_edge->setInformation(Mat3::Identity());
        line_edge->setVertex(0, vTlc);
        line_edge->setMeasurement(param);
        line_edge->setId(i);
        line_edge->computeError();
        double chi2 = line_edge->chi2();
        if (chi2 > options_.icp_distance_th * options_.icp_distance_th) {
            delete line_edge;
            continue;
        }
        optimizer.addEdge(line_edge);
        ++line_num;
    }

    /// 创建点到面icp边
    int plane_num = 0;
    int corner_points_num = corner_points->size();
    auto surf_points = curr_frame_->surf_points_;
    auto plane_params = curr_frame_->plane_params_;
    for (int i = 0; i < surf_points->size(); ++i) {
        const auto &point = surf_points->points[i];
        const auto &param = plane_params[i];
        if (!param.valid)
            continue;

        Vec3 point_eig(point.x, point.y, point.z);
        auto plane_edge = new PlaneEdge(point_eig);
        plane_edge->setId(i + corner_points_num);
        plane_edge->setVertex(0, vTlc);
        plane_edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        plane_edge->setMeasurement(param);
        plane_edge->computeError();
        double chi2 = plane_edge->chi2();
        if (chi2 > options_.icp_distance_th * options_.icp_distance_th) {
            delete plane_edge;
            continue;
        }
        optimizer.addEdge(plane_edge);
        ++plane_num;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(options_.iterations);

    Tlc_ = vTlc->estimate();
}

/**
 * @brief 整个泛化icp过程
 * @details
 *      1. 寻找匹配
 *      2. 优化结果
 *      3. 一直循环往复
 */
void LidarOdom::AlignLastFrame() {
    for (int ep = 0; ep < options_.epoch; ++ep) {
        curr_frame_->FindCorrWithLast(last_frame_, Tlc_);
        AlignOneEpoch();
    }
}

NAMESPACE_END