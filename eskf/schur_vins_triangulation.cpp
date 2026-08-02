/**
 * @file schur_vins_triangulation.cpp
 * @brief 多视图射线三角化、非线性精化与 Landmark 初始协方差估计。
 *
 * 第 i 个相机中心为 c_i，世界系单位视线为 d_i，世界点 p 到该射线的
 * 正交距离为 ||(I-d_i d_i^T)(p-c_i)||。最小化所有射线距离得到线性系统：
 *   A p = b，
 *   A = sum_i (I-d_i d_i^T)，
 *   b = sum_i (I-d_i d_i^T)c_i。
 * A 的最小特征值反映深度方向是否可观；随后使用带位姿不确定度和 Huber
 * 权重的重投影 Gauss-Newton 精化，并由最终信息矩阵估计点位置协方差。
 */

#include "schur_vins.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>

using namespace slam;

SchurVINS::TriangulationResult
SchurVINS::triangulateLandmark(const Landmark &landmark) const {
    const auto started = std::chrono::steady_clock::now();
    TriangulationResult result;

    auto finish = [&](const TriangulationStatus status) {
        result.status = status;
        result.elapsed_us = static_cast<TYPE>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - started).count()) * TYPE(1e-3);
        return result;
    };

    struct View {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        const Frame *frame{};
        Vec2 measurement{Vec2::Zero()};
        Vec3 camera_center{Vec3::Zero()};
        Vec3 bearing_world{Vec3::Zero()};
        Mat3_3 Rwc{Mat3_3::Identity()};
        Mat6_6 relative_pose_covariance{Mat6_6::Zero()};
    };

    std::vector<View, Eigen::aligned_allocator<View>> views;
    views.reserve(landmark.frm2fet.size());
    const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();

    // 先把数据结构中的观测转换成统一的几何量：世界系相机中心 C_i 和单位视线 d_i。
    // 后续线性初始化、视差判断和非线性重投影都复用它们，避免循环中重复做外参变换。
    for (const auto &[frame_id, feature] : landmark.frm2fet) {
        (void)frame_id;
        if (!feature || !feature->frame || !feature->obs[0]) {
            continue;
        }
        const auto *frame = feature->frame;
        const Vec3 bearing_camera = feature->obs[0]->un_pt.normalized();
        if (!bearing_camera.allFinite() || bearing_camera.z() <= TYPE(0)) {
            continue;
        }

        View view;
        view.frame = frame;
        view.measurement = feature->obs[0]->un_pt.head<2>();
        const Mat3_3 Rwi = frame->q().toRotationMatrix();
        view.Rwc.noalias() = Rwi * Ric;
        view.camera_center.noalias() = frame->p() + Rwi * ext_.t_ic;
        view.bearing_world.noalias() = view.Rwc * bearing_camera;
        view.bearing_world.normalize();
        views.emplace_back(view);
    }

    result.observation_count = views.size();
    if (views.size() < 2) {
        return finish(TriangulationStatus::InsufficientViews);
    }

    // 必须按时间戳排序。紧凑滑窗会复用协方差中的物理槽位 ordering，
    // 因而 ordering 不能代表观测先后顺序。
    std::sort(views.begin(), views.end(), [](const View &lhs, const View &rhs) {
        return lhs.frame->timestamp < rhs.frame->timestamp;
    });

    // 两条世界系单位视线的夹角 theta_ij=acos(d_i^T d_j) 决定深度条件数。
    // 使用所有观测对的最大夹角，而不是只比较首末帧，可兼容轨迹回头或观测中断。
    TYPE max_parallax = TYPE(0);
    size_t best_view_i = 0;
    size_t best_view_j = 1;
    for (size_t i = 0; i + 1 < views.size(); ++i) {
        for (size_t j = i + 1; j < views.size(); ++j) {
            const TYPE cosine = std::clamp(
                views[i].bearing_world.dot(views[j].bearing_world), TYPE(-1), TYPE(1));
            const TYPE parallax = std::acos(cosine);
            if (parallax > max_parallax) {
                max_parallax = parallax;
                best_view_i = i;
                best_view_j = j;
            }
        }
    }
    result.max_parallax_deg = max_parallax * TYPE(180) / std::numbers::pi_v<TYPE>;
    if (result.max_parallax_deg < triangulation_min_parallax_deg) {
        return finish(TriangulationStatus::LowParallax);
    }

    // 最大视差观测对对深度最敏感。在这两个候选中选择位姿协方差 trace 更小的
    // clone 作锚点；锚点仅用于不确定度传播，不会被当成额外量测。
    auto poseCovarianceTrace = [&](const size_t view_index) {
        const size_t state_index = INSState::SIZE +
            AugState::SIZE * views[view_index].frame->ordering;
        return cov_.block<6, 6>(state_index, state_index).trace();
    };
    const size_t anchor_view = poseCovarianceTrace(best_view_i)
        <= poseCovarianceTrace(best_view_j) ? best_view_i : best_view_j;
    const size_t anchor_index = INSState::SIZE +
        AugState::SIZE * views[anchor_view].frame->ordering;
    const Mat6_6 anchor_covariance = cov_.block<6, 6>(anchor_index, anchor_index);

    // 使用 clone 间交叉块构造相对协方差：
    // P(δx_i-δx_a)=P_ii+P_aa-P_ia-P_ai，可抵消共同模态。
    for (auto &view : views) {
        const size_t pose_index = INSState::SIZE +
                                  AugState::SIZE * view.frame->ordering;
        view.relative_pose_covariance =
            cov_.block<6, 6>(pose_index, pose_index) + anchor_covariance
            - cov_.block<6, 6>(pose_index, anchor_index)
            - cov_.block<6, 6>(anchor_index, pose_index);
        view.relative_pose_covariance = TYPE(0.5) *
            (view.relative_pose_covariance + view.relative_pose_covariance.transpose());
    }

    // 射线最小二乘初始化：点 P 到射线 (C_i,d_i) 的垂直残差为
    //   e_i=(I-d_i d_i^T)(P-C_i)。
    // 最小化 sum ||e_i||^2 后的正规方程为
    //   sum(I-d_i d_i^T) P = sum(I-d_i d_i^T) C_i。
    // 当所有视线近似平行时，沿视线方向的特征值趋近 0，因此求解前必须检查条件数。
    Mat3_3 ray_hessian = Mat3_3::Zero();
    Vec3 ray_gradient = Vec3::Zero();
    for (const auto &view : views) {
        const Mat3_3 normal_projector =
            Mat3_3::Identity() - view.bearing_world * view.bearing_world.transpose();
        ray_hessian.noalias() += normal_projector;
        ray_gradient.noalias() += normal_projector * view.camera_center;
    }
    ray_hessian = TYPE(0.5) * (ray_hessian + ray_hessian.transpose());

    Eigen::SelfAdjointEigenSolver<Mat3_3> ray_es(ray_hessian);
    if (ray_es.info() != Eigen::Success || !ray_es.eigenvalues().allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }
    const TYPE ray_min = ray_es.eigenvalues().minCoeff();
    const TYPE ray_max = ray_es.eigenvalues().maxCoeff();
    if (!(ray_max > TYPE(0)) || ray_min <= ray_max * TYPE(1e-8)) {
        return finish(TriangulationStatus::IllConditioned);
    }
    result.condition_number = ray_max / ray_min;
    Vec3 position = ray_es.eigenvectors()
                  * ray_es.eigenvalues().cwiseInverse().asDiagonal()
                  * ray_es.eigenvectors().transpose() * ray_gradient;
    if (!position.allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }

    const TYPE image_variance = std::max(
        triangulation_uv_std * triangulation_uv_std, TYPE(1e-12));

    // 构造带相对位姿不确定度的重投影信息。不构造所有 clone 的稠密联合残差
    // 协方差，以控制每个 landmark 的初始化开销；所得是相对锚点条件下的工程近似。
    auto accumulateReprojection = [&](const Vec3 &point,
                                      Mat3_3 &information,
                                      Vec3 &gradient,
                                      TYPE &squared_error,
                                      TYPE &normalized_error) {
        information.setZero();
        gradient.setZero();
        squared_error = TYPE(0);
        normalized_error = TYPE(0);

        for (const auto &view : views) {
            const Vec3 d_camera = view.Rwc.transpose() * (point - view.camera_center);
            if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
                return false;
            }

            const TYPE inv_depth = TYPE(1) / d_camera.z();
            const TYPE inv_depth2 = inv_depth * inv_depth;
            const Vec2 estimate = d_camera.head<2>() * inv_depth;
            const Vec2 residual = view.measurement - estimate;
            if (!residual.allFinite()) {
                return false;
            }

            // 归一化针孔投影 pi([x,y,z])=[x/z,y/z] 的雅可比；随后分别链式得到
            // 对世界点和 clone 位姿误差 [delta_theta,delta_p] 的雅可比。
            Mat2_3 J_projection;
            J_projection << inv_depth, TYPE(0), -d_camera.x() * inv_depth2,
                            TYPE(0), inv_depth, -d_camera.y() * inv_depth2;
            const Mat2_3 J_landmark = J_projection * view.Rwc.transpose();

            const Vec3 d_world = point - view.frame->p();
            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_landmark * hat(d_world);
            J_pose.rightCols<3>().noalias() = -J_landmark;

            // S_i = sigma_uv^2 I + J_pose P_rel J_pose^T。
            // 这里用相对 clone 协方差，避免把所有帧共有的全局 gauge 不确定度反复计入。
            Mat2_2 residual_covariance =
                J_pose * view.relative_pose_covariance * J_pose.transpose();
            residual_covariance = TYPE(0.5) *
                                  (residual_covariance + residual_covariance.transpose());
            residual_covariance.diagonal().array() += image_variance;
            if (!residual_covariance.allFinite()) {
                return false;
            }

            // 防止协方差仅因浮点舍入出现极小负特征值；真实的大负值会在后续条件检查失败。
            const TYPE trace = residual_covariance.trace();
            const TYPE discriminant = std::sqrt(std::max(
                TYPE(0),
                (residual_covariance(0, 0) - residual_covariance(1, 1)) *
                (residual_covariance(0, 0) - residual_covariance(1, 1)) +
                TYPE(4) * residual_covariance(0, 1) * residual_covariance(0, 1)));
            const TYPE min_eigenvalue = TYPE(0.5) * (trace - discriminant);
            if (min_eigenvalue < image_variance * TYPE(0.1)) {
                residual_covariance.diagonal().array() +=
                    image_variance * TYPE(0.1) - min_eigenvalue;
            }

            const TYPE determinant = residual_covariance.determinant();
            if (!(determinant > image_variance * image_variance * TYPE(1e-6)) ||
                !std::isfinite(determinant)) {
                return false;
            }
            const Mat2_2 weight_matrix = residual_covariance.inverse();
            // 在白化残差上使用 3-sigma Huber。正常观测保持二次代价；异常观测的
            // 影响随 1/||r|| 衰减，但仍保留方向信息，最终再由重投影硬门限判成失败。
            const TYPE mahalanobis2 = std::max(TYPE(0), residual.dot(weight_matrix * residual));
            const TYPE mahalanobis = std::sqrt(mahalanobis2);
            const TYPE huber_weight = mahalanobis > TYPE(3) ? TYPE(3) / mahalanobis : TYPE(1);

            information.noalias() +=
                J_landmark.transpose() * (huber_weight * weight_matrix) * J_landmark;
            gradient.noalias() +=
                J_landmark.transpose() * (huber_weight * weight_matrix) * residual;
            squared_error += residual.squaredNorm();
            normalized_error += mahalanobis2;
        }
        information = TYPE(0.5) * (information + information.transpose());
        return information.allFinite() && gradient.allFinite();
    };

    Mat3_3 information;
    Vec3 gradient;
    TYPE squared_error = TYPE(0);
    TYPE normalized_error = TYPE(0);
    // 以射线解为初值做最多 5 次 Gauss-Newton。当前 residual=measurement-estimate，
    // 且 J_landmark=d(estimate)/dP，所以正规方程是 H*delta=J^T W residual，更新 P+=delta。
    for (size_t iteration = 0; iteration < 5; ++iteration) {
        if (!accumulateReprojection(position, information, gradient,
                                    squared_error, normalized_error)) {
            return finish(TriangulationStatus::NegativeDepth);
        }
        Eigen::LDLT<Mat3_3> ldlt(information);
        if (ldlt.info() != Eigen::Success ||
            ldlt.vectorD().minCoeff() <= ldlt.vectorD().maxCoeff() * TYPE(1e-10)) {
            return finish(TriangulationStatus::IllConditioned);
        }
        const Vec3 increment = ldlt.solve(gradient);
        if (!increment.allFinite() || increment.norm() > TYPE(100)) {
            return finish(TriangulationStatus::IllConditioned);
        }
        position += increment;
        if (increment.norm() < TYPE(1e-6)) {
            break;
        }
    }

    if (!accumulateReprojection(position, information, gradient,
                                squared_error, normalized_error)) {
        return finish(TriangulationStatus::NegativeDepth);
    }
    result.reprojection_rmse = std::sqrt(
        squared_error / static_cast<TYPE>(TYPE(2) * views.size()));
    if (result.reprojection_rmse > triangulation_max_reprojection_rmse) {
        return finish(TriangulationStatus::HighReprojectionError);
    }

    Eigen::SelfAdjointEigenSolver<Mat3_3> information_es(information);
    if (information_es.info() != Eigen::Success ||
        !information_es.eigenvalues().allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }
    const TYPE information_min = information_es.eigenvalues().minCoeff();
    const TYPE information_max = information_es.eigenvalues().maxCoeff();
    if (!(information_min > TYPE(0)) || information_min <= information_max * TYPE(1e-12)) {
        return finish(TriangulationStatus::IllConditioned);
    }
    result.condition_number = information_max / information_min;

    // 条件信息矩阵给出 P_l|poses = Lambda^-1；若白化残差的 chi^2/dof>1，
    // 再按该比例膨胀协方差，防止模型失配时给出虚假的高精度初值。
    const TYPE degrees_of_freedom = std::max<TYPE>(
        TYPE(1), TYPE(2) * static_cast<TYPE>(views.size()) - TYPE(3));
    const TYPE covariance_scale = std::max(TYPE(1), normalized_error / degrees_of_freedom);
    Vec3 covariance_eigenvalues =
        covariance_scale * information_es.eigenvalues().cwiseInverse();
    covariance_eigenvalues = covariance_eigenvalues.cwiseMax(TYPE(1e-10));
    if (std::sqrt(covariance_eigenvalues.maxCoeff()) > triangulation_max_position_std) {
        return finish(TriangulationStatus::ExcessiveUncertainty);
    }

    result.position = position;
    const Mat3_3 conditional_covariance = information_es.eigenvectors()
                                          * covariance_eigenvalues.asDiagonal()
                                          * information_es.eigenvectors().transpose();

    // position 存在世界坐标系中，因此在相对锚点条件协方差之外，还要传播锚点本身的
    // 绝对位姿不确定度。质量门限在上面只检查 conditional_covariance，避免全局 gauge
    // 不确定度把几何上可靠的点拒绝掉。
    Mat3_6 J_anchor_world;
    J_anchor_world.leftCols<3>() = -hat(position - views[anchor_view].frame->p());
    J_anchor_world.rightCols<3>().setIdentity();
    result.covariance = conditional_covariance
                      + J_anchor_world * anchor_covariance * J_anchor_world.transpose();
    result.covariance = TYPE(0.5) * (result.covariance + result.covariance.transpose());
    return finish(TriangulationStatus::Success);
}

void SchurVINS::logTriangulationAttempt(
        Landmark &landmark,
        const TriangulationResult &result,
        const Tus timestamp,
        const std::unordered_map<size_t, Vec3> &ground_truth) {
    if (!enable_logging_) {
        return;
    }

    TriangulationLog log;
    log.timestamp = timestamp;
    log.id = landmark.id;
    log.status = result.status;
    log.observation_count = result.observation_count;
    log.max_parallax_deg = result.max_parallax_deg;
    log.condition_number = result.condition_number;
    log.reprojection_rmse = result.reprojection_rmse;
    log.elapsed_us = result.elapsed_us;
    if (result.status == TriangulationStatus::Success) {
        log.initial_position = result.position;
        log.latest_position = result.position;
        log.initial_covariance = result.covariance;
        log.latest_covariance = result.covariance;
        log.initial_cov_trace = result.covariance.trace();
    }
    if (const auto gt = ground_truth.find(landmark.id); gt != ground_truth.end()) {
        log.ground_truth = gt->second;
        log.has_ground_truth = true;
        if (result.status == TriangulationStatus::Success) {
            const Vec3 error = result.position - gt->second;
            Eigen::LDLT<Mat3_3> ldlt(result.covariance);
            if (ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > TYPE(0)) {
                log.initial_nees = std::max(TYPE(0), error.dot(ldlt.solve(error)));
            }
        }
    }
    triangulation_logs_.emplace_back(log);
    if (result.status == TriangulationStatus::Success) {
        landmark.triangulation_log_index = triangulation_logs_.size() - 1;
    }
}

void SchurVINS::recordLandmarkRefinement(const Landmark &landmark, bool shadow) {
    if (!enable_logging_ || landmark.triangulation_log_index >= triangulation_logs_.size()) {
        return;
    }
    auto &log = triangulation_logs_[landmark.triangulation_log_index];
    if (shadow) {
        log.shadow_initialized = landmark.shadow_initialized;
        log.shadow_position = landmark.shadow_position;
        log.shadow_covariance = landmark.shadow_cov_position;
        ++log.shadow_refinement_count;
    } else {
        log.latest_position = landmark.position;
        log.latest_covariance = landmark.cov_position;
        ++log.refinement_count;
    }
}
