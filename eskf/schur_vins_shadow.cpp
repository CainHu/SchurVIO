/**
 * @file schur_vins_shadow.cpp
 * @brief 与导航后验解耦的影子 Landmark 地图后处理器。
 *
 * 影子点不进入 ESKF 状态，也不修改用于构造导航残差的 Landmark::position。
 * 对固定的 clone 位姿先验，单点状态 p_s 使用独立 EKF：
 *   r = z - pi(T_wc^{-1} p_s),
 *   S = H_p P_s H_p^T + R_eff,
 *   K = P_s H_p^T S^{-1},
 *   p_s <- p_s + K r,
 *   P_s <- (I-KH_p)P_s(I-KH_p)^T + K R_eff K^T.
 *
 * R_eff 不只包含像平面噪声，还加入 H_x P_x H_x^T，以近似传播 clone
 * 位姿不确定度。由于没有保存 P_{x,p_s}，该估计只能作为地图质量诊断或输出，
 * 不能反向约束导航状态，否则会把被忽略的相关性重复计算。
 */

#include "schur_vins.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

using namespace slam;

void SchurVINS::updateShadowLandmarks(
    const CameraData &cam_data,
    const std::vector<std::pair<LandmarkID, Landmark *>> &landmarks,
    const bool is_keyframe,
    const double dt) {
    // 一次性 MSCKF 轨迹在消费后不再保留可重复观测，因此默认 MSCKF 后端不会
    // 运行影子点更新。影子地图只用于重复窗口后端的地图输出与一致性诊断。
    if (!enable_shadow_landmark_postprocessor_ || !is_keyframe ||
        schedulerConsumesTracksOnce()) {
        return;
    }
    const Mat3_3 Ric_landmark = ext_.q_ic.toRotationMatrix();
    const TYPE image_variance = triangulation_uv_std * triangulation_uv_std;
    const TYPE huber_delta = std::max(
        visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
    for (const auto &[id, lmk] : landmarks) {
        (void)id;
        Feature *newest_feature = nullptr;
        for (const auto &[frame_id, feature] : lmk->frm2fet) {
            (void)frame_id;
            if (feature && feature->frame && feature->obs[0] &&
                feature->frame->timestamp == cam_data.timestamp) {
                newest_feature = feature;
                break;
            }
        }
        if (!newest_feature) {
            continue;
        }
        if (!lmk->shadow_initialized) {
            // 初值复制自主 Landmark，但从此两者独立演化：主点参与导航残差，
            // 影子点只消费当前关键帧观测，不把修正反馈到 ESKF。
            lmk->shadow_position = lmk->position;
            lmk->shadow_cov_position = lmk->cov_position;
            lmk->shadow_nis_ema = TYPE(1);
            lmk->shadow_initialized = true;
        }

        TYPE inflation_scale = shadow_landmark_adaptive_inflation_
            ? TYPE(0)
            : TYPE(1);
        if (shadow_landmark_adaptive_inflation_) {
            const TYPE excess_nis =
                std::max(TYPE(0), lmk->shadow_nis_ema - TYPE(1));
            inflation_scale = std::clamp(
                landmark_adaptive_inflation_gain_ * excess_nis,
                TYPE(0), landmark_adaptive_inflation_max_scale_);
        }
        lmk->shadow_cov_position.diagonal().array() +=
            landmark_process_noise_density_ * std::max(TYPE(dt), TYPE(0)) *
            inflation_scale;

        const auto *frame = newest_feature->frame;
        const Mat3_3 Rwi = frame->q().toRotationMatrix();
        const Mat3_3 Rwc = Rwi * Ric_landmark;
        const Vec3 camera_center = frame->p() + Rwi * ext_.t_ic;
        const Vec3 d_camera =
            Rwc.transpose() * (lmk->shadow_position - camera_center);
        if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
            continue;
        }
        const TYPE inv_depth = TYPE(1) / d_camera.z();
        const TYPE inv_depth2 = inv_depth * inv_depth;
        const Vec2 estimate = d_camera.head<2>() * inv_depth;
        const Vec2 residual =
            newest_feature->obs[0]->un_pt.head<2>() - estimate;
        if (!residual.allFinite() ||
            residual.norm() > visual_hard_reprojection_limit) {
            continue;
        }

        Mat2_3 projection_jacobian;
        projection_jacobian <<
            inv_depth, TYPE(0), -d_camera.x() * inv_depth2,
            TYPE(0), inv_depth, -d_camera.y() * inv_depth2;
        const Mat2_3 landmark_jacobian =
            projection_jacobian * Rwc.transpose();
        const TYPE robust_weight = residual.norm() > huber_delta
            ? huber_delta / residual.norm()
            : TYPE(1);
        Mat2_2 measurement_covariance = Mat2_2::Identity() *
            (image_variance / std::max(robust_weight, TYPE(1e-6)));
        // 影子点虽不保存与导航状态的交叉协方差 P_xl，但当前重投影仍依赖 clone 位姿。
        // 因此把 H_x P_x H_x^T 并入等效量测噪声；若漏掉该项，影子点协方差会
        // 虚假地过小，而真实误差实际上由 clone 位姿不确定度主导。
        Mat2_6 pose_jacobian;
        pose_jacobian.leftCols<3>().noalias() =
            landmark_jacobian * hat(lmk->shadow_position - frame->p());
        pose_jacobian.rightCols<3>().noalias() = -landmark_jacobian;
        const size_t frame_offset =
            INSState::SIZE + AugState::SIZE * frame->ordering;
        const Eigen::Matrix<TYPE, 6, 6> frame_covariance =
            cov_.block<6, 6>(frame_offset, frame_offset);
        measurement_covariance.noalias() +=
            pose_jacobian * frame_covariance * pose_jacobian.transpose();
        measurement_covariance = TYPE(0.5) *
            (measurement_covariance + measurement_covariance.transpose());
        Mat2_2 innovation_covariance =
            landmark_jacobian * lmk->shadow_cov_position *
            landmark_jacobian.transpose() + measurement_covariance;
        innovation_covariance = TYPE(0.5) *
            (innovation_covariance + innovation_covariance.transpose());
        Eigen::LDLT<Mat2_2> innovation_ldlt(innovation_covariance);
        if (innovation_ldlt.info() != Eigen::Success ||
            innovation_ldlt.vectorD().minCoeff() <= TYPE(0)) {
            continue;
        }

        TYPE normalized_innovation = std::max(
            TYPE(0), residual.dot(innovation_ldlt.solve(residual)) / TYPE(2));
        if (shadow_landmark_adaptive_inflation_ &&
            normalized_innovation > TYPE(1)) {
            // 当前创新过大时立即膨胀 P_s。只更新 NIS 的指数滑动平均会晚一帧才生效，
            // 无法降低本次异常观测的增益，因此这里再增加即时 fading factor。
            const TYPE fading_factor = std::clamp(
                TYPE(1) + landmark_adaptive_inflation_gain_ *
                    (normalized_innovation - TYPE(1)),
                TYPE(1), landmark_adaptive_inflation_max_scale_);
            lmk->shadow_cov_position *= fading_factor;
            innovation_covariance =
                landmark_jacobian * lmk->shadow_cov_position *
                landmark_jacobian.transpose() + measurement_covariance;
            innovation_covariance = TYPE(0.5) *
                (innovation_covariance + innovation_covariance.transpose());
            innovation_ldlt.compute(innovation_covariance);
            if (innovation_ldlt.info() != Eigen::Success ||
                innovation_ldlt.vectorD().minCoeff() <= TYPE(0)) {
                continue;
            }
            normalized_innovation = std::max(
                TYPE(0), residual.dot(innovation_ldlt.solve(residual)) / TYPE(2));
        }
        const TYPE alpha = std::clamp(
            landmark_nis_ema_alpha_, TYPE(0), TYPE(1));
        lmk->shadow_nis_ema =
            (TYPE(1) - alpha) * lmk->shadow_nis_ema +
            alpha * normalized_innovation;

        // 标准 EKF 增益 K=P_s H_p^T S^{-1}，随后用 Joseph 形式更新 P_s。
        // Joseph 形式在有限精度下更容易保持半正定和对称性。
        const Mat3_2 gain = lmk->shadow_cov_position *
            landmark_jacobian.transpose() *
            innovation_ldlt.solve(Mat2_2::Identity());
        lmk->shadow_position.noalias() += gain * residual;
        const Mat3_3 identity_minus_kh =
            Mat3_3::Identity() - gain * landmark_jacobian;
        lmk->shadow_cov_position =
            identity_minus_kh * lmk->shadow_cov_position *
                identity_minus_kh.transpose() +
            gain * measurement_covariance * gain.transpose();
        lmk->shadow_cov_position = TYPE(0.5) *
            (lmk->shadow_cov_position + lmk->shadow_cov_position.transpose());
        recordLandmarkRefinement(*lmk, true);
    }
}
