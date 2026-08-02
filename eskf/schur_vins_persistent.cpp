/**
 * @file schur_vins_persistent.cpp
 * @brief 影子候选池、持久 Landmark 晋升与联合 EKF 更新。
 *
 * 导航/clone 状态固定占据协方差前 COV_SIZE 维，持久点按 3 维世界坐标依次
 * 追加在后部。普通轨迹仍由 MSCKF Schur 消元；只有跨两次独立轨迹均表现稳定
 * 的影子候选才晋升，因此联合状态规模始终受预算限制。
 */

#include "schur_vins.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace slam;

bool SchurVINS::isPersistentLandmark(const LandmarkID id) const {
    return persistent_landmark_indices_.find(id) !=
           persistent_landmark_indices_.end();
}

size_t SchurVINS::persistentLandmarkOffset(const size_t index) const {
    return COV_SIZE + LMK_SIZE * index;
}

SchurVINS::TrackGeometryQuality
SchurVINS::evaluateTrackGeometry(const Landmark &landmark) const {
    // 几何评分不是概率权重，而是调度启发式。五个分量分别鼓励：更大视差、
    // 更多独立观测、更好的条件数、更小重投影误差和更小位置不确定度。
    // 分数只决定候选/晋升，不直接乘到 H 或 R，避免破坏量测噪声的统计含义。
    TrackGeometryQuality quality{};
    quality.max_parallax_deg = landmark.geometry_max_parallax_deg;
    quality.condition_number = landmark.geometry_condition_number;
    quality.reprojection_rmse = landmark.geometry_reprojection_rmse;
    quality.position_std = landmark.geometry_position_std;
    quality.observation_count = landmark.geometry_observation_count;

    if (!landmark.is_triangulated || quality.observation_count < 2 ||
        !std::isfinite(quality.max_parallax_deg) ||
        !std::isfinite(quality.condition_number) ||
        !std::isfinite(quality.reprojection_rmse) ||
        !std::isfinite(quality.position_std)) {
        return quality;
    }

    const TYPE parallax_span = std::max(TYPE(8), triangulation_min_parallax_deg);
    const TYPE parallax_score = std::clamp(
        (quality.max_parallax_deg - triangulation_min_parallax_deg) /
            parallax_span,
        TYPE(0), TYPE(1));
    const TYPE observation_score = std::clamp(
        (static_cast<TYPE>(quality.observation_count) - TYPE(2)) / TYPE(8),
        TYPE(0), TYPE(1));
    const TYPE condition_log = std::max(
        TYPE(0), std::log10(std::max(quality.condition_number, TYPE(1))));
    const TYPE condition_score = TYPE(1) /
        (TYPE(1) + condition_log / TYPE(4));
    const TYPE reprojection_scale = std::max(
        TYPE(3) * triangulation_uv_std, TYPE(1e-8));
    const TYPE reprojection_score =
        std::exp(-quality.reprojection_rmse / reprojection_scale);
    const TYPE uncertainty_score = TYPE(1) /
        (TYPE(1) + quality.position_std);

    quality.score = TYPE(0.30) * parallax_score +
                    TYPE(0.20) * observation_score +
                    TYPE(0.15) * condition_score +
                    TYPE(0.20) * reprojection_score +
                    TYPE(0.15) * uncertainty_score;
    quality.valid = true;
    quality.promotable =
        quality.observation_count >= 4 &&
        quality.max_parallax_deg >= triangulation_min_parallax_deg + TYPE(0.5) &&
        quality.condition_number <= TYPE(1e6) &&
        quality.reprojection_rmse <= triangulation_max_reprojection_rmse &&
        quality.position_std <= persistent_max_position_std_ &&
        quality.score >= persistent_min_geometry_score_;
    return quality;
}

bool SchurVINS::shouldDeferTrackConsumption(
    const Landmark &landmark,
    const TriangulationStatus status,
    const bool lost) const {
    if (!enable_hybrid_persistent_landmarks_ || lost ||
        landmark.deferred_consumption_count >= deferred_track_max_frames_) {
        return false;
    }
    return status == TriangulationStatus::LowParallax ||
           status == TriangulationStatus::IllConditioned ||
           status == TriangulationStatus::ExcessiveUncertainty;
}

void SchurVINS::updateShadowCandidate(
    const Landmark &landmark,
    const TrackGeometryQuality &quality,
    const Tus timestamp) {
    updateShadowCandidateEstimate(
        landmark.id, landmark.position, landmark.cov_position,
        quality, timestamp);
}

void SchurVINS::updateShadowCandidateEstimate(
    const LandmarkID id,
    const Vec3 &position,
    const Mat3_3 &covariance,
    const TrackGeometryQuality &quality,
    const Tus timestamp) {
    if (!enable_hybrid_persistent_landmarks_ || !quality.valid ||
        isPersistentLandmark(id) || !position.allFinite() ||
        !covariance.allFinite()) {
        return;
    }

    auto candidate_it = shadow_candidates_.find(id);
    if (candidate_it == shadow_candidates_.end()) {
        if (shadow_candidates_.size() >= shadow_candidate_capacity_) {
            const auto weakest = std::min_element(
                shadow_candidates_.begin(), shadow_candidates_.end(),
                [](const auto &lhs, const auto &rhs) {
                    if (lhs.second.quality_ema == rhs.second.quality_ema) {
                        return lhs.second.last_seen < rhs.second.last_seen;
                    }
                    return lhs.second.quality_ema < rhs.second.quality_ema;
                });
            if (weakest != shadow_candidates_.end()) {
                shadow_candidates_.erase(weakest);
            }
        }
        ShadowCandidateState candidate{};
        candidate.id = id;
        candidate.position = position;
        candidate.covariance = covariance;
        candidate.quality_ema = quality.score;
        candidate.stable_updates = quality.promotable ? 1 : 0;
        candidate.observation_count = quality.observation_count;
        candidate.last_seen = timestamp;
        shadow_candidates_.emplace(id, candidate);
        ++n_shadow_candidate_updates_;
        return;
    }

    auto &candidate = candidate_it->second;
    // 把一次独立轨迹三角化看成候选点的 3D 伪量测：
    //   r=p_m-p_c, S=P_c+P_m, NIS=r^T S^-1 r。
    // 候选与导航完全解耦，因此这里的 EKF 只用于稳定性筛选；它的 P_c 不会
    // 被复制为正式 SLAM 点协方差。
    Mat3_3 innovation_covariance = candidate.covariance + covariance;
    innovation_covariance = TYPE(0.5) *
        (innovation_covariance + innovation_covariance.transpose());
    Eigen::LDLT<Mat3_3> ldlt(innovation_covariance);
    const Vec3 residual = position - candidate.position;
    TYPE nis = std::numeric_limits<TYPE>::infinity();
    if (ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > TYPE(0)) {
        nis = residual.dot(ldlt.solve(residual));
    }

    const TYPE alpha = std::clamp(
        shadow_candidate_ema_alpha_, TYPE(0), TYPE(1));
    candidate.quality_ema =
        (TYPE(1) - alpha) * candidate.quality_ema + alpha * quality.score;
    candidate.consistency_nis_ema = std::isfinite(nis)
        ? (TYPE(1) - alpha) * candidate.consistency_nis_ema + alpha * nis
        : candidate.consistency_nis_ema;
    candidate.observation_count += quality.observation_count;
    candidate.last_seen = timestamp;

    if (!std::isfinite(nis) || nis > shadow_candidate_nis_threshold_) {
        candidate.position = position;
        candidate.covariance = covariance;
        candidate.stable_updates = quality.promotable ? 1 : 0;
        ++n_shadow_candidate_rejections_;
        return;
    }

    const Mat3_3 gain = candidate.covariance * ldlt.solve(Mat3_3::Identity());
    candidate.position.noalias() += gain * residual;
    const Mat3_3 identity_minus_gain = Mat3_3::Identity() - gain;
    candidate.covariance =
        identity_minus_gain * candidate.covariance *
            identity_minus_gain.transpose() +
        gain * covariance * gain.transpose();
    candidate.covariance = TYPE(0.5) *
        (candidate.covariance + candidate.covariance.transpose());
    candidate.stable_updates = quality.promotable
        ? candidate.stable_updates + 1
        : 0;
    ++n_shadow_candidate_updates_;
}

bool SchurVINS::shadowCandidateReady(
    const LandmarkID id,
    const TrackGeometryQuality &quality) const {
    if (!enable_hybrid_persistent_landmarks_ || !quality.promotable ||
        persistent_landmarks_.size() >= persistent_landmark_budget_) {
        return false;
    }
    const auto candidate = shadow_candidates_.find(id);
    return candidate != shadow_candidates_.end() &&
           candidate->second.stable_updates + (quality.promotable ? 1 : 0) >=
               shadow_candidate_min_stable_updates_ &&
           candidate->second.quality_ema >= persistent_min_geometry_score_ &&
           candidate->second.consistency_nis_ema <=
               shadow_candidate_nis_threshold_;
}

void SchurVINS::applyJointStateCorrection(const VecX &dx) {
    if (dx.size() < static_cast<Eigen::Index>(COV_SIZE)) {
        throw std::invalid_argument("joint correction is smaller than navigation state");
    }
    updateState(dx.head(COV_SIZE));
    for (size_t index = 0; index < persistent_landmarks_.size(); ++index) {
        const size_t offset = persistentLandmarkOffset(index);
        if (offset + LMK_SIZE <= static_cast<size_t>(dx.size())) {
            persistent_landmarks_[index].position += dx.segment<3>(offset);
        }
    }
}

bool SchurVINS::promotePersistentLandmark(
    Landmark &landmark,
    const TrackGeometryQuality &quality,
    const Mat3_3 &hll,
    const Eigen::Matrix<TYPE, Eigen::Dynamic, 3> &hpl,
    const Vec3 &gl,
    const Mat3_3 &parameter_to_world,
    const VecX &navigation_increment,
    const TYPE visual_variance,
    const Tus timestamp) {
    if (!shadowCandidateReady(landmark.id, quality) ||
        isPersistentLandmark(landmark.id) ||
        hpl.rows() != static_cast<Eigen::Index>(COV_SIZE)) {
        return false;
    }

    Eigen::SelfAdjointEigenSolver<Mat3_3> es(
        TYPE(0.5) * (hll + hll.transpose()));
    if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
        return false;
    }
    const TYPE maximum = es.eigenvalues().maxCoeff();
    const TYPE threshold = hll_rank_relative_threshold_ * maximum;
    if (!(maximum > TYPE(0)) || es.eigenvalues().minCoeff() <= threshold) {
        return false;
    }
    const Mat3_3 hll_inverse = es.eigenvectors() *
        es.eigenvalues().cwiseInverse().asDiagonal() *
        es.eigenvectors().transpose();

    // 联合正规方程回代：
    //   delta_l=Hll^-1(gl-Hlx delta_x)。
    // parameter_to_world=T 把局部参数增量映射为世界 XYZ 增量。
    const Vec3 parameter_increment = hll_inverse *
        (gl - hpl.transpose() * navigation_increment);
    const Vec3 world_increment = parameter_to_world * parameter_increment;
    if (!world_increment.allFinite() || world_increment.norm() > TYPE(20)) {
        return false;
    }

    // 延迟初始化把新点写成导航误差的线性函数：
    //   delta_l = J_x delta_x + v_l,
    //   J_x=-T Hll^-1 Hlx,
    //   Cov(v_l)=T(R Hll^-1)T^T。
    // 因而 P_lx=J_x P_xx，P_ll=J_x P_xx J_x^T+Cov(v_l)。
    const Eigen::Matrix<TYPE, 3, Eigen::Dynamic> state_jacobian =
        -parameter_to_world * hll_inverse * hpl.transpose();
    Mat3_3 conditional_covariance = parameter_to_world *
        (visual_variance * hll_inverse) * parameter_to_world.transpose();
    conditional_covariance = TYPE(0.5) *
        (conditional_covariance + conditional_covariance.transpose());

    const size_t previous_size = static_cast<size_t>(cov_.rows());
    const auto navigation_covariance =
        cov_.topLeftCorner(COV_SIZE, COV_SIZE);
    // cov_.topRows(COV_SIZE) 包含导航到“现有全部联合状态”的协方差，
    // 所以新点不仅获得 P_lx，也会通过公共导航状态获得与旧持久点的 P_l,new-old。
    const Eigen::Matrix<TYPE, 3, Eigen::Dynamic> cross_covariance =
        state_jacobian * cov_.topRows(COV_SIZE);
    Mat3_3 landmark_covariance =
        state_jacobian * navigation_covariance * state_jacobian.transpose() +
        conditional_covariance;
    landmark_covariance = TYPE(0.5) *
        (landmark_covariance + landmark_covariance.transpose());
    if (!landmark_covariance.allFinite() ||
        landmark_covariance.diagonal().minCoeff() <= TYPE(0)) {
        return false;
    }

    MatXX augmented = MatXX::Zero(previous_size + LMK_SIZE,
                                  previous_size + LMK_SIZE);
    augmented.topLeftCorner(previous_size, previous_size) = cov_;
    augmented.bottomLeftCorner(LMK_SIZE, previous_size) = cross_covariance;
    augmented.topRightCorner(previous_size, LMK_SIZE) =
        cross_covariance.transpose();
    augmented.bottomRightCorner<LMK_SIZE, LMK_SIZE>() = landmark_covariance;
    cov_.swap(augmented);

    PersistentLandmarkState persistent{};
    persistent.id = landmark.id;
    persistent.position = landmark.position + world_increment;
    persistent.position_fej = persistent.position;
    persistent.promotion_quality = quality.score;
    persistent.last_seen = timestamp;
    const size_t index = persistent_landmarks_.size();
    persistent_landmarks_.push_back(persistent);
    persistent_landmark_indices_[persistent.id] = index;
    shadow_candidates_.erase(persistent.id);
    ++n_persistent_landmarks_promoted_;
    return true;
}

size_t SchurVINS::updatePersistentLandmarks(
    const CameraData &cam_data,
    Frame *current_frame,
    const bool is_keyframe) {
    if (!enable_hybrid_persistent_landmarks_ || !is_keyframe ||
        !current_frame || persistent_landmarks_.empty()) {
        return 0;
    }

    const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
    const TYPE image_variance = std::max(
        triangulation_uv_std * triangulation_uv_std *
            std::max(persistent_measurement_noise_scale_, TYPE(1)),
        TYPE(1e-12));
    const TYPE huber_delta = std::max(
        visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
    size_t accepted = 0;

    for (const auto &[id, measurement] : cam_data.measurements) {
        const auto index_it = persistent_landmark_indices_.find(id);
        if (index_it == persistent_landmark_indices_.end()) {
            continue;
        }
        auto &landmark = persistent_landmarks_[index_it->second];
        const Mat3_3 Rwi = current_frame->q().toRotationMatrix();
        const Vec3 d_world = landmark.position - current_frame->p();
        const Vec3 d_camera = Ric.transpose() *
            (Rwi.transpose() * d_world - ext_.t_ic);
        if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
            ++landmark.rejected_count;
            ++n_persistent_landmark_rejections_;
            continue;
        }

        const TYPE inv_depth = TYPE(1) / d_camera.z();
        const Vec2 estimate = d_camera.head<2>() * inv_depth;
        const Vec2 residual = measurement - estimate;
        if (!residual.allFinite() ||
            residual.norm() > visual_hard_reprojection_limit) {
            ++landmark.rejected_count;
            ++n_persistent_landmark_rejections_;
            continue;
        }

        const Mat3_3 Rwi_fej = current_frame->q_fej().toRotationMatrix();
        const Vec3 d_world_fej =
            landmark.position_fej - current_frame->p_fej();
        const Vec3 d_camera_fej = Ric.transpose() *
            (Rwi_fej.transpose() * d_world_fej - ext_.t_ic);
        if (!d_camera_fej.allFinite() || d_camera_fej.z() <= TYPE(0.05)) {
            ++landmark.rejected_count;
            ++n_persistent_landmark_rejections_;
            continue;
        }

        const TYPE inv_depth_fej = TYPE(1) / d_camera_fej.z();
        const TYPE inv_depth2_fej = inv_depth_fej * inv_depth_fej;
        Mat2_3 projection_jacobian;
        projection_jacobian <<
            inv_depth_fej, TYPE(0), -d_camera_fej.x() * inv_depth2_fej,
            TYPE(0), inv_depth_fej, -d_camera_fej.y() * inv_depth2_fej;
        const Mat2_3 landmark_jacobian = projection_jacobian *
            (Ric.transpose() * Rwi_fej.transpose());
        Mat2_6 pose_jacobian;
        pose_jacobian.leftCols<3>().noalias() =
            landmark_jacobian * hat(d_world_fej);
        pose_jacobian.rightCols<3>().noalias() = -landmark_jacobian;

        const TYPE robust_weight = residual.norm() > huber_delta
            ? huber_delta / residual.norm()
            : TYPE(1);
        const Mat2_2 measurement_covariance = Mat2_2::Identity() *
            (image_variance / std::max(robust_weight, TYPE(1e-6)));
        MatXX measurement_jacobian = MatXX::Zero(2, cov_.cols());
        const size_t frame_offset = INSState::SIZE +
            AugState::SIZE * current_frame->ordering;
        const size_t landmark_offset =
            persistentLandmarkOffset(index_it->second);
        measurement_jacobian.block<2, 6>(0, frame_offset) = pose_jacobian;
        measurement_jacobian.block<2, 3>(0, landmark_offset) =
            landmark_jacobian;

        // 联合量测 H 同时在当前 clone 和该持久点的 3 维块上非零：
        //   S=H P H^T+R, K=P H^T S^-1。
        // 因为 P 保存了 P_xl/P_ll，该更新会以统计一致的方式同时修正导航和点。
        const MatXX covariance_times_jacobian =
            cov_ * measurement_jacobian.transpose();
        Mat2_2 innovation_covariance =
            measurement_jacobian * covariance_times_jacobian +
            measurement_covariance;
        innovation_covariance = TYPE(0.5) *
            (innovation_covariance + innovation_covariance.transpose());
        Eigen::LDLT<Mat2_2> innovation_ldlt(innovation_covariance);
        if (innovation_ldlt.info() != Eigen::Success ||
            innovation_ldlt.vectorD().minCoeff() <= TYPE(0)) {
            ++landmark.rejected_count;
            ++n_persistent_landmark_rejections_;
            continue;
        }
        const TYPE nis = residual.dot(innovation_ldlt.solve(residual));
        if (!std::isfinite(nis) || nis > persistent_update_chi2_threshold_) {
            ++landmark.rejected_count;
            ++n_persistent_landmark_rejections_;
            continue;
        }

        const MatXX gain = covariance_times_jacobian *
            innovation_ldlt.solve(Mat2_2::Identity());
        const VecX correction = gain * residual;
        // Joseph 形式的低秩等价展开：
        //   P+=(I-KH)P(I-KH)^T+KRK^T
        //     =P-K(HP)- (PH^T)K^T + KSK^T。
        cov_.noalias() -= gain * covariance_times_jacobian.transpose();
        cov_.noalias() -= covariance_times_jacobian * gain.transpose();
        cov_.noalias() += gain * innovation_covariance * gain.transpose();
        cov_ = TYPE(0.5) * (cov_ + cov_.transpose());
        applyJointStateCorrection(correction);

        landmark.last_seen = cam_data.timestamp;
        ++landmark.update_count;
        ++n_persistent_landmark_updates_;
        ++accepted;
    }
    return accepted;
}
