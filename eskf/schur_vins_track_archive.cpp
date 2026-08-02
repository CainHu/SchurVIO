/**
 * @file schur_vins_track_archive.cpp
 * @brief 低视差轨迹摘要归档，以及跨轨迹影子候选的几何一致性检查。
 *
 * 这里故意不把历史 pose 快照写入 ESKF 量测方程。旧 clone 被边缘化后，若直接把
 * 其位姿当成无误差常量，就等价于丢弃 P_x_old 和 P_old,current，会制造虚假信息。
 * 本模块只回答一个较弱的问题：同一外部特征 ID 再次出现后，历史射线与当前射线
 * 是否已经形成可三角化几何。得到的 3D 点只进入影子候选池；正式晋升仍必须使用
 * 当前有效 clone 重新构造 Hpp/Hpl/Hll，并由延迟初始化维护完整 P_xl/P_ll。
 */

#include "schur_vins.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

using namespace slam;

namespace {
    [[nodiscard]] SchurVINS::ArchivedBearingSnapshot makeSnapshot(
        const Feature &feature,
        const Mat3_3 &rotation_imu_camera,
        const Vec3 &translation_imu_camera) {
        SchurVINS::ArchivedBearingSnapshot snapshot;
        const Frame &frame = *feature.frame;
        const Mat3_3 rotation_world_imu = frame.q().toRotationMatrix();
        snapshot.bearing_camera = feature.obs[0]->un_pt.normalized();
        snapshot.rotation_world_camera =
            rotation_world_imu * rotation_imu_camera;
        // 投影模型为 p_c=R_ic^T(R_wi^T(p_w-p_wi)-t_ic)，因此相机中心为
        // c_w=p_wi+R_wi t_ic。
        snapshot.camera_center_world =
            frame.p() + rotation_world_imu * translation_imu_camera;
        snapshot.timestamp = frame.timestamp;
        return snapshot;
    }

    [[nodiscard]] TYPE bearingParallaxDeg(
        const SchurVINS::ArchivedBearingSnapshot &lhs,
        const SchurVINS::ArchivedBearingSnapshot &rhs) {
        const Vec3 lhs_world =
            lhs.rotation_world_camera * lhs.bearing_camera;
        const Vec3 rhs_world =
            rhs.rotation_world_camera * rhs.bearing_camera;
        const TYPE cosine = std::clamp(
            lhs_world.normalized().dot(rhs_world.normalized()),
            TYPE(-1), TYPE(1));
        return std::acos(cosine) * TYPE(180) /
            std::numbers::pi_v<TYPE>;
    }
}

void SchurVINS::pruneDeferredTrackArchives(const Tus timestamp) {
    for (auto archive = deferred_track_archives_.begin();
         archive != deferred_track_archives_.end();) {
        const bool expired = timestamp > archive->second.archived_at &&
            timestamp - archive->second.archived_at >
                deferred_track_archive_max_age_us_;
        if (expired) {
            archive = deferred_track_archives_.erase(archive);
            ++n_track_archives_expired_;
        } else {
            ++archive;
        }
    }
}

void SchurVINS::archiveDeferredTrack(
    const Landmark &landmark,
    const TriangulationStatus status,
    const Tus timestamp) {
    if (!enable_hybrid_persistent_landmarks_ ||
        landmark.frm2fet.size() < 2 ||
        isPersistentLandmark(landmark.id)) {
        return;
    }
    if (status != TriangulationStatus::LowParallax &&
        status != TriangulationStatus::IllConditioned &&
        status != TriangulationStatus::ExcessiveUncertainty &&
        status != TriangulationStatus::InsufficientViews) {
        return;
    }

    const Feature *first_feature = nullptr;
    const Feature *last_feature = nullptr;
    for (const auto &[frame_id, feature] : landmark.frm2fet) {
        (void)frame_id;
        if (!feature || !feature->frame || !feature->obs[0]) {
            continue;
        }
        if (!first_feature) {
            first_feature = feature;
        }
        last_feature = feature;
    }
    if (!first_feature || !last_feature || first_feature == last_feature) {
        return;
    }

    const Mat3_3 rotation_imu_camera = ext_.q_ic.toRotationMatrix();
    const ArchivedBearingSnapshot first = makeSnapshot(
        *first_feature, rotation_imu_camera, ext_.t_ic);
    const ArchivedBearingSnapshot last = makeSnapshot(
        *last_feature, rotation_imu_camera, ext_.t_ic);

    if (deferred_track_archives_.size() >= deferred_track_archive_capacity_ &&
        deferred_track_archives_.find(landmark.id) ==
            deferred_track_archives_.end()) {
        const auto oldest = std::min_element(
            deferred_track_archives_.begin(), deferred_track_archives_.end(),
            [](const auto &lhs, const auto &rhs) {
                return lhs.second.archived_at < rhs.second.archived_at;
            });
        if (oldest != deferred_track_archives_.end()) {
            deferred_track_archives_.erase(oldest);
            ++n_track_archives_expired_;
        }
    }

    auto existing = deferred_track_archives_.find(landmark.id);
    if (existing == deferred_track_archives_.end()) {
        DeferredTrackArchive archive;
        archive.id = landmark.id;
        archive.first = first;
        archive.last = last;
        archive.last_status = status;
        archive.observation_count = landmark.frm2fet.size();
        archive.archived_parallax_deg = bearingParallaxDeg(first, last);
        archive.archived_at = timestamp;
        deferred_track_archives_.emplace(landmark.id, archive);
    } else {
        // 同一 ID 多次短暂离开视野时保留最早射线，并用最新射线扩展时间/基线。
        // 这仍只是候选筛选数据，不会被当成独立导航量测。
        existing->second.last = last;
        existing->second.last_status = status;
        existing->second.observation_count += landmark.frm2fet.size();
        existing->second.archived_parallax_deg = std::max(
            existing->second.archived_parallax_deg,
            bearingParallaxDeg(existing->second.first, last));
        existing->second.archived_at = timestamp;
    }
    ++n_track_archives_created_;
}

void SchurVINS::updateShadowCandidateFromArchive(
    const LandmarkID id,
    const Frame &current_frame,
    const Vec2 &measurement,
    const Tus timestamp) {
    auto archive_it = deferred_track_archives_.find(id);
    if (!enable_hybrid_persistent_landmarks_ ||
        archive_it == deferred_track_archives_.end() ||
        isPersistentLandmark(id)) {
        return;
    }

    const Mat3_3 rotation_world_imu =
        current_frame.q().toRotationMatrix();
    ArchivedBearingSnapshot current;
    current.bearing_camera =
        Vec3(measurement.x(), measurement.y(), TYPE(1)).normalized();
    current.rotation_world_camera =
        rotation_world_imu * ext_.q_ic.toRotationMatrix();
    current.camera_center_world =
        current_frame.p() + rotation_world_imu * ext_.t_ic;
    current.timestamp = current_frame.timestamp;

    DeferredTrackArchive &archive = archive_it->second;
    const std::array<ArchivedBearingSnapshot, 3> snapshots{
        archive.first, archive.last, current};

    TYPE max_parallax_deg = TYPE(0);
    for (size_t lhs = 0; lhs < snapshots.size(); ++lhs) {
        for (size_t rhs = lhs + 1; rhs < snapshots.size(); ++rhs) {
            max_parallax_deg = std::max(
                max_parallax_deg,
                bearingParallaxDeg(snapshots[lhs], snapshots[rhs]));
        }
    }
    if (max_parallax_deg < triangulation_min_parallax_deg) {
        return;
    }
    if (archive.last_candidate_attempt != 0 &&
        timestamp > archive.last_candidate_attempt &&
        timestamp - archive.last_candidate_attempt <
            deferred_track_archive_retry_interval_us_) {
        return;
    }
    archive.last_candidate_attempt = timestamp;

    // 射线交会的初值：
    //   min_p sum_i ||(I-d_i d_i^T)(p-c_i)||^2
    // 对应正规方程 A p=b，其中 A=sum(I-d_i d_i^T)。纯旋转时所有 d_i
    // 几乎平行，A 的最小特征值趋近 0，下面的秩与条件数检查会拒绝该候选。
    Mat3_3 ray_hessian = Mat3_3::Zero();
    Vec3 ray_gradient = Vec3::Zero();
    for (const auto &snapshot : snapshots) {
        const Vec3 direction =
            (snapshot.rotation_world_camera * snapshot.bearing_camera).normalized();
        const Mat3_3 projector =
            Mat3_3::Identity() - direction * direction.transpose();
        ray_hessian += projector;
        ray_gradient += projector * snapshot.camera_center_world;
    }
    Eigen::SelfAdjointEigenSolver<Mat3_3> ray_es(
        TYPE(0.5) * (ray_hessian + ray_hessian.transpose()));
    if (ray_es.info() != Eigen::Success ||
        ray_es.eigenvalues().minCoeff() <= TYPE(1e-8)) {
        ++n_track_archives_rejected_;
        return;
    }
    const TYPE condition_number = ray_es.eigenvalues().maxCoeff() /
        ray_es.eigenvalues().minCoeff();
    if (!std::isfinite(condition_number) || condition_number > TYPE(1e8)) {
        ++n_track_archives_rejected_;
        return;
    }
    const Vec3 position = ray_es.eigenvectors() *
        ray_es.eigenvalues().cwiseInverse().asDiagonal() *
        ray_es.eigenvectors().transpose() * ray_gradient;
    if (!position.allFinite()) {
        ++n_track_archives_rejected_;
        return;
    }

    TYPE squared_reprojection_error = TYPE(0);
    Mat3_3 information = Mat3_3::Zero();
    for (const auto &snapshot : snapshots) {
        const Vec3 delta = position - snapshot.camera_center_world;
        const TYPE range = delta.norm();
        if (!(range > TYPE(0.05))) {
            ++n_track_archives_rejected_;
            return;
        }
        const Vec3 point_camera =
            snapshot.rotation_world_camera.transpose() * delta;
        if (point_camera.z() <= TYPE(0.05)) {
            ++n_track_archives_rejected_;
            return;
        }
        const Vec2 prediction = point_camera.head<2>() / point_camera.z();
        const Vec2 observation =
            snapshot.bearing_camera.head<2>() / snapshot.bearing_camera.z();
        squared_reprojection_error += (observation - prediction).squaredNorm();

        // bearing 对世界点的局部雅可比近似为 (I-dd^T)/range。这里得到的
        // 协方差仅供影子候选 NIS 与质量评分使用，不会直接写入导航状态。
        const Vec3 direction = delta / range;
        const Mat3_3 bearing_jacobian =
            (Mat3_3::Identity() - direction * direction.transpose()) / range;
        information.noalias() += bearing_jacobian.transpose() * bearing_jacobian;
    }
    const TYPE reprojection_rmse = std::sqrt(
        squared_reprojection_error /
        static_cast<TYPE>(snapshots.size()));
    if (!std::isfinite(reprojection_rmse) ||
        reprojection_rmse > triangulation_max_reprojection_rmse) {
        ++n_track_archives_rejected_;
        return;
    }

    Eigen::SelfAdjointEigenSolver<Mat3_3> information_es(
        TYPE(0.5) * (information + information.transpose()));
    if (information_es.info() != Eigen::Success ||
        information_es.eigenvalues().minCoeff() <= TYPE(1e-12)) {
        ++n_track_archives_rejected_;
        return;
    }
    Mat3_3 covariance =
        triangulation_uv_std * triangulation_uv_std *
        information_es.eigenvectors() *
        information_es.eigenvalues().cwiseInverse().asDiagonal() *
        information_es.eigenvectors().transpose();
    covariance = TYPE(0.5) * (covariance + covariance.transpose());

    Landmark detached;
    detached.id = id;
    detached.is_triangulated = true;
    detached.position = position;
    detached.cov_position = covariance;
    detached.geometry_max_parallax_deg = max_parallax_deg;
    detached.geometry_condition_number = condition_number;
    detached.geometry_reprojection_rmse = reprojection_rmse;
    detached.geometry_position_std = std::sqrt(std::max(
        TYPE(0), covariance.trace() / TYPE(3)));
    detached.geometry_observation_count =
        archive.observation_count + size_t(1);
    const TrackGeometryQuality quality = evaluateTrackGeometry(detached);
    if (!quality.valid) {
        ++n_track_archives_rejected_;
        return;
    }

    updateShadowCandidateEstimate(
        id, position, covariance, quality, timestamp);
    deferred_track_archives_.erase(archive_it);
    ++n_track_archives_reused_;
}
