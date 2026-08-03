#include "frame_selection_policy.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace {
    using namespace slam;

    [[nodiscard]] bool validTrackObservation(const Feature *feature) {
        return feature && feature->frame && feature->obs[0] &&
               feature->obs[0]->un_pt.allFinite() &&
               feature->obs[0]->un_pt.squaredNorm() > TYPE(1e-12);
    }

    [[nodiscard]] Vec3 worldBearing(
        const Feature &feature,
        const Quat &q_ic) {
        const Mat3_3 Rwc = feature.frame->q().toRotationMatrix() *
                           q_ic.toRotationMatrix();
        return (Rwc * feature.obs[0]->un_pt.normalized()).normalized();
    }

    // 用所有有效单位世界视线的最大夹角衡量平移视差。纯旋转时，经过各 clone
    // 姿态补偿后的世界视线近似重合，因此该角度接近零；存在平移基线时角度增大。
    [[nodiscard]] TYPE maximumRotationCompensatedParallaxDeg(
        const Landmark &landmark,
        const Quat &q_ic,
        const std::optional<FrameID> excluded_frame_id = std::nullopt) {
        std::vector<Vec3, Eigen::aligned_allocator<Vec3>> bearings;
        bearings.reserve(landmark.frm2fet.size());
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            if ((excluded_frame_id && frame_id == *excluded_frame_id) ||
                !validTrackObservation(feature)) {
                continue;
            }
            const Vec3 bearing = worldBearing(*feature, q_ic);
            if (bearing.allFinite()) {
                bearings.push_back(bearing);
            }
        }

        TYPE maximum_angle_deg = TYPE(0);
        for (size_t first = 0; first < bearings.size(); ++first) {
            for (size_t second = first + 1; second < bearings.size(); ++second) {
                const TYPE cosine = std::clamp(
                    bearings[first].dot(bearings[second]), TYPE(-1), TYPE(1));
                maximum_angle_deg = std::max(
                    maximum_angle_deg,
                    std::acos(cosine) * TYPE(180) / std::numbers::pi_v<TYPE>);
            }
        }
        return maximum_angle_deg;
    }

    [[nodiscard]] size_t validObservationCount(
        const Landmark &landmark,
        const std::optional<FrameID> excluded_frame_id = std::nullopt) {
        size_t count = 0;
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            if ((!excluded_frame_id || frame_id != *excluded_frame_id) &&
                validTrackObservation(feature)) {
                ++count;
            }
        }
        return count;
    }

    [[nodiscard]] bool observedInFrame(
        const Landmark &landmark,
        const Frame *frame) {
        if (!frame) {
            return false;
        }
        const auto observation = landmark.frm2fet.find(frame->id);
        return observation != landmark.frm2fet.end() &&
               validTrackObservation(observation->second);
    }

    [[nodiscard]] TYPE medianKeyframeInterval(const Map &map) {
        std::vector<TYPE> intervals;
        intervals.reserve(map.sfw.size());
        for (size_t index = 1; index < map.sfw.size(); ++index) {
            const Frame *previous = map.sfw[index - 1];
            const Frame *current = map.sfw[index];
            if (previous && current && current->timestamp > previous->timestamp) {
                intervals.push_back(static_cast<TYPE>(
                    current->timestamp - previous->timestamp));
            }
        }
        if (intervals.empty()) {
            return TYPE(1);
        }
        const size_t middle = intervals.size() / 2;
        std::nth_element(
            intervals.begin(), intervals.begin() + middle, intervals.end());
        return std::max(intervals[middle], TYPE(1));
    }
}

namespace slam {
    // VINS-Mono 风格关键帧判定采用旋转补偿后的平均视差。设上一帧单位视线为
    // b_{k-1}，当前帧单位视线为 b_k，IMU 预测的相机相对旋转为
    // R_{k<-k-1}=R_wc,k^T R_wc,k-1，则纯旋转预测视线为
    // b_rot=R_{k<-k-1} b_{k-1}，平移视差角为 acos(b_k^T b_rot)。
    // 平均视差超过阈值时保留关键帧；公共轨迹过少或关键帧间隔过长时强制保留，
    // 防止低纹理和长时间纯旋转导致窗口只剩极少有效基线。
    VINSMonoFrameDecision decideVINSMonoFrame(
        const Map &map,
        const CameraData &camera_data,
        const Quat &current_orientation,
        const Quat &q_ic,
        const VINSMonoKeyframeParameters &parameters) {
        VINSMonoFrameDecision decision;
        if (camera_data.measurements.size() < parameters.minimum_frame_features) {
            return decision;
        }
        if (map.sfw.empty()) {
            decision.is_keyframe = true;
            return decision;
        }

        const Frame *latest_keyframe = nullptr;
        for (size_t index = map.sfw.size(); index > 0; --index) {
            const Frame *candidate = map.sfw[index - 1];
            if (candidate && candidate->is_key_frame) {
                latest_keyframe = candidate;
                break;
            }
        }
        if (!latest_keyframe ||
            camera_data.timestamp >= latest_keyframe->timestamp +
                                     parameters.maximum_keyframe_interval_us) {
            decision.is_keyframe = true;
            return decision;
        }

        const Frame *previous = map.sfw[map.sfw.size() - 1];
        const Mat3_3 Rwc_previous =
            previous->q().toRotationMatrix() * q_ic.toRotationMatrix();
        const Mat3_3 Rwc_current =
            current_orientation.toRotationMatrix() * q_ic.toRotationMatrix();
        const Mat3_3 previous_to_current =
            Rwc_current.transpose() * Rwc_previous;

        TYPE parallax_sum_deg = TYPE(0);
        for (const auto &[id, measurement] : camera_data.measurements) {
            const auto previous_feature = previous->lmk2fet.find(id);
            if (previous_feature == previous->lmk2fet.end() ||
                !previous_feature->second ||
                !previous_feature->second->obs[0]) {
                continue;
            }
            const Vec3 previous_bearing =
                previous_feature->second->obs[0]->un_pt.normalized();
            const Vec3 current_bearing =
                Vec3(measurement.x(), measurement.y(), TYPE(1)).normalized();
            const TYPE cosine = std::clamp(
                current_bearing.dot(previous_to_current * previous_bearing),
                TYPE(-1), TYPE(1));
            parallax_sum_deg +=
                std::acos(cosine) * TYPE(180) / std::numbers::pi_v<TYPE>;
            ++decision.common_tracks;
        }

        if (decision.common_tracks < parameters.minimum_common_tracks) {
            decision.is_keyframe = true;
            return decision;
        }
        decision.mean_rotation_compensated_parallax_deg =
            parallax_sum_deg / static_cast<TYPE>(decision.common_tracks);
        decision.is_keyframe =
            decision.mean_rotation_compensated_parallax_deg >=
            parameters.minimum_rotation_compensated_parallax_deg;
        return decision;
    }

    KeyframeRemovalDecision planKeyframeRedundancyRemoval(
        const Map &map,
        const Quat &q_ic,
        const size_t retained_clone_count,
        const KeyframeRedundancyParameters &parameters) {
        KeyframeRemovalDecision decision;
        const size_t window_size = map.sfw.size();
        if (window_size <= retained_clone_count || window_size == 0) {
            return decision;
        }

        // 最新两帧通常承载大量尚未形成平移基线的 continuing tracks。VINS-Mono
        // 风格删除次新帧在固定 MSCKF 后端中会反复触发这些轨迹，因此先把最近帧从
        // 候选集合硬排除，再从剩余帧的较老一半中寻找明显冗余的关键帧。
        const size_t protected_count = std::min(
            parameters.protected_recent_keyframes,
            window_size > 0 ? window_size - 1 : size_t(0));
        const size_t unprotected_count = window_size - protected_count;
        const TYPE bounded_fraction = std::clamp(
            parameters.candidate_window_fraction, TYPE(0.05), TYPE(1));
        const size_t candidate_count = std::clamp(
            static_cast<size_t>(std::ceil(
                static_cast<TYPE>(unprotected_count) * bounded_fraction)),
            size_t(1), unprotected_count);
        decision.candidate_count = candidate_count;

        // MSCKF 在轨迹丢失时会立即消费并从所有 Frame::lmk2fet 中解除引用，所以
        // 很多数据集上，最老 clone 到达窗口边界时已经没有任何活跃轨迹。此时它
        // 不可能再参与未来视觉因子，删除它严格支配删除任意仍有轨迹的中间帧；
        // 直接返回还能避免后续 O(N_candidate * N_track * N_obs^2) 的视差计算。
        const Frame *oldest_frame = map.sfw[0];
        size_t oldest_active_tracks = 0;
        if (oldest_frame) {
            for (const auto &[landmark_id, feature] : oldest_frame->lmk2fet) {
                (void)landmark_id;
                if (validTrackObservation(feature) && feature->landmark) {
                    ++oldest_active_tracks;
                }
            }
        }
        if (oldest_frame && oldest_active_tracks == 0) {
            decision.oldest.chronological_index = 0;
            decision.oldest.frame_id = oldest_frame->id;
            decision.oldest.redundancy_ratio = TYPE(1);
            decision.oldest.low_support_ratio = TYPE(1);
            decision.oldest.age_ratio = TYPE(1);
            decision.oldest.score =
                parameters.redundancy_reward +
                parameters.low_support_reward +
                parameters.age_reward;
            decision.selected = decision.oldest;
            decision.valid = true;
            return decision;
        }

        const Frame *latest_frame = map.sfw[window_size - 1];
        const TYPE parallax_threshold = std::max(
            parameters.minimum_parallax_deg, TYPE(1e-3));
        const TYPE median_interval = medianKeyframeInterval(map);
        std::vector<KeyframeRemovalCandidate> candidates;
        candidates.reserve(candidate_count);
        size_t maximum_active_tracks = 0;

        for (size_t index = 0; index < candidate_count; ++index) {
            KeyframeRemovalCandidate candidate;
            candidate.chronological_index = index;
            const Frame *frame = map.sfw[index];
            if (!frame) {
                continue;
            }
            candidate.frame_id = frame->id;
            candidate.age_ratio = window_size > 1
                ? TYPE(window_size - 1 - index) / TYPE(window_size - 1)
                : TYPE(1);

            // 删除内部帧会把左右两个时间间隔合并。均匀关键帧序列中该项约为 1，
            // 删除最老边界帧则为 0；它只提供轻微连续性偏好，不覆盖轨迹几何判据。
            if (index > 0 && index + 1 < window_size) {
                const Frame *previous = map.sfw[index - 1];
                const Frame *next = map.sfw[index + 1];
                if (previous && next && next->timestamp > previous->timestamp) {
                    const TYPE merged_interval = static_cast<TYPE>(
                        next->timestamp - previous->timestamp);
                    candidate.temporal_gap_ratio = std::clamp(
                        merged_interval / median_interval - TYPE(1),
                        TYPE(0), TYPE(1));
                }
            }

            TYPE parallax_loss_sum = TYPE(0);
            size_t parallax_loss_track_count = 0;
            for (const auto &[landmark_id, feature] : frame->lmk2fet) {
                (void)landmark_id;
                if (!validTrackObservation(feature) || !feature->landmark) {
                    continue;
                }
                const Landmark &landmark = *feature->landmark;
                if (landmark.frm2fet.find(frame->id) == landmark.frm2fet.end()) {
                    continue;
                }

                ++candidate.active_tracks;
                const size_t observation_count = validObservationCount(landmark);
                const size_t remaining_observations =
                    validObservationCount(landmark, frame->id);
                const bool continues = observedInFrame(landmark, latest_frame);
                const bool consumed_by_length =
                    observation_count >= retained_clone_count;
                const TYPE parallax_before =
                    maximumRotationCompensatedParallaxDeg(landmark, q_ic);
                const TYPE parallax_after =
                    maximumRotationCompensatedParallaxDeg(
                        landmark, q_ic, frame->id);
                const TYPE parallax_loss_ratio = std::clamp(
                    (parallax_before - parallax_after) / parallax_threshold,
                    TYPE(0), TYPE(1));

                // lost 和长度上限本来就会结算轨迹，不属于候选删帧新增的副作用。
                // 对仍在当前帧可见、且尚未达到长度上限的轨迹，删除候选 clone
                // 才会额外触发 touches_marginalized_clone。
                const bool candidate_triggers_consumption =
                    continues && !consumed_by_length;
                const bool low_parallax_risk =
                    candidate_triggers_consumption &&
                    !landmark.is_triangulated &&
                    parallax_before < parallax_threshold;
                if (low_parallax_risk) {
                    ++candidate.low_parallax_tracks;
                }

                // 若低视差轨迹被延迟消费，候选帧观测仍会随 clone 一起删除。
                // 当剩余观测不足两帧，或候选帧贡献了明显比例的最大视差时，
                // 将其标记为“独特几何”，防止为了共视计数而破坏未来三角化基线。
                const bool unique_geometry =
                    low_parallax_risk &&
                    (remaining_observations < 2 ||
                     parallax_loss_ratio >= TYPE(0.25));
                if (unique_geometry) {
                    ++candidate.unique_geometry_tracks;
                }

                // 下列情况说明删除该观测不会破坏一条仍需继续积累基线的轨迹：
                // 轨迹本来就会结算；已经有深度；当前几何已达到三角化门限；或者
                // 删除后仍保留至少两个视角且最大视差几乎不下降。
                const bool redundant =
                    !candidate_triggers_consumption ||
                    landmark.is_triangulated ||
                    parallax_before >= parallax_threshold ||
                    (remaining_observations >= 2 &&
                     parallax_loss_ratio <= TYPE(0.10));
                if (redundant) {
                    ++candidate.redundant_tracks;
                }
                if (candidate_triggers_consumption && !landmark.is_triangulated) {
                    parallax_loss_sum += parallax_loss_ratio;
                    ++parallax_loss_track_count;
                }
            }

            maximum_active_tracks = std::max(
                maximum_active_tracks, candidate.active_tracks);
            if (candidate.active_tracks == 0) {
                // 没有活跃轨迹引用的 clone 不会再参与未来视觉因子，可视作完全冗余。
                candidate.redundancy_ratio = TYPE(1);
            } else {
                candidate.redundancy_ratio =
                    TYPE(candidate.redundant_tracks) /
                    TYPE(candidate.active_tracks);
                candidate.mean_parallax_loss_ratio =
                    parallax_loss_track_count > 0
                        ? parallax_loss_sum /
                          TYPE(parallax_loss_track_count)
                        : TYPE(0);
            }
            candidates.push_back(candidate);
        }

        if (candidates.empty()) {
            return decision;
        }

        for (auto &candidate : candidates) {
            candidate.low_support_ratio = maximum_active_tracks > 0
                ? TYPE(1) - TYPE(candidate.active_tracks) /
                              TYPE(maximum_active_tracks)
                : TYPE(1);
            const TYPE low_parallax_ratio = candidate.active_tracks > 0
                ? TYPE(candidate.low_parallax_tracks) /
                  TYPE(candidate.active_tracks)
                : TYPE(0);
            const TYPE unique_geometry_ratio = candidate.active_tracks > 0
                ? TYPE(candidate.unique_geometry_tracks) /
                  TYPE(candidate.active_tracks)
                : TYPE(0);

            // 删除效用：高冗余、低活跃支持和较老帧得到奖励；会触发低视差延迟、
            // 破坏独特基线、显著降低最大视差或制造内部时间缺口的候选受到惩罚。
            candidate.score =
                parameters.redundancy_reward * candidate.redundancy_ratio +
                parameters.low_support_reward * candidate.low_support_ratio +
                parameters.age_reward * candidate.age_ratio -
                parameters.low_parallax_penalty * low_parallax_ratio -
                parameters.unique_geometry_penalty * unique_geometry_ratio -
                parameters.parallax_loss_penalty *
                    candidate.mean_parallax_loss_ratio -
                parameters.temporal_gap_penalty *
                    candidate.temporal_gap_ratio;
        }

        decision.oldest = candidates.front();
        const auto best = std::max_element(
            candidates.begin(), candidates.end(),
            [](const KeyframeRemovalCandidate &lhs,
               const KeyframeRemovalCandidate &rhs) {
                if (lhs.score == rhs.score) {
                    // 分数完全相同时保留 FIFO 的确定性，优先删除时间更早的帧。
                    return lhs.chronological_index > rhs.chronological_index;
                }
                return lhs.score < rhs.score;
            });
        decision.selected = *best;

        // 非最老候选必须获得明确的评分优势；差异只来自数值噪声或少量轨迹时，
        // 回退到经过现有消融验证的最老关键帧策略。
        if (decision.selected.chronological_index != 0 &&
            decision.selected.score <
                decision.oldest.score + parameters.minimum_score_advantage) {
            decision.selected = decision.oldest;
            decision.used_oldest_fallback = true;
        }
        decision.valid = true;
        return decision;
    }
}
