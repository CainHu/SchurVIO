#include "rdvio_scheduler.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace slam {
    RDVIOFrameDecision decideRDVIOFrame(
        const Map &map,
        const CameraData &camera_data,
        const Quat &current_orientation,
        const Quat &q_ic,
        const bool fallback_keyframe,
        const RDVIOParameters &parameters) {
        RDVIOFrameDecision decision;
        decision.is_keyframe = fallback_keyframe;

        if (map.sfw.empty()) {
            // RD-VIO starts its hierarchy from a normal keyframe.
            decision.is_keyframe = true;
            return decision;
        }

        const Frame *previous = map.sfw[map.sfw.size() - 1];
        std::vector<TYPE> angular_errors;
        angular_errors.reserve(std::min(previous->lmk2fet.size(),
                                        camera_data.measurements.size()));
        const Mat3_3 Rwc_previous =
            previous->q().toRotationMatrix() * q_ic.toRotationMatrix();
        const Mat3_3 Rwc_current =
            current_orientation.toRotationMatrix() * q_ic.toRotationMatrix();
        const Mat3_3 previous_to_current =
            Rwc_current.transpose() * Rwc_previous;

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
            angular_errors.push_back(
                std::acos(cosine) * TYPE(180) / std::numbers::pi_v<TYPE>);
        }

        if (angular_errors.size() >= parameters.min_common_tracks) {
            const size_t quantile_index = std::min(
                angular_errors.size() - 1, angular_errors.size() * 7 / 10);
            std::nth_element(angular_errors.begin(),
                             angular_errors.begin() + quantile_index,
                             angular_errors.end());
            decision.misalignment_deg = angular_errors[quantile_index];
            decision.is_rotation_frame =
                decision.misalignment_deg < parameters.rotation_threshold_deg;
        }

        const bool previous_rotation = previous->is_rotation_frame;
        decision.transition = previous_rotation
            ? (decision.is_rotation_frame ? RDVIOCase::RR : RDVIOCase::RN)
            : (decision.is_rotation_frame ? RDVIOCase::NR : RDVIOCase::NN);

        // The flat ESKF clone window represents RD-VIO subframes with
        // is_key_frame=false. RN/NR transitions close the previous segment.
        if (map.sfw.size() == 1 || previous->is_key_frame) {
            decision.is_keyframe = false;
        } else if (decision.transition == RDVIOCase::RR) {
            decision.is_keyframe = false;
        } else if (decision.transition == RDVIOCase::RN) {
            decision.promote_previous_to_keyframe = true;
            decision.is_keyframe = true;
        } else if (decision.transition == RDVIOCase::NR) {
            decision.promote_previous_to_keyframe = true;
            decision.is_keyframe = false;
        } else {
            size_t normal_subframes = 0;
            for (size_t index = map.sfw.size(); index > 0; --index) {
                const Frame *frame = map.sfw[index - 1];
                if (frame->is_key_frame) {
                    break;
                }
                normal_subframes += frame->is_rotation_frame ? 0 : 1;
            }
            decision.is_keyframe =
                normal_subframes >= parameters.normal_subframe_limit;
        }
        return decision;
    }

    RDVIOFrameRemovalPlan planRDVIOFrameRemovals(
        const Map &map,
        const RDVIOParameters &parameters) {
        RDVIOFrameRemovalPlan plan;
        std::vector<size_t> rotation_subframes;
        const size_t current_window_size = map.sfw.size();

        for (size_t index = current_window_size; index > 0; --index) {
            const size_t chronological = index - 1;
            const Frame *frame = map.sfw[chronological];
            if (frame->is_key_frame || !frame->is_rotation_frame) {
                break;
            }
            rotation_subframes.push_back(chronological);
        }
        std::reverse(rotation_subframes.begin(), rotation_subframes.end());

        if (rotation_subframes.size() >=
            parameters.rotation_compression_trigger) {
            const size_t complete_groups = rotation_subframes.size() / 3;
            for (size_t group = 0; group < complete_groups; ++group) {
                plan.chronological_indices.push_back(
                    rotation_subframes[group * 3]);
                plan.chronological_indices.push_back(
                    rotation_subframes[group * 3 + 1]);
                plan.compressed_frame_count += 2;
            }
        }

        const size_t projected_size =
            current_window_size - plan.chronological_indices.size();
        if (projected_size > parameters.retained_clone_count) {
            size_t oldest = 0;
            while (std::find(plan.chronological_indices.begin(),
                             plan.chronological_indices.end(), oldest) !=
                   plan.chronological_indices.end()) {
                ++oldest;
            }
            plan.chronological_indices.push_back(oldest);
        }

        std::sort(plan.chronological_indices.begin(),
                  plan.chronological_indices.end());
        plan.chronological_indices.erase(
            std::unique(plan.chronological_indices.begin(),
                        plan.chronological_indices.end()),
            plan.chronological_indices.end());
        return plan;
    }
}
