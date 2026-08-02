#include "frame_selection_policy.h"

#include <algorithm>
#include <cmath>
#include <numbers>

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
}
