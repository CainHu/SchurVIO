#pragma once

#include "../common.h"
#include "../data_structure/map.h"
#include "visual_update_scheduler.h"

#include <cstddef>
#include <cstdint>

#define SCHUR_VIO_FRAME_POLICY_AUTO 0
#define SCHUR_VIO_FRAME_POLICY_KEYFRAME_ONLY 1
#define SCHUR_VIO_FRAME_POLICY_FIFO 2
#define SCHUR_VIO_FRAME_POLICY_KEYFRAME_PRIORITY 3
#define SCHUR_VIO_FRAME_POLICY_VINS_MONO 4
#define SCHUR_VIO_FRAME_POLICY_RDVIO 5
#define SCHUR_VIO_FRAME_POLICY_KEYFRAME_REDUNDANCY 6

#ifndef SCHUR_VIO_FRAME_POLICY
#define SCHUR_VIO_FRAME_POLICY SCHUR_VIO_FRAME_POLICY_AUTO
#endif

#ifndef SCHUR_VIO_FRAME_WINDOW_SIZE
#define SCHUR_VIO_FRAME_WINDOW_SIZE 0
#endif

static_assert(SCHUR_VIO_FRAME_POLICY >= SCHUR_VIO_FRAME_POLICY_AUTO &&
              SCHUR_VIO_FRAME_POLICY <= SCHUR_VIO_FRAME_POLICY_KEYFRAME_REDUNDANCY,
              "Unknown SCHUR_VIO_FRAME_POLICY value");
static_assert(SCHUR_VIO_FRAME_WINDOW_SIZE >= 0 &&
              SCHUR_VIO_FRAME_WINDOW_SIZE < 30,
              "SCHUR_VIO_FRAME_WINDOW_SIZE must leave one incoming-clone slot");

namespace slam {
    enum class FrameSelectionPolicy : uint8_t {
        Auto = SCHUR_VIO_FRAME_POLICY_AUTO,
        KeyframeOnly = SCHUR_VIO_FRAME_POLICY_KEYFRAME_ONLY,
        FIFO = SCHUR_VIO_FRAME_POLICY_FIFO,
        KeyframePriority = SCHUR_VIO_FRAME_POLICY_KEYFRAME_PRIORITY,
        VINSMono = SCHUR_VIO_FRAME_POLICY_VINS_MONO,
        RDVIO = SCHUR_VIO_FRAME_POLICY_RDVIO,
        KeyframeRedundancy = SCHUR_VIO_FRAME_POLICY_KEYFRAME_REDUNDANCY
    };

    constexpr FrameSelectionPolicy CONFIGURED_FRAME_SELECTION_POLICY =
        static_cast<FrameSelectionPolicy>(SCHUR_VIO_FRAME_POLICY);

    [[nodiscard]] constexpr FrameSelectionPolicy defaultFrameSelectionPolicy(
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        switch (scheduler) {
            case VisualUpdateScheduler::Legacy:
                return FrameSelectionPolicy::KeyframeOnly;
            case VisualUpdateScheduler::SchurVINS:
                return FrameSelectionPolicy::KeyframePriority;
            case VisualUpdateScheduler::MSCKF:
                // The fixed-backend ablation shows that keyframe-only clone
                // selection recovers Legacy-level RMSE without reusing a
                // visual sample in multiple posteriors.
                return FrameSelectionPolicy::KeyframeOnly;
            case VisualUpdateScheduler::VINSMono:
                return FrameSelectionPolicy::VINSMono;
            case VisualUpdateScheduler::RDVIO:
                return FrameSelectionPolicy::RDVIO;
        }
        return FrameSelectionPolicy::FIFO;
    }

    [[nodiscard]] constexpr FrameSelectionPolicy resolvedFrameSelectionPolicy(
        const FrameSelectionPolicy configured = CONFIGURED_FRAME_SELECTION_POLICY,
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        return configured == FrameSelectionPolicy::Auto
            ? defaultFrameSelectionPolicy(scheduler)
            : configured;
    }

    constexpr FrameSelectionPolicy FRAME_SELECTION_POLICY =
        resolvedFrameSelectionPolicy();

    [[nodiscard]] constexpr const char *frameSelectionPolicyName(
        const FrameSelectionPolicy policy = FRAME_SELECTION_POLICY) {
        switch (policy) {
            case FrameSelectionPolicy::Auto: return "auto";
            case FrameSelectionPolicy::KeyframeOnly: return "keyframe_only";
            case FrameSelectionPolicy::FIFO: return "fifo";
            case FrameSelectionPolicy::KeyframePriority: return "keyframe_priority";
            case FrameSelectionPolicy::VINSMono: return "vins_mono";
            case FrameSelectionPolicy::RDVIO: return "rdvio";
            case FrameSelectionPolicy::KeyframeRedundancy: return "keyframe_redundancy";
        }
        return "unknown";
    }

    [[nodiscard]] constexpr bool framePolicyAugmentsEveryImage(
        const FrameSelectionPolicy policy = FRAME_SELECTION_POLICY) {
        return policy != FrameSelectionPolicy::KeyframeOnly &&
               policy != FrameSelectionPolicy::KeyframeRedundancy;
    }

    [[nodiscard]] constexpr size_t framePolicyDefaultRetainedCloneCount(
        const FrameSelectionPolicy policy = FRAME_SELECTION_POLICY,
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        switch (policy) {
            case FrameSelectionPolicy::Auto: return 20;
            case FrameSelectionPolicy::KeyframeOnly:
                return scheduler == VisualUpdateScheduler::Legacy ? 29 : 20;
            case FrameSelectionPolicy::FIFO: return 20;
            case FrameSelectionPolicy::KeyframePriority: return 3;
            case FrameSelectionPolicy::VINSMono: return 10;
            case FrameSelectionPolicy::RDVIO: return 18;
            case FrameSelectionPolicy::KeyframeRedundancy: return 20;
        }
        return 20;
    }

    [[nodiscard]] constexpr size_t framePolicyRetainedCloneCount(
        const FrameSelectionPolicy policy = FRAME_SELECTION_POLICY) {
        if constexpr (SCHUR_VIO_FRAME_WINDOW_SIZE > 0) {
            return SCHUR_VIO_FRAME_WINDOW_SIZE;
        }
        return framePolicyDefaultRetainedCloneCount(policy);
    }

    struct VINSMonoKeyframeParameters {
        size_t minimum_frame_features{10};
        size_t minimum_common_tracks{20};
        TYPE minimum_rotation_compensated_parallax_deg{TYPE(1.25)};
        Tus maximum_keyframe_interval_us{1000000};
    };

    struct VINSMonoFrameDecision {
        bool is_keyframe{false};
        size_t common_tracks{};
        TYPE mean_rotation_compensated_parallax_deg{};
    };

    [[nodiscard]] VINSMonoFrameDecision decideVINSMonoFrame(
        const Map &map,
        const CameraData &camera_data,
        const Quat &current_orientation,
        const Quat &q_ic,
        const VINSMonoKeyframeParameters &parameters);

    // 冗余感知关键帧删除仍然保持 KeyframeOnly 的“只增广关键帧”语义，只替换窗口
    // 满时删除哪一个关键帧。第一版故意采用保守参数：最新两帧完全保护，候选只来自
    // 较老半窗；只有候选明显优于最老帧时才允许删除中间关键帧。
    struct KeyframeRedundancyParameters {
        size_t protected_recent_keyframes{2};
        TYPE candidate_window_fraction{TYPE(0.5)};
        TYPE minimum_parallax_deg{TYPE(2)};
        TYPE minimum_score_advantage{TYPE(0.10)};

        TYPE redundancy_reward{TYPE(1.20)};
        TYPE low_support_reward{TYPE(0.50)};
        TYPE age_reward{TYPE(0.35)};
        TYPE low_parallax_penalty{TYPE(4.00)};
        TYPE unique_geometry_penalty{TYPE(2.00)};
        TYPE parallax_loss_penalty{TYPE(1.50)};
        TYPE temporal_gap_penalty{TYPE(0.25)};
    };

    // 单个候选关键帧的可解释评分项。score 越大越适合删除；所有比率均归一化
    // 到 [0,1]，便于在不同特征点数量和仿真场景之间使用同一组权重。
    struct KeyframeRemovalCandidate {
        size_t chronological_index{};
        FrameID frame_id{};
        TYPE score{};
        size_t active_tracks{};
        size_t redundant_tracks{};
        size_t low_parallax_tracks{};
        size_t unique_geometry_tracks{};
        TYPE redundancy_ratio{};
        TYPE low_support_ratio{};
        TYPE age_ratio{};
        TYPE mean_parallax_loss_ratio{};
        TYPE temporal_gap_ratio{};
    };

    struct KeyframeRemovalDecision {
        bool valid{false};
        bool used_oldest_fallback{false};
        size_t candidate_count{};
        KeyframeRemovalCandidate selected;
        KeyframeRemovalCandidate oldest;
    };

    [[nodiscard]] KeyframeRemovalDecision planKeyframeRedundancyRemoval(
        const Map &map,
        const Quat &q_ic,
        size_t retained_clone_count,
        const KeyframeRedundancyParameters &parameters);
}
