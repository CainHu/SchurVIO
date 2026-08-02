#pragma once

#include "../common.h"
#include "../data_structure/map.h"

namespace slam {
    enum class RDVIOCase : uint8_t {
        None = 0,
        RR = 1,
        NN = 2,
        RN = 3,
        NR = 4
    };

    struct RDVIOParameters {
        TYPE rotation_threshold_deg{TYPE(0.60)};
        size_t min_common_tracks{20};
        size_t normal_subframe_limit{3};
        size_t rotation_compression_trigger{9};
        size_t retained_clone_count{18};
    };

    struct RDVIOFrameDecision {
        bool is_keyframe{false};
        bool is_rotation_frame{false};
        bool promote_previous_to_keyframe{false};
        TYPE misalignment_deg{TYPE(180)};
        RDVIOCase transition{RDVIOCase::None};
    };

    struct RDVIOFrameRemovalPlan {
        std::vector<size_t> chronological_indices;
        size_t compressed_frame_count{};
    };

    // Classify the incoming image as rotation-dominant (R) or normal (N), then
    // apply the RR/NN/RN/NR keyframe policy. The function is read-only: callers
    // explicitly apply promotion of the previous frame after inspecting the
    // returned decision.
    [[nodiscard]] RDVIOFrameDecision decideRDVIOFrame(
        const Map &map,
        const CameraData &camera_data,
        const Quat &current_orientation,
        const Quat &q_ic,
        bool fallback_keyframe,
        const RDVIOParameters &parameters);

    // Compress the trailing pure-rotation subwindow and enforce the clone
    // budget. Returned indices are chronological, sorted and unique.
    [[nodiscard]] RDVIOFrameRemovalPlan planRDVIOFrameRemovals(
        const Map &map,
        const RDVIOParameters &parameters);
}
