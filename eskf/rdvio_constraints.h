#pragma once

#include "../common.h"
#include "../data_structure/map.h"

namespace slam {
    struct RDVIOConstraintStatistics {
        size_t observations_used{};
        size_t new_observations{};
        size_t tracks_used{};
        size_t rotation_constraints{};
        size_t zero_translation_constraints{};
    };

    // Add RD-VIO-only factors directly to the pose normal equation. Rotation
    // factors are depth-free and consume one recent R transition per track.
    [[nodiscard]] RDVIOConstraintStatistics accumulateRDVIOConstraints(
        const Map &map,
        const std::vector<Landmark *> &rotation_only_tracks,
        const Mat3_3 &Ric,
        bool add_zero_translation_constraint,
        bool use_fej,
        TYPE visual_batch_variance,
        TYPE zero_translation_std,
        TYPE hard_reprojection_limit,
        MatXX &Hpp,
        VecX &gp);
}
