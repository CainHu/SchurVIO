#pragma once

#include "../common.h"
#include "../data_structure/landmark.h"

#include <cstdint>

// Three-degree-of-freedom landmark coordinates used by the Schur visual
// linearization.  The stored map value remains a world-frame XYZ point; only
// the local optimization coordinates and their Jacobians change.
#define SCHUR_VIO_LMK_WORLD_XYZ 0
#define SCHUR_VIO_LMK_ANCHORED_XYZ 1
#define SCHUR_VIO_LMK_ANCHORED_INV_DEPTH 2
#define SCHUR_VIO_LMK_ANCHORED_LOG_DEPTH 3

#ifndef SCHUR_VIO_LANDMARK_PARAMETERIZATION
#define SCHUR_VIO_LANDMARK_PARAMETERIZATION SCHUR_VIO_LMK_WORLD_XYZ
#endif

static_assert(SCHUR_VIO_LANDMARK_PARAMETERIZATION >= SCHUR_VIO_LMK_WORLD_XYZ &&
              SCHUR_VIO_LANDMARK_PARAMETERIZATION <= SCHUR_VIO_LMK_ANCHORED_LOG_DEPTH,
              "Unknown SCHUR_VIO_LANDMARK_PARAMETERIZATION value");

namespace slam {
    enum class LandmarkParameterization : uint8_t {
        WorldXYZ = SCHUR_VIO_LMK_WORLD_XYZ,
        AnchoredXYZ = SCHUR_VIO_LMK_ANCHORED_XYZ,
        AnchoredInverseDepth = SCHUR_VIO_LMK_ANCHORED_INV_DEPTH,
        AnchoredLogDepth = SCHUR_VIO_LMK_ANCHORED_LOG_DEPTH
    };

    constexpr LandmarkParameterization LANDMARK_PARAMETERIZATION =
        static_cast<LandmarkParameterization>(SCHUR_VIO_LANDMARK_PARAMETERIZATION);

    [[nodiscard]] constexpr const char *landmarkParameterizationName(
        const LandmarkParameterization mode = LANDMARK_PARAMETERIZATION) {
        switch (mode) {
            case LandmarkParameterization::WorldXYZ: return "world_xyz";
            case LandmarkParameterization::AnchoredXYZ: return "anchored_xyz";
            case LandmarkParameterization::AnchoredInverseDepth: return "anchored_inv_depth_3d";
            case LandmarkParameterization::AnchoredLogDepth: return "anchored_log_depth_3d";
        }
        return "unknown";
    }

    [[nodiscard]] constexpr bool isAnchoredLandmarkParameterization(
        const LandmarkParameterization mode = LANDMARK_PARAMETERIZATION) {
        return mode != LandmarkParameterization::WorldXYZ;
    }

    struct LandmarkParameterizationLinearization {
        Mat3_3 parameter_to_world{Mat3_3::Identity()};
        Frame *anchor{};
        bool valid{true};
    };

    // Build d(point_world)/d(local_parameter) at the current linearization
    // point. Landmark::position remains world XYZ for every mode.
    [[nodiscard]] LandmarkParameterizationLinearization
    linearizeLandmarkParameterization(
        Landmark &landmark,
        const Mat3_3 &Ric,
        const Vec3 &t_ic,
        bool use_fej,
        LandmarkParameterization mode = LANDMARK_PARAMETERIZATION);

    // Anchor-pose contribution induced by an anchored 3-DoF landmark.
    [[nodiscard]] Mat2_6 landmarkAnchorPoseJacobian(
        const Mat2_3 &J_landmark_world,
        const Landmark &landmark,
        const Frame &anchor,
        bool use_fej);

    // Convert a normal equation expressed in local landmark coordinates back
    // to the persistent world-XYZ covariance basis.
    [[nodiscard]] bool transformLandmarkNormalToWorld(
        const Mat3_3 &parameter_to_world,
        Mat3_3 &Hll,
        Vec3 &gradient);
}
