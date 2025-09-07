//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_FRAME_H
#define VINSEKF_FRAME_H

#include "../type.h"
#include "feature.h"

namespace slam {
//    using CameraMsg = std::array<cv::Mat, N_CAMERA>;
    using CameraMsg = std::array<void *, N_CAMERA>;
    using Landmark2FeatureMsg = std::unordered_map<LandmarkID, FeatureMsg *>;

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        enum STATE {
            QX = 0,
            QY,
            QZ,
            QW,
            PX,
            PY,
            PZ,
            VX,
            VY,
            VZ,
            BGX,
            BGY,
            BGZ,
            BAX,
            BAY,
            BAZ,
            GX,
            GY,
            GZ,

            DIM
        };

    public:
        Frame() { state[QW] = 1.; }
        explicit Frame(const std::array<TYPE, DIM> &state_init)
                : state(state_init) {}
        explicit Frame(const TYPE *state_init) { std::memcpy(state.data(), state_init, DIM * sizeof(TYPE)); }

        bool delete_landmark(LandmarkID lmk_id);

        void record_to_state_fej() { state_fej = state; }

        auto q() { return Eigen::Map<Quat>(state.data() + QX); }
        auto p() { return Eigen::Map<Vec3>(state.data() + PX); }
        auto v() { return Eigen::Map<Vec3>(state.data() + VX); }
        auto bg() { return Eigen::Map<Vec3>(state.data() + BGX); }
        auto ba() { return Eigen::Map<Vec3>(state.data() + BAX); }
        auto g() { return Eigen::Map<Vec3>(state.data() + GX); }

        [[nodiscard]] auto q() const { return Eigen::Map<const Quat>(state.data() + QX); }
        [[nodiscard]] auto p() const { return Eigen::Map<const Vec3>(state.data() + PX); }
        [[nodiscard]] auto v() const { return Eigen::Map<const Vec3>(state.data() + VX); }
        [[nodiscard]] auto bg() const { return Eigen::Map<const Vec3>(state.data() + BGX); }
        [[nodiscard]] auto ba() const { return Eigen::Map<const Vec3>(state.data() + BAX); }
        [[nodiscard]] auto g() const { return Eigen::Map<const Vec3>(state.data() + GX); }

    public:
        bool is_initialized{false};
        bool is_key_frame{false};

        Tus timestamp{};

        FrameID    id{};
        FrameOrder ordering{};

        Landmark2FeatureMsg lmk2fet;
        CameraMsg           cam2img;

        std::array<TYPE, DIM> state{};     // q, t, v, ba, bg, g
        std::array<TYPE, DIM> state_fej{}; // q, t, v, ba, bg, g
    };

    using KeyFrame = Frame;
} // namespace slam

#endif //VINSEKF_FRAME_H
