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
    using Landmark2FeatureMsg = std::unordered_map<LandmarkID, Feature *>;

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

        [[nodiscard]] auto q_fej() const { return Eigen::Map<const Quat>(state_fej.data() + QX); }
        [[nodiscard]] auto p_fej() const { return Eigen::Map<const Vec3>(state_fej.data() + PX); }

        void reset() {
            is_initialized = false;
            is_key_frame = false;
            is_rotation_frame = false;
            rdvio_case = 0;
            rdvio_misalignment_deg = TYPE(0);

            timestamp = 0;
            id = 0;
            ordering = 0;

            lmk2fet.clear();
//            cam2img

            memset(state.data(), 0, state.size() * sizeof(TYPE));
            state[QW] = 1.;
            state_fej = state;

            cov.setIdentity();
        }

    public:
        bool is_initialized{false};
        bool is_key_frame{false};
        bool is_rotation_frame{false};
        // 0=None, 1=RR, 2=NN, 3=RN, 4=NR.
        uint8_t rdvio_case{};
        TYPE rdvio_misalignment_deg{};

        Tus timestamp{};

        FrameID    id{};
        FrameOrder ordering{};

        Landmark2FeatureMsg lmk2fet;
        CameraMsg           cam2img{};

        std::array<TYPE, DIM> state{};     // q, t, v, ba, bg, g
        std::array<TYPE, DIM> state_fej{}; // q, t, v, ba, bg, g
        Eigen::Matrix<TYPE, DIM, DIM> cov{Eigen::Matrix<TYPE, DIM, DIM>::Zero()};
    };

    using KeyFrame = Frame;
} // namespace slam

#endif //VINSEKF_FRAME_H
