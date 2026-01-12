//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_LANDMARK_H
#define VINSEKF_LANDMARK_H

#include "../type.h"
#include "feature.h"
#include "frame.h"

namespace slam {
    using Frame2FeatureMsg = std::map<FrameID, Feature *>;

    class Map;

    struct Landmark {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Landmark() = default;
        explicit Landmark(Map *common_map) : map(common_map) {}
        explicit Landmark(LandmarkID landmark_id, Map *common_map)
                : id(landmark_id), map(common_map) {}

        bool delete_frame(FrameID frame_id);

        bool pos2inv(const std::optional<Quat>& q_ic=std::nullopt, const std::optional<Vec3>& t_ic=std::nullopt);
        bool inv2pos(const std::optional<Quat>& q_ic=std::nullopt, const std::optional<Vec3>& t_ic=std::nullopt);

        [[nodiscard]] bool is_valid() const { return is_triangulated && !is_outlier && anchor_obs; }

        void reset() {
            is_triangulated = false;
            is_outlier = false;

            id = 0;
            inv_depth = TYPE(0);
            position.setZero();

            var_ins_depth = TYPE(1);
            cov_position.setIdentity();

            anchor_obs = nullptr;
            frm2fet.clear();

            map = nullptr;
        }

        bool is_triangulated{false};
        bool is_outlier{false};

        LandmarkID id{};

        TYPE inv_depth{};
        Vec3 position{};

        TYPE var_ins_depth{TYPE(1)};
        Mat3_3 cov_position{Mat3_3 ::Identity() * 1e-4};

        Observation *anchor_obs{};
        Frame2FeatureMsg frm2fet;

        Map *map{};
    };
}

#endif //VINSEKF_LANDMARK_H
