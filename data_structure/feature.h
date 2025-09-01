//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_FEATURE_H
#define VINSEKF_FEATURE_H

#include "../type.h"

namespace slam {
    class Landmark;
    class Frame;
    class Feature;

    using FeatureMsg = std::array<Feature *, N_CAMERA>;

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        bool is_outlier{false};

        LandmarkID landmark_id{};
        CameraID   camera_id{};

        Count track_cnt{};
        Vec2  pt{};
        Vec3  un_pt{};
        Vec2  field_speed{};

        Landmark *landmark{};
        Frame    *frame{};
    };
}

#endif //VINSEKF_FEATURE_H
