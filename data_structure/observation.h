//
// Created by Cain on 2025/9/17.
//

#ifndef VINSEKF_OBSERVATION_H
#define VINSEKF_OBSERVATION_H

#include "../type.h"

namespace slam {
    class Feature;

    class Observation {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void reset() {
            is_outlier = false;
            camera_id = 0;
            fet = nullptr;
            track_cnt = 0;
            pt.setZero();
            un_pt.setZero();
            field_speed.setZero();
        }

        CameraID camera_id{};
        Feature *fet;
        Count track_cnt{};
        Vec2  pt{};
        Vec3  un_pt{};
        Vec2  field_speed{};
        bool is_outlier{false};

    };
}

#endif //VINSEKF_OBSERVATION_H
