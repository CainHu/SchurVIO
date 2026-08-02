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
            visual_update_count = 0;
        }

        CameraID camera_id{};
        Feature *fet{};
        Count track_cnt{};
        Vec2  pt{Vec2::Zero()};
        Vec3  un_pt{Vec3::Zero()};
        Vec2  field_speed{Vec2::Zero()};
        bool is_outlier{false};
        // Number of posterior batches that consumed this exact pixel sample.
        // It must stay at zero before an MSCKF one-shot track update.
        uint16_t visual_update_count{};

    };
}

#endif //VINSEKF_OBSERVATION_H
