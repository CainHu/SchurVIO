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
            used_by_depth_free_rotation = false;
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
        // 该像素若先被无深度旋转因子消费，后续深度轨迹线性化应主动过滤它，
        // 而不是把这种有意分流记为“重复观测保护触发”。
        bool used_by_depth_free_rotation{false};

    };
}

#endif //VINSEKF_OBSERVATION_H
