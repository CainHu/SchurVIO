//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_MAP_H
#define VINSEKF_MAP_H

#include "../type.h"
#include "feature.h"
#include "frame.h"
#include "landmark.h"
#include "pool.h"

namespace slam {
    using FrameSlidingWindow = std::deque<Frame *>;
    using FrameStream = std::list<Frame *>;
    using LandmarkMap = std::unordered_map<LandmarkID, Landmark *>;

    struct Map {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Map() = default;
        ~Map() = default;

        constexpr static size_t N_MSG = 100000;
        constexpr static size_t N_FET = 1000000;
        constexpr static size_t N_FRM = 1000;
        constexpr static size_t N_LMK = 100000;

        Pool<FeatureMsg> pool_msg{N_MSG};
        Pool<Feature> pool_fet{N_FET};
        Pool<Frame> pool_frm{N_FRM};
        Pool<Landmark> pool_lmk{N_LMK};

        std::array<Quat, N_CAMERA> q_ic;
        std::array<Vec3, N_CAMERA> t_ic;

        FrameSlidingWindow frm_slw;
        FrameStream frm_stm;
        LandmarkMap lmk_map;
    };
}

#endif //VINSEKF_MAP_H
