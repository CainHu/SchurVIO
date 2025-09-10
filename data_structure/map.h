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
#include "sliding_window.h"

namespace slam {
    using FrameDeque = std::deque<Frame *>;
    using FrameList = std::list<Frame *>;
    using FrameVector = std::vector<Frame *>;
    using LandmarkMap = std::unordered_map<LandmarkID, Landmark *>;

    struct Map {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Map() = default;
        ~Map() = default;

        Landmark *addLandmark(LandmarkID id);

        constexpr static size_t N_WIN = 7;
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

        FrameDeque frm_deq;
        FrameList frm_lst;
        FrameVector frm_vec;

        SlidingWindow sfw {N_WIN};
        LandmarkMap lmk_map;
    };
}

#endif //VINSEKF_MAP_H
