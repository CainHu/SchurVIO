//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_FEATURE_H
#define VINSEKF_FEATURE_H

#include "../type.h"
#include "observation.h"

namespace slam {
    class Landmark;
    class Frame;
    class Feature;

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        void reset() {
            for (auto &o : obs) {
                o = nullptr;
            }

            landmark = nullptr;
            frame = nullptr;
        }

        Landmark *landmark{};
        Frame    *frame{};
        std::array<Observation*, N_CAMERA> obs{};
    };
}

#endif //VINSEKF_FEATURE_H
