//
// Created by 许家仁 on 2025/8/23.
//

#include "map.h"

namespace slam {
    Landmark *Map::addLandmark(const LandmarkID id) {
        Landmark *pt {nullptr};
        if (auto it = lmk_map.find(id); it == lmk_map.end()) {
            pt = pool_lmk.allocate();
            lmk_map.emplace(id, pt);
        }
        return pt;
    }
}