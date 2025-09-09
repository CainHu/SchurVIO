//
// Created by 许家仁 on 2025/8/23.
//

#include "frame.h"

using namespace slam;
bool Frame::delete_landmark(LandmarkID lmk_id) {
    auto it = lmk2msg.find(lmk_id);
    if (it == lmk2msg.end()) {
        return false;
    }

    lmk2msg.erase(it);

    return true;
}