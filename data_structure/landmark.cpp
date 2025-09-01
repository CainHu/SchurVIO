//
// Created by 许家仁 on 2025/8/23.
//

#include "landmark.h"
#include "map.h"

using namespace slam;

bool Landmark::delete_frame(FrameID frame_id) {
    auto iter = frm2fet.find(frame_id);
    if (iter == frm2fet.end()) {
        return false;
    }

    frm2fet.erase(iter);

    if (frm2fet.empty()) {
        anchor_fet = nullptr;
        return true;
    }

    // TODO: update anchor_fet

    return true;
}

bool Landmark::pos2inv(const std::optional<Quat>& q_ic, const std::optional<Vec3>& t_ic) {
    if (!is_valid()) {
        return false;
    }

    const auto camera_id = anchor_fet->camera_id;
    const auto &lmk_un_pt = anchor_fet->un_pt;
    const auto frame = anchor_fet->frame;
    const Vec3 lmk_imu = frame->q().inverse() * (position - frame->p());
    inv_depth = lmk_un_pt.squaredNorm() / lmk_un_pt.dot(
            (q_ic.has_value() ? q_ic.value().inverse() : map->q_ic[camera_id])
                  * (lmk_imu - (t_ic.has_value() ? t_ic.value() : map->t_ic[camera_id])));

    return true;
}

bool Landmark::inv2pos(const std::optional<Quat>& q_ic, const std::optional<Vec3>& t_ic) {
    if (!is_valid()) {
        return false;
    }

    const auto camera_id = anchor_fet->camera_id;
    const auto &lmk_un_pt = anchor_fet->un_pt;
    const auto frame = anchor_fet->frame;
    const Vec3 lmk_imu = (q_ic.has_value() ? q_ic.value() : map->q_ic[camera_id]) * (lmk_un_pt / inv_depth)
            + (t_ic.has_value() ? t_ic.value() : map->t_ic[camera_id]);
    position = frame->q() * lmk_imu + frame->p();

    return true;
}