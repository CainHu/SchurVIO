#pragma once

#include "../common.h"
#include "../data_structure/map.h"

namespace slam {
    struct RDVIOConstraintStatistics {
        size_t observations_used{};
        size_t new_observations{};
        size_t tracks_used{};
        size_t rotation_constraints{};
        size_t zero_translation_constraints{};
    };

    /**
     * 把与 Landmark 深度无关的约束直接累加到位姿正规方程 Hpp/gp。
     *
     * 对相邻两帧单位 bearing b_i、b_j，静态点在纯旋转条件下满足：
     *   b_j = R_cj,w R_w,ci b_i。
     * 用 b_j 的二维切平面基 B_j 消去单位球面法向分量：
     *   r_R = B_j^T (b_j-R_cj,w R_w,ci b_i)。
     * 该残差不含点深度，因此低视差轨迹也能约束相对姿态。
     *
     * 若 add_zero_translation_constraint=true，还会添加
     *   r_t = -(p_j-p_i)
     * 的零平移伪量测。该项只属于显式 RD-VIO 模式；默认 MSCKF 仅复用旋转残差。
     * 每条轨迹只选择最近一个尚未消费、且终点被判定为 R 帧的相邻帧对。
     */
    [[nodiscard]] RDVIOConstraintStatistics accumulateRDVIOConstraints(
        const Map &map,
        const std::vector<Landmark *> &rotation_only_tracks,
        const Mat3_3 &Ric,
        bool add_zero_translation_constraint,
        bool use_fej,
        TYPE visual_batch_variance,
        TYPE rotation_information_scale,
        TYPE zero_translation_std,
        TYPE hard_reprojection_limit,
        MatXX &Hpp,
        VecX &gp);
}
