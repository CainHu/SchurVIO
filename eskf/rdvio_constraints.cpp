#include "rdvio_constraints.h"

#include <algorithm>
#include <cmath>

namespace slam {
    RDVIOConstraintStatistics accumulateRDVIOConstraints(
        const Map &map,
        const std::vector<Landmark *> &rotation_only_tracks,
        const Mat3_3 &Ric,
        const bool add_zero_translation_constraint,
        const bool use_fej,
        const TYPE visual_batch_variance,
        const TYPE rotation_information_scale,
        const TYPE zero_translation_std,
        const TYPE hard_reprojection_limit,
        MatXX &Hpp,
        VecX &gp) {
        RDVIOConstraintStatistics statistics;

        if (add_zero_translation_constraint && map.sfw.size() >= 2) {
            // 零平移伪量测：r_t=0-(p_j-p_i)。当前误差状态的位置项采用加法，
            // 因而 J_i=[0,-I]、J_j=[0,I]。权重写成 visual_batch_variance/
            // sigma_t^2，是因为整个视觉正规方程最后统一按 visual_batch_variance
            // 解释为伪量测噪声；这样该因子的实际方差仍为 sigma_t^2。
            const Frame *previous = map.sfw[map.sfw.size() - 2];
            const Frame *current = map.sfw[map.sfw.size() - 1];
            const Vec3 residual = -(current->p() - previous->p());
            Eigen::Matrix<TYPE, 3, 6> J_previous =
                Eigen::Matrix<TYPE, 3, 6>::Zero();
            Eigen::Matrix<TYPE, 3, 6> J_current =
                Eigen::Matrix<TYPE, 3, 6>::Zero();
            J_previous.rightCols<3>() = -Mat3_3::Identity();
            J_current.rightCols<3>() = Mat3_3::Identity();
            const size_t previous_index = INSState::SIZE +
                AugState::SIZE * previous->ordering;
            const size_t current_index = INSState::SIZE +
                AugState::SIZE * current->ordering;
            const TYPE weight = visual_batch_variance /
                std::max(zero_translation_std * zero_translation_std,
                         TYPE(1e-12));

            Hpp.block<6, 6>(previous_index, previous_index)
                .triangularView<Eigen::Upper>() +=
                weight * J_previous.transpose() * J_previous;
            Hpp.block<6, 6>(current_index, current_index)
                .triangularView<Eigen::Upper>() +=
                weight * J_current.transpose() * J_current;
            if (previous_index < current_index) {
                Hpp.block<6, 6>(previous_index, current_index).noalias() +=
                    weight * J_previous.transpose() * J_current;
            } else {
                Hpp.block<6, 6>(current_index, previous_index).noalias() +=
                    weight * J_current.transpose() * J_previous;
            }
            gp.segment<6>(previous_index).noalias() +=
                weight * J_previous.transpose() * residual;
            gp.segment<6>(current_index).noalias() +=
                weight * J_current.transpose() * residual;
            ++statistics.zero_translation_constraints;
        }

        // 轨迹深度不可观时仍可约束相对旋转：
        //   d_w        = R_wc,i b_i
        //   b_j_hat    = R_wc,j^T d_w
        //   r_R        = B_j^T (b_j-b_j_hat)
        // 其中 B_j=[t_x,t_y] 是 b_j 的正交切平面基。投影到切平面后残差只有
        // 2 维，且不需要引入深度或执行 Schur 消元。
        for (Landmark *landmark : rotation_only_tracks) {
            Feature *from_feature = nullptr;
            Feature *to_feature = nullptr;
            Feature *previous_feature = nullptr;
            for (const auto &[frame_id, feature] : landmark->frm2fet) {
                (void)frame_id;
                if (previous_feature && feature && feature->frame &&
                    feature->frame->is_rotation_frame &&
                    previous_feature->obs[0] && feature->obs[0] &&
                    previous_feature->obs[0]->visual_update_count == 0 &&
                    feature->obs[0]->visual_update_count == 0) {
                    from_feature = previous_feature;
                    to_feature = feature;
                }
                previous_feature = feature;
            }
            if (!from_feature || !to_feature) {
                continue;
            }

            const Vec3 bearing_from = from_feature->obs[0]->un_pt.normalized();
            const Vec3 bearing_to = to_feature->obs[0]->un_pt.normalized();
            const Mat3_3 Rwc_from =
                from_feature->frame->q().toRotationMatrix() * Ric;
            const Mat3_3 Rwc_to =
                to_feature->frame->q().toRotationMatrix() * Ric;
            const Vec3 predicted_to =
                Rwc_to.transpose() * (Rwc_from * bearing_from);

            const Vec3 seed = std::abs(bearing_to.z()) < TYPE(0.9)
                ? Vec3::UnitZ() : Vec3::UnitY();
            const Vec3 tangent_x = bearing_to.cross(seed).normalized();
            const Vec3 tangent_y = bearing_to.cross(tangent_x).normalized();
            Eigen::Matrix<TYPE, 3, 2> tangent;
            tangent.col(0) = tangent_x;
            tangent.col(1) = tangent_y;
            const Vec2 residual =
                tangent.transpose() * (bearing_to - predicted_to);
            if (!residual.allFinite() ||
                residual.norm() > hard_reprojection_limit) {
                continue;
            }

            const Mat3_3 Rwc_from_jac =
                (use_fej
                    ? from_feature->frame->q_fej().toRotationMatrix()
                    : from_feature->frame->q().toRotationMatrix()) * Ric;
            const Mat3_3 Rwc_to_jac =
                (use_fej
                    ? to_feature->frame->q_fej().toRotationMatrix()
                    : to_feature->frame->q().toRotationMatrix()) * Ric;
            const Vec3 direction_world_jac = Rwc_from_jac * bearing_from;
            // 左乘姿态误差下，delta(R d)≈-hat(Rd) delta_theta；代入
            // b_j_hat=R_wc,j^T R_wc,i b_i，可得前后两帧姿态雅可比互为相反数。
            const Mat2_3 common = tangent.transpose() *
                Rwc_to_jac.transpose() * hat(direction_world_jac);
            Mat2_6 J_from = Mat2_6::Zero();
            Mat2_6 J_to = Mat2_6::Zero();
            J_from.leftCols<3>() = -common;
            J_to.leftCols<3>() = common;
            const size_t from_index = INSState::SIZE +
                AugState::SIZE * from_feature->frame->ordering;
            const size_t to_index = INSState::SIZE +
                AugState::SIZE * to_feature->frame->ordering;
            // 两个 bearing 都含像素噪声，差分残差近似具有 2 sigma_uv^2 方差，
            // 基础信息权重为 1/2。工程缩放 rotation_information_scale 用于吸收
            // R/N 误分类、IMU 旋转补偿误差和相邻 bearing 相关性；默认取保守值，
            // 避免一个启发式退化约束压过正常的多视图 Schur 信息。
            const TYPE weight = TYPE(0.5) *
                std::max(rotation_information_scale, TYPE(0));

            Hpp.block<6, 6>(from_index, from_index)
                .triangularView<Eigen::Upper>() +=
                weight * J_from.transpose() * J_from;
            Hpp.block<6, 6>(to_index, to_index)
                .triangularView<Eigen::Upper>() +=
                weight * J_to.transpose() * J_to;
            if (from_index < to_index) {
                Hpp.block<6, 6>(from_index, to_index).noalias() +=
                    weight * J_from.transpose() * J_to;
            } else {
                Hpp.block<6, 6>(to_index, from_index).noalias() +=
                    weight * J_to.transpose() * J_from;
            }
            gp.segment<6>(from_index).noalias() +=
                weight * J_from.transpose() * residual;
            gp.segment<6>(to_index).noalias() +=
                weight * J_to.transpose() * residual;

            ++from_feature->obs[0]->visual_update_count;
            ++to_feature->obs[0]->visual_update_count;
            from_feature->obs[0]->used_by_depth_free_rotation = true;
            to_feature->obs[0]->used_by_depth_free_rotation = true;
            statistics.observations_used += 2;
            statistics.new_observations += 2;
            ++statistics.tracks_used;
            ++statistics.rotation_constraints;
        }
        return statistics;
    }
}
