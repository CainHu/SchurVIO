//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_LANDMARK_H
#define VINSEKF_LANDMARK_H

#include "../type.h"
#include "feature.h"
#include "frame.h"
#include <limits>

namespace slam {
    using Frame2FeatureMsg = std::map<FrameID, Feature *>;

    class Map;

    struct Landmark {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Landmark() = default;
        explicit Landmark(Map *common_map) : map(common_map) {}
        explicit Landmark(LandmarkID landmark_id, Map *common_map)
                : id(landmark_id), map(common_map) {}

        bool delete_frame(FrameID frame_id);

        bool pos2inv(const std::optional<Quat>& q_ic=std::nullopt, const std::optional<Vec3>& t_ic=std::nullopt);
        bool inv2pos(const std::optional<Quat>& q_ic=std::nullopt, const std::optional<Vec3>& t_ic=std::nullopt);

        [[nodiscard]] bool is_valid() const { return is_triangulated && !is_outlier && anchor_obs; }

        void reset() {
            is_triangulated = false;
            is_outlier = false;

            id = 0;
            inv_depth = TYPE(0);
            position.setZero();

            var_ins_depth = TYPE(1);
            cov_position = Mat3_3::Identity() * TYPE(1e-4);
            independent_nis_ema = TYPE(1);

            shadow_initialized = false;
            shadow_position.setZero();
            shadow_cov_position = Mat3_3::Identity() * TYPE(1e-4);
            shadow_nis_ema = TYPE(1);

            last_triangulation_obs_count = 0;
            triangulation_log_index = std::numeric_limits<size_t>::max();

            anchor_obs = nullptr;
            frm2fet.clear();

            map = nullptr;
        }

        bool is_triangulated{false};
        bool is_outlier{false};

        LandmarkID id{};

        TYPE inv_depth{};
        Vec3 position{};

        TYPE var_ins_depth{TYPE(1)};
        Mat3_3 cov_position{Mat3_3 ::Identity() * 1e-4};

        // The historical independent landmark EKF intentionally omits P_xl.
        // Keep its consistency state with the landmark so covariance inflation
        // can react to the innovation history without changing the ESKF state.
        TYPE independent_nis_ema{TYPE(1)};

        // Detached map post-processor.  This estimate is never used to build
        // the ESKF visual residual, so an optimistic map covariance cannot feed
        // back into the navigation posterior.
        bool shadow_initialized{false};
        Vec3 shadow_position{Vec3::Zero()};
        Mat3_3 shadow_cov_position{Mat3_3::Identity() * TYPE(1e-4)};
        TYPE shadow_nis_ema{TYPE(1)};

        // 三角化失败后，仅在关键帧观测数增加时重试，避免非关键帧更新反复做无效计算。
        size_t last_triangulation_obs_count{};
        // 指向 SchurVINS 中成功初始化记录；landmark 被移出滑窗后记录仍可用于离线评估。
        size_t triangulation_log_index{std::numeric_limits<size_t>::max()};

        Observation *anchor_obs{};
        Frame2FeatureMsg frm2fet;

        Map *map{};
    };
}

#endif //VINSEKF_LANDMARK_H
