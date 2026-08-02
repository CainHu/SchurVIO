//
// Created by 许家仁 on 2025/8/26.
//

#ifndef VINSEKF_SCHUR_VINS_H
#define VINSEKF_SCHUR_VINS_H

#include "../common.h"
#include "../data_structure/map.h"
#include "frame_selection_policy.h"
#include "landmark_parameterization.h"
#include "visual_update_scheduler.h"

/*
 * 开发日志:
 * 1. 滑窗中，相邻两帧不能过小，不然会导致估计极度不准确
 * 2. 若仿真中的bg, ba真的为随机游走，估计的 vel 质量会很差
 * 3. 对 ekf 中的噪声 std 十分敏感
 * 4. 特征点若使用 Global Position 来参数化, Hll 的条件数会比较大，因为深度相比平移，更难估计
 * */

namespace slam {
    constexpr static auto operator""_s(unsigned long long s) {
        return static_cast<Tus>(s * 1000000);
    }

    constexpr static auto operator""_ms(unsigned long long ms) {
        return static_cast<Tus>(ms * 1000);
    }

    constexpr static auto operator""_us(unsigned long long us) {
        return static_cast<Tus>(us);
    }

    constexpr static auto operator""_s(long double s) {
        return static_cast<Tus>(s * 1e6);
    }

    constexpr static auto operator""_ms(long double ms) {
        return static_cast<Tus>(ms * 1e3);
    }

    constexpr static auto operator""_us(long double us) {
        return static_cast<Tus>(us);
    }

    constexpr static bool CONFIG_DEBUG = false;

    // Hpp 的分解方式(仅 USE_SCHUR 路径):
    //   false = SelfAdjointEigenSolver (Hpp = V·λ·V^T)
    //   true  = LDLT                   (Hpp = L·D·L^T)
    // 两者都能把 Cov[e] = σ²·Hpp 对角化，从而支持序贯更新，
    // 但用的是不同的基，结果不逐位相同(数值误差内应一致)。
    // 对比数据见 docs/OPT_LDLT.md。
    constexpr static bool USE_LDLT_FOR_HPP = true;

    // Hll (每个 landmark 的 3x3 块) 的分解方式，含义同上。
    // 注意 Hll 恒有 1 个接近 0 的特征值(深度/视线方向)，
    // 无论用哪种分解都必须做零空间过滤。详见 docs/HLL_STRUCTURE.md。
    constexpr static bool USE_LDLT_FOR_HLL = true;

    // 联合协方差的实现方式：true 显式构造 A；false 按 A 的分块结构传播，
    // CONFIG_DEBUG 和优化 else 均不会显式构造 A。
    constexpr static bool USE_STABLE_COVARIANCE_PREDICTION = false;
    constexpr static Tus IMU_TS = 5000;
    constexpr static Tus CAM_TS = 50000;

    class SchurVINS {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        enum class TriangulationStatus : uint8_t {
            Success = 0,
            InsufficientViews,
            LowParallax,
            IllConditioned,
            NegativeDepth,
            HighReprojectionError,
            ExcessiveUncertainty
        };

        enum class LandmarkInitializationMode : uint8_t {
            Triangulation = 0,
            TriangulationWithOraclePosition,
            GroundTruth
        };

        enum class LandmarkUpdateMode : uint8_t {
            // Keep the initialized point fixed while it remains in the map.
            Fixed = 0,
            // Historical path: independent landmark EKF without P_xl.
            IndependentEkf,
            // Independent EKF plus a fixed isotropic covariance diffusion.
            IndependentEkfInflated,
            // Independent EKF with innovation-adaptive covariance diffusion.
            IndependentEkfAdaptive,
            // Re-solve the point from all raw keyframe observations after a
            // keyframe pose update; failed re-triangulation keeps the old point.
            Retriangulate,
            // Back-substitute the joint Schur normal equation and accept only
            // a reprojection-cost-decreasing step. No independent covariance
            // accumulation and therefore no false persistent independence.
            SchurBackSubstitution
        };

        struct TriangulationLog {
            Tus timestamp{};
            LandmarkID id{};
            TriangulationStatus status{TriangulationStatus::InsufficientViews};
            size_t observation_count{};
            TYPE max_parallax_deg{};
            TYPE condition_number{};
            TYPE reprojection_rmse{};
            TYPE elapsed_us{};
            Vec3 initial_position{Vec3::Zero()};
            Vec3 latest_position{Vec3::Zero()};
            Mat3_3 initial_covariance{Mat3_3::Identity()};
            Mat3_3 latest_covariance{Mat3_3::Identity()};
            Vec3 shadow_position{Vec3::Zero()};
            Mat3_3 shadow_covariance{Mat3_3::Identity()};
            Vec3 ground_truth{Vec3::Zero()};
            TYPE initial_cov_trace{};
            TYPE initial_nees{};
            size_t refinement_count{};
            size_t shadow_refinement_count{};
            bool shadow_initialized{};
            bool has_ground_truth{};
        };

        struct TrackGeometryQuality {
            TYPE score{};
            TYPE max_parallax_deg{};
            TYPE condition_number{std::numeric_limits<TYPE>::infinity()};
            TYPE reprojection_rmse{std::numeric_limits<TYPE>::infinity()};
            TYPE position_std{std::numeric_limits<TYPE>::infinity()};
            size_t observation_count{};
            bool valid{};
            bool promotable{};
        };

        struct ShadowCandidateState {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            LandmarkID id{};
            Vec3 position{Vec3::Zero()};
            Mat3_3 covariance{Mat3_3::Identity()};
            TYPE quality_ema{};
            TYPE consistency_nis_ema{TYPE(1)};
            size_t stable_updates{};
            size_t observation_count{};
            Tus last_seen{};
        };

        // 已离开滑窗的低视差轨迹不能再参与导航更新，因为对应 clone 已被边缘化，
        // 其位姿误差与当前状态之间的相关性不能被一个固定 pose 快照替代。这里仅保存
        // 首尾两条射线，用于未来重新出现时检查“是否已经形成足够平移基线”。
        struct ArchivedBearingSnapshot {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Vec3 bearing_camera{Vec3::UnitZ()};
            Mat3_3 rotation_world_camera{Mat3_3::Identity()};
            Vec3 camera_center_world{Vec3::Zero()};
            Tus timestamp{};
        };

        struct DeferredTrackArchive {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            LandmarkID id{};
            ArchivedBearingSnapshot first;
            ArchivedBearingSnapshot last;
            TriangulationStatus last_status{TriangulationStatus::InsufficientViews};
            size_t observation_count{};
            TYPE archived_parallax_deg{};
            Tus archived_at{};
            Tus last_candidate_attempt{};
        };

        struct PersistentLandmarkState {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            LandmarkID id{};
            Vec3 position{Vec3::Zero()};
            Vec3 position_fej{Vec3::Zero()};
            TYPE promotion_quality{};
            Tus last_seen{};
            size_t update_count{};
            size_t rejected_count{};
        };

        explicit SchurVINS(slam::Map &map);

        // 处理IMU数据(预测步骤)
        void processIMU(const IMUData &imu_data);

        void processFrame(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map);

    void setQPV(const Quat &q, const Vec3 &p, const Vec3 &v);

    protected:
        // 状态预测
        void predict(const IMUData& imu_data, double dt);

        //
        void pushFrame(const CameraData &cam_data, bool is_keyframe);
        void popFrame(size_t chronological_index = 0);

        // 视觉更新
        void updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, double dt);

        // 独立影子地图后处理：只更新 shadow_position/shadow_cov_position，
        // 不改变导航后验使用的 Landmark::position，也不向 ESKF 反馈信息。
        void updateShadowLandmarks(
            const CameraData &cam_data,
            const std::vector<std::pair<LandmarkID, Landmark *>> &landmarks,
            bool is_keyframe,
            double dt);

        [[nodiscard]] bool isPersistentLandmark(LandmarkID id) const;
        [[nodiscard]] size_t persistentLandmarkOffset(size_t index) const;
        [[nodiscard]] TrackGeometryQuality evaluateTrackGeometry(
            const Landmark &landmark) const;
        [[nodiscard]] bool shouldDeferTrackConsumption(
            const Landmark &landmark,
            TriangulationStatus status,
            bool lost) const;
        void updateShadowCandidate(
            const Landmark &landmark,
            const TrackGeometryQuality &quality,
            Tus timestamp);
        void updateShadowCandidateEstimate(
            LandmarkID id,
            const Vec3 &position,
            const Mat3_3 &covariance,
            const TrackGeometryQuality &quality,
            Tus timestamp);
        void archiveDeferredTrack(
            const Landmark &landmark,
            TriangulationStatus status,
            Tus timestamp);
        void updateShadowCandidateFromArchive(
            LandmarkID id,
            const Frame &current_frame,
            const Vec2 &measurement,
            Tus timestamp);
        void pruneDeferredTrackArchives(Tus timestamp);
        [[nodiscard]] bool shadowCandidateReady(
            LandmarkID id,
            const TrackGeometryQuality &quality) const;
        bool promotePersistentLandmark(
            Landmark &landmark,
            const TrackGeometryQuality &quality,
            const Mat3_3 &hll,
            const Eigen::Matrix<TYPE, Eigen::Dynamic, 3> &hpl,
            const Vec3 &gl,
            const Mat3_3 &parameter_to_world,
            const VecX &navigation_increment,
            TYPE visual_variance,
            Tus timestamp);
        size_t updatePersistentLandmarks(
            const CameraData &cam_data,
            Frame *current_frame,
            bool is_keyframe);
        void applyJointStateCorrection(const VecX &dx);

        // 将误差状态注入名义状态。姿态采用左乘误差：R <- Exp(dtheta) R；
        // 位置、速度和零偏采用加法误差；随后每个 clone 按物理 ordering 注入。
        void updateState(const VecX &dx);

        struct TriangulationResult {
            TriangulationStatus status{TriangulationStatus::InsufficientViews};
            Vec3 position{Vec3::Zero()};
            Mat3_3 covariance{Mat3_3::Identity()};
            size_t observation_count{};
            TYPE max_parallax_deg{};
            TYPE condition_number{};
            TYPE reprojection_rmse{};
            TYPE elapsed_us{};
        };

        [[nodiscard]] TriangulationResult triangulateLandmark(const Landmark &landmark) const;
        void logTriangulationAttempt(Landmark &landmark,
                                     const TriangulationResult &result,
                                     Tus timestamp,
                                     const std::unordered_map<size_t, Vec3> &ground_truth);
        void recordLandmarkRefinement(const Landmark &landmark, bool shadow = false);

//    private:
    public:
        INSState state_;

        Vec3 gyro_corr_;
        Vec3 accel_corr_;
        Vec3 accel_corr_world_;
        Mat3_3 Rnb_;
        Vec3 a_world_;

        IMUData imu_data_last_;
        CameraData cam_data_last_;

        Tus imu_ts_{IMU_TS};
        Tus cam_ts_{CAM_TS};

        constexpr static size_t WIN_SIZE = Map::N_WIN;
        constexpr static size_t COV_SIZE = INSState::SIZE + WIN_SIZE * AugState::SIZE;
//        size_t latest_free_sfw_idx_{0};
//        std::vector<size_t> free_sfw_idx_;
//        std::vector<std::pair<AugState, CameraData>> sfw_;
//        std::unordered_map<size_t, LmkState> lmk_;
        Eigen::MatrixXd cov_;

        ExtState ext_;

        constexpr static size_t LMK_SIZE = 3;
        constexpr static LandmarkParameterization landmark_parameterization =
            LANDMARK_PARAMETERIZATION;
        constexpr static VisualUpdateScheduler visual_update_scheduler =
            VISUAL_UPDATE_SCHEDULER;
        constexpr static FrameSelectionPolicy frame_selection_policy =
            FRAME_SELECTION_POLICY;
        static_assert(framePolicyRetainedCloneCount(frame_selection_policy) < WIN_SIZE,
                      "frame policy must leave one slot for the incoming clone");

        // Schur 序贯伪量测的噪声密度。更新中使用 R_i=uv_var/(d_i*dt)，
        // 因此它不是像素方差。30 s / 600 点四场景长时扫描后取 1e-2：
        // 1e-4 在滑窗充分运行后会放大线性化/gauge 漂移，1e-2 的最坏误差更稳健。
        TYPE uv_var = TYPE(1e-2);
        // MSCKF 一次性轨迹对应真实像素样本批次，其协方差为
        // triangulation_uv_std^2 乘以该鲁棒缩放，不按相机 dt 重复积分。
        TYPE msckf_visual_noise_scale = TYPE(1);
        // RD-VIO 风格判定：公共特征经 IMU 旋转补偿后的角误差 70% 分位数
        // 小于该阈值时，将当前帧标为旋转主导 R 帧。
        TYPE rdvio_rotation_threshold_deg = TYPE(0.60);
        size_t rdvio_min_common_tracks = 20;
        size_t rdvio_subframe_size = 3;
        size_t rdvio_rotation_compression_trigger = 9;
        TYPE rdvio_zero_translation_std = TYPE(0.03);
        TYPE vins_mono_keyframe_parallax_deg = TYPE(1.25);
        size_t vins_mono_min_common_tracks = 20;
        Tus vins_mono_max_keyframe_interval_us = 1000000;
        // 过程噪声整体缩放因子(1.0 = 使用 INSState 中配置的原值)，用于敏感度扫描
        TYPE proc_noise_scale_ = TYPE(1);
        // 严格消融开关。生产默认使用真实三角化与 Landmark 修正；真值初始化
        // 只允许分析程序使用，不能进入默认算法。
        LandmarkInitializationMode landmark_initialization_mode_ =
            LandmarkInitializationMode::Triangulation;
        LandmarkUpdateMode landmark_update_mode_ = LandmarkUpdateMode::Retriangulate;
        // 历史消融命令兼容开关；false 时无条件使用 Fixed，不读取 update mode。
        bool refine_landmarks_ = true;
        // 基于 FEJ 的可观性约束，保护 VIO 的四维 gauge：全局平移 3 维和绕
        // 重力方向的全局偏航 1 维。默认开启，仅在严格 A/B 中允许关闭。
        bool enforce_observability_constraint_ = true;
        // 可选的先验白化硬投影。Hll 伪逆、投影后的 Hpp 和 gp 必须共享同一
        // 有效子空间；生产默认仍是 FEJ，该路径只用于显式开启的实验。
        bool project_observability_constraint_ = false;
        TYPE hll_rank_relative_threshold_ = TYPE(1e-8);
        TYPE hpp_rank_relative_threshold_ = TYPE(1e-6);

        // 独立地图协方差实验参数。固定项单位为 m^2/s，每次视觉更新按 dt 积分；
        // 自适应模式再由每个 Landmark 的归一化创新 EMA 调节膨胀量。
        TYPE landmark_process_noise_density_ = TYPE(1e-3);
        TYPE landmark_adaptive_inflation_gain_ = TYPE(1);
        TYPE landmark_adaptive_inflation_max_scale_ = TYPE(25);
        TYPE landmark_nis_ema_alpha_ = TYPE(0.05);

        // 影子 Landmark：只消费最新关键帧观测，且永不修改 ESKF 构造残差时
        // 使用的 Landmark::position。默认关闭，不增加生产路径计算量。
        bool enable_shadow_landmark_postprocessor_ = false;
        bool shadow_landmark_adaptive_inflation_ = true;
        // 默认混合后端：普通轨迹仍走一次性 MSCKF，只有通过影子候选稳定性检查的
        // 少量点才进入联合状态，并完整维护 P_xl/P_ll。
        bool enable_hybrid_persistent_landmarks_ = true;
        // 默认 MSCKF 也启用无深度旋转约束，但不采用 RD-VIO 的窗口调度。
        // 对低视差轨迹使用球面切平面残差，只约束相邻 clone 的相对旋转。
        bool enable_depth_free_rotation_constraints_ = true;
        TYPE depth_free_rotation_information_scale_ = TYPE(0.02);
        size_t persistent_landmark_budget_ = 20;
        size_t shadow_candidate_capacity_ = 200;
        size_t shadow_candidate_min_stable_updates_ = 2;
        size_t deferred_track_max_frames_ = 80;
        size_t deferred_track_archive_capacity_ = 400;
        Tus deferred_track_archive_max_age_us_ = 15000000;
        Tus deferred_track_archive_retry_interval_us_ = 250000;
        size_t persistent_grid_columns_ = 4;
        size_t persistent_grid_rows_ = 3;
        size_t persistent_grid_cell_quota_ = 2;
        TYPE persistent_min_geometry_score_ = TYPE(0.68);
        TYPE persistent_max_position_std_ = TYPE(3.0);
        TYPE persistent_update_chi2_threshold_ = TYPE(9.21);
        // 持久点会跨很多关键帧重复观测。完整联合协方差消除了“把旧地图当独立量测”
        // 的主要重复计数，但 FEJ 长期线性化误差、前端时间相关性和未建模地图过程噪声
        // 仍会使理想像素方差过于乐观。100 s 五场景扫描选用 64 倍方差：所有场景
        // 均不劣于原始 MSCKF，且比直接使用原始像素方差更稳定。
        TYPE persistent_measurement_noise_scale_ = TYPE(64);
        TYPE shadow_candidate_nis_threshold_ = TYPE(11.34);
        TYPE shadow_candidate_ema_alpha_ = TYPE(0.25);
        constexpr static TYPE lmk_var = TYPE(0.01);

        // 三角化使用归一化像平面噪声；仿真中约为 1 pixel / fx = 0.0054。
        // 它与历史视觉后验中的 uv_var（聚合伪量测噪声）含义不同，不能直接复用 400。
        TYPE triangulation_uv_std = TYPE(0.0054);
        TYPE triangulation_min_parallax_deg =
            TYPE(schedulerDefaultTriangulationParallaxDeg());
        TYPE triangulation_max_reprojection_rmse = TYPE(0.03);
        TYPE triangulation_max_position_std = TYPE(50);

        // Schur 重投影残差采用 Huber 权重；阈值以归一化像平面标准差为单位。
        TYPE visual_huber_delta_sigma = TYPE(3);
        // 超过该归一化残差的观测视为明显错误，避免错误深度/关联造成灾难性更新。
        TYPE visual_hard_reprojection_limit = TYPE(0.1);

        // ---- 数据采集(用于可视化/分析，见 tools/) ----
        struct UpdateLog {
            Tus timestamp{};
            // 视觉更新【前】(先验)与【后】(后验)的状态，用于看修正作用
            Vec3 p_prior{Vec3::Zero()}, p_post{Vec3::Zero()};
            Vec3 v_prior{Vec3::Zero()}, v_post{Vec3::Zero()};
            Quat q_prior{Quat::Identity()}, q_post{Quat::Identity()};
            Vec3 bg_post{Vec3::Zero()}, ba_post{Vec3::Zero()}, g_post{Vec3::Zero()};
            // 本次视觉更新施加的修正量范数
            TYPE dx_p_norm{}, dx_q_norm{}, dx_v_norm{};
            // 协方差(位置/姿态/速度的 trace，开根号得米/弧度量级)
            TYPE cov_p_trace{}, cov_q_trace{}, cov_v_trace{};
            // Schur 序贯伪量测的归一化创新平方统计；均值理论期望约为 1。
            TYPE nis_mean{};
            size_t nis_dof{};
            size_t n_lmk{};      // 参与本次更新的 landmark 数
            size_t n_obs_used{};
            size_t n_obs_downweighted{};
            size_t n_obs_rejected{};
            size_t n_obs_new{};
            size_t n_obs_reused{};
            size_t n_tracks_consumed{};
            size_t win_size{};   // 滑窗帧数
            bool is_keyframe{};
            TYPE oc_leak_before{};
            TYPE oc_leak_after{};
            bool is_rotation_frame{};
            uint8_t rdvio_case{};
            TYPE rdvio_misalignment_deg{};
            size_t rotation_only_constraints{};
        };
        std::vector<UpdateLog> logs_;
        std::vector<TriangulationLog> triangulation_logs_;
        bool enable_logging_ = false;

        Eigen::VectorXd Rll_;

        slam::Map &map_;
        std::unordered_map<LandmarkID, ShadowCandidateState> shadow_candidates_;
        std::unordered_map<LandmarkID, DeferredTrackArchive> deferred_track_archives_;
        std::vector<PersistentLandmarkState> persistent_landmarks_;
        std::unordered_map<LandmarkID, size_t> persistent_landmark_indices_;

        size_t posterior_times_ = 0;
        size_t t_cost_ = 0;
        size_t t_refine_cost_ = 0;
        size_t n_lmk_total_ = 0;
        size_t t_perlmk_qr_ = 0;   // 每个 landmark 的小 QR (2K x 3)
        size_t t_bigqr_ = 0;       // J_STATE 的大 QR
        size_t t_seq_state_ = 0;   // 序贯更新 state (198x198 协方差)
        size_t t_lmk_update_ = 0;  // 更新 landmark 位置
        size_t n_seq_rows_ = 0;    // 序贯更新的行数

        // USE_SCHUR 路径
        size_t t_build_H_ = 0;     // 构建 Hpp/Hpl/Hll
        size_t t_schur_ = 0;       // Schur 补
        size_t t_eig_state_ = 0;   // Hpp 特征分解 + 序贯更新 state
        size_t t_eig_lmk_ = 0;     // Hll 特征分解 + 更新 landmark
        size_t t_eig_decomp_ = 0;  // 仅 Hpp 的分解(特征分解或 LDLT)
        size_t n_skipped_ = 0;     // 被判定为零空间而跳过的方向数
        size_t n_negative_ = 0;    // 对角元严格为负的方向数(Hpp 不定)
        size_t n_oc_projections_ = 0;
        TYPE oc_max_leak_before_ = TYPE(0);
        TYPE oc_max_leak_after_ = TYPE(0);
        size_t n_hll_rank_tests_ = 0;
        size_t n_hll_discarded_directions_ = 0;
        TYPE hll_discarded_gradient_ratio_sum_ = TYPE(0);
        TYPE hll_discarded_gradient_ratio_max_ = TYPE(0);
        size_t n_hll_condition_tests_ = 0;
        TYPE hll_effective_condition_sum_ = TYPE(0);
        TYPE hll_effective_condition_max_ = TYPE(0);
        size_t n_hpp_rank_tests_ = 0;
        size_t n_hpp_discarded_directions_ = 0;
        TYPE hpp_discarded_gradient_ratio_sum_ = TYPE(0);
        TYPE hpp_discarded_gradient_ratio_max_ = TYPE(0);
        size_t n_lmk_update_attempts_ = 0;
        size_t n_lmk_update_accepted_ = 0;
        size_t n_lmk_retriangulation_success_ = 0;
        TYPE lmk_reprojection_cost_reduction_ = TYPE(0);
        // Visual measurement lifecycle diagnostics. In the recommended MSCKF
        // scheduler n_reused_observations_ must remain exactly zero.
        size_t n_new_observations_ = 0;
        size_t n_reused_observations_ = 0;
        size_t n_tracks_consumed_ = 0;
        size_t n_tracks_dropped_ = 0;
        size_t n_visual_updates_skipped_ = 0;
        // Defensive guard: a reused MSCKF sample is rejected before H/g.
        // This counter and n_reused_observations_ must both remain zero.
        size_t n_duplicate_observations_blocked_ = 0;
        size_t n_tracks_deferred_ = 0;
        size_t n_track_archives_created_ = 0;
        size_t n_track_archives_reused_ = 0;
        size_t n_track_archives_rejected_ = 0;
        size_t n_track_archives_expired_ = 0;
        size_t n_shadow_candidate_updates_ = 0;
        size_t n_shadow_candidate_rejections_ = 0;
        size_t n_persistent_landmarks_promoted_ = 0;
        size_t n_persistent_landmark_updates_ = 0;
        size_t n_persistent_landmark_rejections_ = 0;
        size_t n_keyframes_selected_ = 0;
        size_t n_nonkeyframes_selected_ = 0;
        size_t n_frames_stored_ = 0;
        size_t n_rdvio_rotation_frames_ = 0;
        size_t n_rdvio_normal_frames_ = 0;
        std::array<size_t, 5> n_rdvio_cases_{};
        size_t n_rdvio_compressed_frames_ = 0;
        size_t n_rdvio_rotation_constraints_ = 0;
        size_t n_rdvio_zero_translation_constraints_ = 0;
        size_t n_rotation_dominant_frames_ = 0;
        size_t n_translation_dominant_frames_ = 0;
        size_t n_depth_free_rotation_constraints_ = 0;
    };
}

#endif //VINSEKF_SCHUR_VINS_H
