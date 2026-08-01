//
// Created by 许家仁 on 2025/8/26.
//

#ifndef VINSEKF_SCHUR_VINS_H
#define VINSEKF_SCHUR_VINS_H

#include "../common.h"
#include "../data_structure/map.h"

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
            Vec3 ground_truth{Vec3::Zero()};
            TYPE initial_cov_trace{};
            TYPE initial_nees{};
            size_t refinement_count{};
            bool has_ground_truth{};
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
        void popFrame();

        // 更新 map
        void updateMap(const CameraData &cam_data);

        // 视觉更新
        void updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, double dt);

        void updateState(auto &&dx);

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
        void recordLandmarkRefinement(const Landmark &landmark);

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

        // Schur 序贯伪量测的噪声密度。更新中使用 R_i=uv_var/(d_i*dt)，
        // 因此它不是像素方差。30 s / 600 点四场景长时扫描后取 1e-2：
        // 1e-4 在滑窗充分运行后会放大线性化/gauge 漂移，1e-2 的最坏误差更稳健。
        TYPE uv_var = TYPE(1e-2);
        // 过程噪声整体缩放因子(1.0 = 使用 INSState 中配置的原值)，用于敏感度扫描
        TYPE proc_noise_scale_ = TYPE(1);
        // Strict-ablation switches. Production defaults keep real triangulation and
        // landmark refinement enabled; GT initialization is analysis-only.
        LandmarkInitializationMode landmark_initialization_mode_ =
            LandmarkInitializationMode::Triangulation;
        bool refine_landmarks_ = true;
        // FEJ-based observability constraint for the Schur visual update.
        // It preserves the four VIO gauge directions: global translation (3)
        // and global yaw about gravity (1). Enabled by default; analysis can
        // disable it for a controlled A/B comparison.
        bool enforce_observability_constraint_ = true;
        // Optional hard projection of the Schur-reduced normal equation. FEJ
        // is the primary constraint; keep this experimental projection off
        // unless explicitly evaluating it, because projecting the gradient can
        // amplify residual inconsistency near the numerical nullspace.
        bool project_observability_constraint_ = false;
        constexpr static TYPE lmk_var = TYPE(0.01);

        // 三角化使用归一化像平面噪声；仿真中约为 1 pixel / fx = 0.0054。
        // 它与历史视觉后验中的 uv_var（聚合伪量测噪声）含义不同，不能直接复用 400。
        TYPE triangulation_uv_std = TYPE(0.0054);
        TYPE triangulation_min_parallax_deg = TYPE(5.0);
        TYPE triangulation_max_reprojection_rmse = TYPE(0.03);
        TYPE triangulation_max_position_std = TYPE(50);

        // Schur 重投影残差采用 Huber 权重；阈值以归一化像平面标准差为单位。
        TYPE visual_huber_delta_sigma = TYPE(3);
        // 超过该归一化残差的观测视为明显错误，避免错误深度/关联造成灾难性更新。
        TYPE visual_hard_reprojection_limit = TYPE(0.1);

        // ---- 数据采集(用于可视化/分析，见 tools/) ----
        struct UpdateLog {
            Tus timestamp;
            // 视觉更新【前】(先验)与【后】(后验)的状态，用于看修正作用
            Vec3 p_prior, p_post;
            Vec3 v_prior, v_post;
            Quat q_prior, q_post;
            Vec3 bg_post, ba_post, g_post;
            // 本次视觉更新施加的修正量范数
            TYPE dx_p_norm, dx_q_norm, dx_v_norm;
            // 协方差(位置/姿态/速度的 trace，开根号得米/弧度量级)
            TYPE cov_p_trace, cov_q_trace, cov_v_trace;
            // Schur 序贯伪量测的归一化创新平方统计；均值理论期望约为 1。
            TYPE nis_mean;
            size_t nis_dof;
            size_t n_lmk;      // 参与本次更新的 landmark 数
            size_t n_obs_used;
            size_t n_obs_downweighted;
            size_t n_obs_rejected;
            size_t win_size;   // 滑窗帧数
            bool is_keyframe;
            TYPE oc_leak_before;
            TYPE oc_leak_after;
        };
        std::vector<UpdateLog> logs_;
        std::vector<TriangulationLog> triangulation_logs_;
        bool enable_logging_ = false;

        Eigen::VectorXd Rll_;

        slam::Map &map_;

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
    };
}

#endif //VINSEKF_SCHUR_VINS_H
