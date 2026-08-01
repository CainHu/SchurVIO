// 分析工具: 跑仿真并导出 CSV，供 tools/report.html 可视化
//
// 用法:
//   VinsAnalysis [uv_var] [tag] [proc_scale] [scenario] [duration] [features]
//                [landmark_init] [refine] [imu_noise] [bias_rw] [summary_group]
//     uv_var  Schur 序贯伪量测的噪声密度
//     tag     输出文件名后缀，用于噪声扫描时区分多组结果
//     scenario circle_out / circle_in / helix_3d / stop_go
//
// 输出(写到 out/ 目录):
//   traj_<scenario>_<tag>.csv   每相机帧: GT / EST 位姿速度、误差、协方差
//   update_<scenario>_<tag>.csv 每次视觉更新: 先验/后验、修正量、NIS/鲁棒门控
//   lmk_<scenario>.csv          landmark 真值位置(只在 tag=base 时写)
//   triangulation_<scenario>_<tag>.csv  三角化质量与真值离线对比
//   summary.csv        每组参数一行汇总(追加)

#include "../vio_frontend_simulator.h"
#include "../vio_frontend_simulator1.h"
#include "../vio_representative_simulator.h"
#include "../eskf/schur_vins.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace {

// 四元数误差 -> 轴角向量(弧度)，用于姿态误差
Eigen::Vector3d attitudeError(const Eigen::Quaterniond &gt,
                              const Eigen::Quaterniond &est) {
    return slam::quat2vec((gt.inverse() * est).normalized());
}

template<int N>
double computeNEES(const Eigen::Matrix<double, N, 1> &error,
                   const Eigen::Matrix<double, N, N> &covariance) {
    const Eigen::Matrix<double, N, N> cov_sym =
        0.5 * (covariance + covariance.transpose());
    Eigen::LDLT<Eigen::Matrix<double, N, N>> ldlt(cov_sym);
    if (ldlt.info() != Eigen::Success) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double scale = std::max(1.0, cov_sym.diagonal().cwiseAbs().maxCoeff());
    if (ldlt.vectorD().minCoeff() <= scale * 1e-12) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const Eigen::Matrix<double, N, 1> weighted_error = ldlt.solve(error);
    const double value = error.dot(weighted_error);
    if (!std::isfinite(value) || value < -1e-9) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::max(0.0, value);
}

std::string joinPath(const std::string &dir, const std::string &name) {
    return dir + "/" + name;
}

const char *triangulationStatusName(const slam::SchurVINS::TriangulationStatus status) {
    using Status = slam::SchurVINS::TriangulationStatus;
    switch (status) {
        case Status::Success: return "success";
        case Status::InsufficientViews: return "insufficient_views";
        case Status::LowParallax: return "low_parallax";
        case Status::IllConditioned: return "ill_conditioned";
        case Status::NegativeDepth: return "negative_depth";
        case Status::HighReprojectionError: return "high_reprojection_error";
        case Status::ExcessiveUncertainty: return "excessive_uncertainty";
    }
    return "unknown";
}

struct ScenarioData {
    std::vector<ImuData> imu;
    std::vector<CameraData> camera;
    std::vector<State> ground_truth;
    std::unordered_map<size_t, Eigen::Vector3d> landmarks;
    double focal_length{};
    double camera_noise_std{};
};

bool generateScenario(const std::string &name,
                      const double duration,
                      const size_t feature_count,
                      const bool legacy_white_noise,
                      const bool enable_bias_random_walk,
                      ScenarioData &data) {
    constexpr double acc_noise_density = 0.02;
    constexpr double gyro_noise_density = 0.002;
    constexpr double acc_bias_random_walk = 0.0005;
    constexpr double gyro_bias_random_walk = 0.0001;

    auto configure = [&](auto &simulator) {
        simulator.setImuNoise(acc_noise_density, gyro_noise_density,
                              enable_bias_random_walk ? acc_bias_random_walk : 0.0,
                              enable_bias_random_walk ? gyro_bias_random_walk : 0.0);
        simulator.setLegacyWhiteNoiseDiscretization(legacy_white_noise);
    };

    auto collect = [&](auto &simulator) {
        simulator.generateData(data.imu, data.camera, data.ground_truth);
        data.landmarks = simulator.getFeaturePositions();
        data.focal_length = simulator.getCameraFocalLength();
        data.camera_noise_std = simulator.getCameraNoiseStd();
    };

    if (name == "circle_out") {
        VIOFrontendSimulator simulator;
        configure(simulator);
        simulator.setTrajectoryParams(5.0, 1.0, duration);
        simulator.setCircularFeaturesParams(feature_count, {8.0, 10.0, 12.0},
                                             Eigen::Vector3d(0, 0, 1.5));
        collect(simulator);
        return true;
    }
    if (name == "circle_in") {
        VIOFrontendSimulator1 simulator;
        configure(simulator);
        simulator.setTrajectoryParams(10.0, 1.0, duration);
        simulator.setCircularFeaturesParams(feature_count, {3.0, 5.0, 7.0},
                                             Eigen::Vector3d(0, 0, 1.5));
        collect(simulator);
        return true;
    }
    if (name == "helix_3d" || name == "stop_go") {
        const auto trajectory = name == "helix_3d"
            ? VIORepresentativeSimulator::Trajectory::Helix3D
            : VIORepresentativeSimulator::Trajectory::StopGo;
        VIORepresentativeSimulator simulator(trajectory);
        configure(simulator);
        simulator.setDuration(duration);
        simulator.setFeatureCount(feature_count);
        collect(simulator);
        return true;
    }
    return false;
}

} // namespace

int main(int argc, char **argv) {
    const double uv_var = (argc > 1) ? std::atof(argv[1]) : 1e-2;
    const std::string tag = (argc > 2) ? argv[2] : "base";
    const double proc_scale = (argc > 3) ? std::atof(argv[3]) : 1.0;
    const std::string scenario = (argc > 4) ? argv[4] : "circle_out";
    const double duration = (argc > 5) ? std::atof(argv[5]) : 60.0;
    const size_t feature_count = (argc > 6) ? static_cast<size_t>(std::strtoull(argv[6], nullptr, 10)) : 1000;
    const std::string landmark_init = (argc > 7) ? argv[7] : "tri";
    const bool refine_landmarks = (argc > 8) ? std::atoi(argv[8]) != 0 : true;
    const std::string imu_noise_model = (argc > 9) ? argv[9] : "density";
    const bool enable_bias_random_walk = (argc > 10) ? std::atoi(argv[10]) != 0 : true;
    const std::string summary_group = (argc > 11) ? argv[11] : "main";
    const bool legacy_white_noise = imu_noise_model == "legacy";
    using LandmarkInit = slam::SchurVINS::LandmarkInitializationMode;
    const LandmarkInit landmark_initialization_mode = landmark_init == "gt"
        ? LandmarkInit::GroundTruth
        : (landmark_init == "tri_gt"
           ? LandmarkInit::TriangulationWithOraclePosition
           : LandmarkInit::Triangulation);
    const std::string out_dir = "out";
    const std::string run_name = scenario + "_" + tag;

    // ---- 仿真 ----
    ScenarioData simulation;
    if (!generateScenario(scenario, duration, feature_count, legacy_white_noise,
                          enable_bias_random_walk, simulation)) {
        std::fprintf(stderr, "unknown scenario: %s\n", scenario.c_str());
        return 2;
    }
    const auto &imu_data = simulation.imu;
    const auto &camera_data = simulation.camera;
    const auto &ground_truth = simulation.ground_truth;
    const auto &feature_positions = simulation.landmarks;
    if (ground_truth.empty() || camera_data.empty()) {
        std::fprintf(stderr, "scenario %s generated no data\n", scenario.c_str());
        return 2;
    }

    // ---- EKF ----
    slam::Map map;
    static slam::SchurVINS ekf(map);
    ekf.uv_var = uv_var;
    ekf.proc_noise_scale_ = proc_scale;
    ekf.enable_logging_ = true;
    ekf.landmark_initialization_mode_ = landmark_initialization_mode;
    ekf.refine_landmarks_ = refine_landmarks;
    ekf.triangulation_uv_std = simulation.camera_noise_std / simulation.focal_length;
    ekf.setQPV(ground_truth[0].q, ground_truth[0].p, ground_truth[0].v);

    // 每相机帧的轨迹记录
    struct TrajRow {
        double t;
        Eigen::Vector3d p_gt, p_est, v_gt, v_est;
        Eigen::Quaterniond q_gt, q_est;
        Eigen::Vector3d bg_gt, bg_est, ba_gt, ba_est, g_est;
        double cov_p, cov_q, cov_v;   // trace
        double nees_p, nees_q, nees_v, nees_qpv;
        size_t n_meas;
    };
    std::vector<TrajRow> traj;
    traj.reserve(camera_data.size());

    const auto t0 = camera_data.empty() ? 0 : camera_data.front().timestamp;

    size_t imu_idx = 0, gt_idx = 0;
    for (const auto &cam : camera_data) {
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp < cam.timestamp) {
            ekf.processIMU(imu_data[imu_idx]);
            ++imu_idx;
        }
        while (gt_idx < ground_truth.size() && ground_truth[gt_idx].timestamp < cam.timestamp) {
            ++gt_idx;
        }
        if (gt_idx >= ground_truth.size()) break;

        ekf.processFrame(cam, feature_positions);

        using I = slam::INSState;
        TrajRow r;
        r.t = static_cast<double>(cam.timestamp - t0) * 1e-6;
        r.p_gt = ground_truth[gt_idx].p;
        r.q_gt = ground_truth[gt_idx].q;
        r.v_gt = ground_truth[gt_idx].v;
        r.bg_gt = ground_truth[gt_idx].bg;
        r.ba_gt = ground_truth[gt_idx].ba;
        r.p_est = ekf.state_.position;
        r.q_est = ekf.state_.orientation;
        r.v_est = ekf.state_.velocity;
        r.bg_est = ekf.state_.gyro_bias;
        r.ba_est = ekf.state_.accel_bias;
        r.g_est = ekf.state_.gravity;
        r.cov_p = ekf.cov_.diagonal().segment<3>(I::P).sum();
        r.cov_q = ekf.cov_.diagonal().segment<3>(I::Q).sum();
        r.cov_v = ekf.cov_.diagonal().segment<3>(I::V).sum();

        const Eigen::Vector3d dp = r.p_est - r.p_gt;
        const Eigen::Vector3d dv = r.v_est - r.v_gt;
        const Eigen::Vector3d da = attitudeError(r.q_gt, r.q_est);
        r.nees_p = computeNEES<3>(dp, ekf.cov_.block<3, 3>(I::P, I::P));
        r.nees_q = computeNEES<3>(da, ekf.cov_.block<3, 3>(I::Q, I::Q));
        r.nees_v = computeNEES<3>(dv, ekf.cov_.block<3, 3>(I::V, I::V));

        Eigen::Matrix<double, 9, 1> error_qpv;
        error_qpv.segment<3>(0) = da;
        error_qpv.segment<3>(3) = dp;
        error_qpv.segment<3>(6) = dv;
        Eigen::Matrix<double, 9, 9> cov_qpv;
        constexpr int offsets[3] = {I::Q, I::P, I::V};
        for (int bi = 0; bi < 3; ++bi) {
            for (int bj = 0; bj < 3; ++bj) {
                cov_qpv.block<3, 3>(bi * 3, bj * 3) =
                    ekf.cov_.block<3, 3>(offsets[bi], offsets[bj]);
            }
        }
        r.nees_qpv = computeNEES<9>(error_qpv, cov_qpv);
        r.n_meas = cam.measurements.size();
        traj.emplace_back(r);
    }

    // ---- 写 traj CSV ----
    {
        const auto path = joinPath(out_dir, "traj_" + run_name + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }
        std::fprintf(f, "t,"
                        "px_gt,py_gt,pz_gt,px_est,py_est,pz_est,"
                        "vx_gt,vy_gt,vz_gt,vx_est,vy_est,vz_est,"
                        "qw_gt,qx_gt,qy_gt,qz_gt,qw_est,qx_est,qy_est,qz_est,"
                        "err_p,err_v,err_att,"
                        "ex,ey,ez,eroll,epitch,eyaw,"
                        "bgx_gt,bgy_gt,bgz_gt,bgx,bgy,bgz,"
                        "bax_gt,bay_gt,baz_gt,bax,bay,baz,gx,gy,gz,"
                        "sigma_p,sigma_q,sigma_v,"
                        "nees_p,nees_q,nees_v,nees_qpv,n_meas\n");
        for (const auto &r : traj) {
            const Eigen::Vector3d dp = r.p_est - r.p_gt;
            const Eigen::Vector3d dv = r.v_est - r.v_gt;
            const Eigen::Vector3d da = attitudeError(r.q_gt, r.q_est);
            std::fprintf(f,
                "%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%zu\n",
                r.t,
                r.p_gt.x(), r.p_gt.y(), r.p_gt.z(), r.p_est.x(), r.p_est.y(), r.p_est.z(),
                r.v_gt.x(), r.v_gt.y(), r.v_gt.z(), r.v_est.x(), r.v_est.y(), r.v_est.z(),
                r.q_gt.w(), r.q_gt.x(), r.q_gt.y(), r.q_gt.z(),
                r.q_est.w(), r.q_est.x(), r.q_est.y(), r.q_est.z(),
                dp.norm(), dv.norm(), da.norm(),
                dp.x(), dp.y(), dp.z(), da.x(), da.y(), da.z(),
                r.bg_gt.x(), r.bg_gt.y(), r.bg_gt.z(),
                r.bg_est.x(), r.bg_est.y(), r.bg_est.z(),
                r.ba_gt.x(), r.ba_gt.y(), r.ba_gt.z(),
                r.ba_est.x(), r.ba_est.y(), r.ba_est.z(),
                r.g_est.x(), r.g_est.y(), r.g_est.z(),
                // 协方差 trace 可能因数值问题变负；此时输出负的 sqrt(|.|) 作为标记,
                // 而不是 nan —— 让下游能看见"协方差失去正定性"这个事实。
                (r.cov_p >= 0 ? std::sqrt(r.cov_p) : -std::sqrt(-r.cov_p)),
                (r.cov_q >= 0 ? std::sqrt(r.cov_q) : -std::sqrt(-r.cov_q)),
                (r.cov_v >= 0 ? std::sqrt(r.cov_v) : -std::sqrt(-r.cov_v)),
                r.nees_p, r.nees_q, r.nees_v, r.nees_qpv,
                r.n_meas);
        }
        std::fclose(f);
        std::printf("wrote %s (%zu rows)\n", path.c_str(), traj.size());
    }

    // ---- 写 update CSV: 视觉更新的先验 vs 后验 ----
    // 关键: 把每次更新的先验/后验状态与同时刻 GT 对比，
    //       就能看出视觉后验是否真的把状态【拉近】了真值。
    double posterior_improve_rate = std::numeric_limits<double>::quiet_NaN();
    {
        const auto path = joinPath(out_dir, "update_" + run_name + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }
        std::fprintf(f, "t,is_kf,n_lmk,win,"
                        "errp_prior,errp_post,errv_prior,errv_post,"
                        "erra_prior,erra_post,"
                        "dxp,dxq,dxv,sigma_p,sigma_q,sigma_v,"
                        "nis_mean,nis_dof,obs_used,obs_downweighted,obs_rejected,improve_p\n");

        size_t g = 0;
        size_t n_improve = 0, n_total = 0;
        for (const auto &L : ekf.logs_) {
            while (g + 1 < ground_truth.size() && ground_truth[g].timestamp < L.timestamp) ++g;
            const auto &gt = ground_truth[g];

            const double ep0 = (L.p_prior - gt.p).norm();
            const double ep1 = (L.p_post  - gt.p).norm();
            const double ev0 = (L.v_prior - gt.v).norm();
            const double ev1 = (L.v_post  - gt.v).norm();
            const double ea0 = attitudeError(gt.q, L.q_prior).norm();
            const double ea1 = attitudeError(gt.q, L.q_post).norm();
            const int improved = (ep1 < ep0) ? 1 : 0;
            n_improve += improved;
            ++n_total;

            std::fprintf(f, "%.6f,%d,%zu,%zu,"
                            "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,"
                            "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%zu,%zu,%zu,%zu,%d\n",
                static_cast<double>(L.timestamp - t0) * 1e-6,
                L.is_keyframe ? 1 : 0, L.n_lmk, L.win_size,
                ep0, ep1, ev0, ev1, ea0, ea1,
                L.dx_p_norm, L.dx_q_norm, L.dx_v_norm,
                std::sqrt(L.cov_p_trace), std::sqrt(L.cov_q_trace), std::sqrt(L.cov_v_trace),
                L.nis_mean, L.nis_dof,
                L.n_obs_used, L.n_obs_downweighted, L.n_obs_rejected,
                improved);
        }
        std::fclose(f);
        posterior_improve_rate = n_total
            ? static_cast<double>(n_improve) / static_cast<double>(n_total)
            : std::numeric_limits<double>::quiet_NaN();
        std::printf("wrote %s (%zu updates, %.1f%% improved position)\n",
                    path.c_str(), ekf.logs_.size(),
                    n_total ? 100.0 * (double)n_improve / (double)n_total : 0.0);
    }

    // ---- 写 triangulation CSV：初始化质量、失败原因、后续修正与真值对比 ----
    {
        const auto path = joinPath(out_dir, "triangulation_" + run_name + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }
        std::fprintf(f,
            "t,id,status,success,n_obs,parallax_deg,condition,reproj_rmse,time_us,"
            "x_init,y_init,z_init,x_final,y_final,z_final,x_gt,y_gt,z_gt,"
            "sigma_init,nees_init,err_init,err_final,correction,refinements,improved\n");

        size_t success_count = 0;
        for (const auto &log : ekf.triangulation_logs_) {
            const bool success = log.status == slam::SchurVINS::TriangulationStatus::Success;
            const double nan = std::numeric_limits<double>::quiet_NaN();
            const double sigma_init = success && log.initial_cov_trace >= 0
                                      ? std::sqrt(log.initial_cov_trace) : nan;
            const double err_init = success && log.has_ground_truth
                                    ? (log.initial_position - log.ground_truth).norm() : nan;
            const double err_final = success && log.has_ground_truth
                                     ? (log.latest_position - log.ground_truth).norm() : nan;
            const double correction = success
                                      ? (log.latest_position - log.initial_position).norm() : nan;
            const int improved = success && log.has_ground_truth && err_final < err_init ? 1 : 0;
            success_count += success ? 1 : 0;

            std::fprintf(f,
                "%.6f,%zu,%s,%d,%zu,%.6e,%.6e,%.6e,%.3f,"
                "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,"
                "%.6e,%.6e,%.6e,%.6e,%.6e,%zu,%d\n",
                static_cast<double>(log.timestamp - t0) * 1e-6,
                static_cast<size_t>(log.id), triangulationStatusName(log.status), success ? 1 : 0,
                log.observation_count, log.max_parallax_deg, log.condition_number,
                log.reprojection_rmse, log.elapsed_us,
                log.initial_position.x(), log.initial_position.y(), log.initial_position.z(),
                log.latest_position.x(), log.latest_position.y(), log.latest_position.z(),
                log.ground_truth.x(), log.ground_truth.y(), log.ground_truth.z(),
                sigma_init, log.initial_nees, err_init, err_final, correction,
                log.refinement_count, improved);
        }
        std::fclose(f);
        std::printf("wrote %s (%zu attempts, %zu successes)\n",
                    path.c_str(), ekf.triangulation_logs_.size(), success_count);
    }

    // ---- 写 landmark 真值(只写一次) ----
    if (tag == "base") {
        const auto path = joinPath(out_dir, "lmk_" + scenario + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (f) {
            std::fprintf(f, "id,x,y,z\n");
            for (const auto &kv : feature_positions) {
                std::fprintf(f, "%zu,%.6f,%.6f,%.6f\n",
                             kv.first, kv.second.x(), kv.second.y(), kv.second.z());
            }
            std::fclose(f);
            std::printf("wrote %s (%zu landmarks)\n", path.c_str(), feature_positions.size());
        }
    }

    // ---- 汇总(追加到 summary.csv) ----
    {
        double sp = 0, sv = 0, sa = 0, sbg = 0, sba = 0, mp = 0;
        double nees_sum = 0;
        size_t nees_count = 0;
        // 统计协方差失去正定性的帧数(trace < 0)，这是数值健康度指标
        size_t n_neg_cov = 0;
        for (const auto &r : traj) {
            if (r.cov_p < 0 || r.cov_q < 0 || r.cov_v < 0) ++n_neg_cov;
        }
        if (n_neg_cov) {
            std::fprintf(stderr,
                "[%s] WARNING: %zu/%zu 帧的协方差 trace 为负(失去正定性)\n",
                tag.c_str(), n_neg_cov, traj.size());
        }
        for (const auto &r : traj) {
            const double e = (r.p_est - r.p_gt).norm();
            sp += e * e;
            sv += (r.v_est - r.v_gt).squaredNorm();
            sa += attitudeError(r.q_gt, r.q_est).squaredNorm();
            sbg += (r.bg_est - r.bg_gt).squaredNorm();
            sba += (r.ba_est - r.ba_gt).squaredNorm();
            if (std::isfinite(r.nees_qpv)) {
                nees_sum += r.nees_qpv / 9.0;
                ++nees_count;
            }
            if (e > mp) mp = e;
        }
        double nis_sum = 0;
        size_t nis_count = 0;
        for (const auto &log : ekf.logs_) {
            if (log.nis_dof && std::isfinite(log.nis_mean)) {
                nis_sum += log.nis_mean;
                ++nis_count;
            }
        }
        const double n = traj.empty() ? 1.0 : (double)traj.size();
        const double rmse_p = std::sqrt(sp / n);
        const double rmse_v = std::sqrt(sv / n);
        const double rmse_a = std::sqrt(sa / n);
        const double rmse_bg = std::sqrt(sbg / n);
        const double rmse_ba = std::sqrt(sba / n);

        // Absolute trajectory error after one rigid SE(3) alignment. Scale is
        // deliberately fixed so monocular/VIO scale errors remain visible.
        double rmse_p_aligned = std::numeric_limits<double>::quiet_NaN();
        if (traj.size() >= 3) {
            Eigen::Vector3d estimate_mean = Eigen::Vector3d::Zero();
            Eigen::Vector3d truth_mean = Eigen::Vector3d::Zero();
            for (const auto &row : traj) {
                estimate_mean += row.p_est;
                truth_mean += row.p_gt;
            }
            estimate_mean /= n;
            truth_mean /= n;

            Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
            for (const auto &row : traj) {
                cross_covariance.noalias() +=
                    (row.p_est - estimate_mean) * (row.p_gt - truth_mean).transpose();
            }
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                cross_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d reflection = Eigen::Matrix3d::Identity();
            reflection(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
            const Eigen::Matrix3d rotation =
                svd.matrixV() * reflection * svd.matrixU().transpose();
            const Eigen::Vector3d translation = truth_mean - rotation * estimate_mean;
            double aligned_squared_error = 0.0;
            for (const auto &row : traj) {
                const Eigen::Vector3d aligned = rotation * row.p_est + translation;
                aligned_squared_error += (aligned - row.p_gt).squaredNorm();
            }
            rmse_p_aligned = std::sqrt(aligned_squared_error / n);
        }

        // One-second relative pose error in each pose's local frame. This is
        // gauge invariant and therefore separates local odometry quality from
        // unobservable global translation/yaw drift.
        double rpe_p_sum = 0.0, rpe_a_sum = 0.0;
        size_t rpe_count = 0;
        size_t lag_index = 1;
        for (size_t i = 0; i < traj.size(); ++i) {
            lag_index = std::max(lag_index, i + 1);
            const double target = traj[i].t + 1.0;
            while (lag_index + 1 < traj.size()
                   && std::abs(traj[lag_index + 1].t - target)
                      < std::abs(traj[lag_index].t - target)) {
                ++lag_index;
            }
            if (lag_index >= traj.size() || std::abs(traj[lag_index].t - target) > 0.1) {
                continue;
            }
            const Eigen::Quaterniond q_gt_rel =
                (traj[i].q_gt.inverse() * traj[lag_index].q_gt).normalized();
            const Eigen::Quaterniond q_est_rel =
                (traj[i].q_est.inverse() * traj[lag_index].q_est).normalized();
            const Eigen::Vector3d p_gt_rel =
                traj[i].q_gt.inverse() * (traj[lag_index].p_gt - traj[i].p_gt);
            const Eigen::Vector3d p_est_rel =
                traj[i].q_est.inverse() * (traj[lag_index].p_est - traj[i].p_est);
            rpe_p_sum += (p_est_rel - p_gt_rel).squaredNorm();
            rpe_a_sum += attitudeError(q_gt_rel, q_est_rel).squaredNorm();
            ++rpe_count;
        }
        const double rpe_1s_p = rpe_count
            ? std::sqrt(rpe_p_sum / static_cast<double>(rpe_count))
            : std::numeric_limits<double>::quiet_NaN();
        const double rpe_1s_att = rpe_count
            ? std::sqrt(rpe_a_sum / static_cast<double>(rpe_count))
            : std::numeric_limits<double>::quiet_NaN();
        const double mean_nees = nees_count ? nees_sum / static_cast<double>(nees_count)
                                             : std::numeric_limits<double>::quiet_NaN();
        const double mean_nis = nis_count ? nis_sum / static_cast<double>(nis_count)
                                           : std::numeric_limits<double>::quiet_NaN();
        const double gravity_error = traj.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : (traj.back().g_est - Eigen::Vector3d(0.0, 0.0, 9.81)).norm();
        size_t triangulation_success = 0;
        for (const auto &log : ekf.triangulation_logs_) {
            triangulation_success +=
                log.status == slam::SchurVINS::TriangulationStatus::Success ? 1 : 0;
        }

        size_t observations_used = 0;
        size_t observations_downweighted = 0;
        size_t observations_rejected = 0;
        for (const auto &log : ekf.logs_) {
            observations_used += log.n_obs_used;
            observations_downweighted += log.n_obs_downweighted;
            observations_rejected += log.n_obs_rejected;
        }
        const double obs_downweight_rate = observations_used
            ? static_cast<double>(observations_downweighted)
              / static_cast<double>(observations_used)
            : std::numeric_limits<double>::quiet_NaN();
        const size_t observations_considered = observations_used + observations_rejected;
        const double obs_reject_rate = observations_considered
            ? static_cast<double>(observations_rejected)
              / static_cast<double>(observations_considered)
            : std::numeric_limits<double>::quiet_NaN();

        const bool is_ablation = summary_group == "ablation";
        const auto path = joinPath(out_dir,
                                   is_ablation ? "ablation_summary.csv" : "summary.csv");
        const bool reset_summary = scenario == "circle_out"
            && ((!is_ablation && tag == "base") || (is_ablation && tag == "abl_full"));
        const bool exists = !reset_summary && [&] {
            FILE *t = std::fopen(path.c_str(), "r");
            if (t) { std::fclose(t); return true; }
            return false;
        }();
        // base 是一组新实验的起点：先清掉旧算法留下的扫描结果，避免报告混用数据。
        FILE *f = std::fopen(path.c_str(), reset_summary ? "w" : "a");
        if (f) {
            if (!exists) {
                if (is_ablation) {
                    std::fprintf(f,
                        "scenario,tag,landmark_init,refine_landmarks,imu_noise_model,bias_random_walk,"
                        "uv_var,proc_scale,duration,features,rmse_p,rmse_p_aligned,rpe_1s_p,rpe_1s_att,"
                        "rmse_v,rmse_att,rmse_bg,rmse_ba,max_err_p,mean_nees,mean_nis,gravity_error,"
                        "neg_cov,t_cost,updates,posterior_improve_rate,obs_downweight_rate,obs_reject_rate,"
                        "tri_success,tri_attempts\n");
                } else {
                    std::fprintf(f,
                        "scenario,tag,uv_var,proc_scale,estimate_gravity,duration,features,"
                        "rmse_p,rmse_v,rmse_att,rmse_bg,rmse_ba,max_err_p,mean_nees,mean_nis,"
                        "gravity_error,neg_cov,t_cost,updates,tri_success,tri_attempts\n");
                }
            }
            if (is_ablation) {
                std::fprintf(f,
                    "%s,%s,%s,%d,%s,%d,%.9g,%.4f,%.1f,%zu,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6e,%.6e,%.6e,"
                    "%zu,%.3f,%zu,%.6f,%.6f,%.6f,%zu,%zu\n",
                    scenario.c_str(), tag.c_str(), landmark_init.c_str(),
                    refine_landmarks ? 1 : 0, imu_noise_model.c_str(),
                    enable_bias_random_walk ? 1 : 0, uv_var, proc_scale,
                    duration, feature_count, rmse_p, rmse_p_aligned, rpe_1s_p,
                    rpe_1s_att, rmse_v, rmse_a, rmse_bg, rmse_ba, mp,
                    mean_nees, mean_nis, gravity_error, n_neg_cov,
                    static_cast<double>(ekf.t_cost_) / static_cast<double>(CLOCKS_PER_SEC),
                    ekf.posterior_times_, posterior_improve_rate,
                    obs_downweight_rate, obs_reject_rate, triangulation_success,
                    ekf.triangulation_logs_.size());
            } else {
                std::fprintf(f,
                    "%s,%s,%.9g,%.4f,%d,%.1f,%zu,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6e,%.6e,"
                    "%.6e,%zu,%.3f,%zu,%zu,%zu\n",
                    scenario.c_str(), tag.c_str(), uv_var, proc_scale,
                    slam::INSState::ESTIMATE_GRAVITY ? 1 : 0, duration, feature_count,
                    rmse_p, rmse_v, rmse_a, rmse_bg, rmse_ba, mp, mean_nees, mean_nis,
                    gravity_error, n_neg_cov,
                    static_cast<double>(ekf.t_cost_) / static_cast<double>(CLOCKS_PER_SEC),
                    ekf.posterior_times_, triangulation_success,
                    ekf.triangulation_logs_.size());
            }
            std::fclose(f);
        }
        std::printf("[%s/%s] uv_var=%.9g  RMSE p=%.4f m  v=%.4f m/s  att=%.4f rad  max_p=%.4f m\n",
                    scenario.c_str(), tag.c_str(), uv_var, rmse_p, rmse_v, rmse_a, mp);
    }

    return 0;
}
