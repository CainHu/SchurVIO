// 分析工具: 跑仿真并导出 CSV，供 tools/report.html 可视化
//
// 用法:
//   VinsAnalysis [uv_var] [tag]
//     uv_var  视觉量测噪声方差(默认 400)
//     tag     输出文件名后缀，用于噪声扫描时区分多组结果
//
// 输出(写到 out/ 目录):
//   traj_<tag>.csv     每相机帧: GT / EST 位姿速度、误差、协方差
//   update_<tag>.csv   每次视觉更新: 先验/后验状态、修正量、NEES
//   lmk.csv            landmark 真值位置(只在 tag=base 时写)
//   summary.csv        每组参数一行汇总(追加)

#include "../vio_frontend_simulator.h"
#include "../eskf/schur_vins.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

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

} // namespace

int main(int argc, char **argv) {
    const double uv_var = (argc > 1) ? std::atof(argv[1]) : 400.0;
    const std::string tag = (argc > 2) ? argv[2] : "base";
    const double proc_scale = (argc > 3) ? std::atof(argv[3]) : 1.0;
    const std::string out_dir = "out";

    // ---- 仿真 ----
    VIOFrontendSimulator simulator;
    simulator.setImuNoise(0.01, 0.01, 0.001, 0.001);
    simulator.setTrajectoryParams(5.0, 1.0, 100.0);
    std::vector<double> ring_radii = {8.0, 10.0, 12.0};
    simulator.setCircularFeaturesParams(1000, ring_radii, Eigen::Vector3d(0, 0, 1.5));

    std::vector<ImuData> imu_data;
    std::vector<CameraData> camera_data;
    std::vector<State> ground_truth;
    simulator.generateData(imu_data, camera_data, ground_truth);
    const auto &feature_positions = simulator.getFeaturePositions();

    // ---- EKF ----
    slam::Map map;
    static slam::SchurVINS ekf(map);
    ekf.uv_var = uv_var;
    ekf.proc_noise_scale_ = proc_scale;
    ekf.enable_logging_ = true;
    ekf.setQPV(ground_truth[0].q, ground_truth[0].p, ground_truth[0].v);

    // 每相机帧的轨迹记录
    struct TrajRow {
        double t;
        Eigen::Vector3d p_gt, p_est, v_gt, v_est;
        Eigen::Quaterniond q_gt, q_est;
        Eigen::Vector3d bg_est, ba_est, g_est;
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
        const auto path = joinPath(out_dir, "traj_" + tag + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }
        std::fprintf(f, "t,"
                        "px_gt,py_gt,pz_gt,px_est,py_est,pz_est,"
                        "vx_gt,vy_gt,vz_gt,vx_est,vy_est,vz_est,"
                        "qw_gt,qx_gt,qy_gt,qz_gt,qw_est,qx_est,qy_est,qz_est,"
                        "err_p,err_v,err_att,"
                        "ex,ey,ez,eroll,epitch,eyaw,"
                        "bgx,bgy,bgz,bax,bay,baz,gx,gy,gz,"
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
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%zu\n",
                r.t,
                r.p_gt.x(), r.p_gt.y(), r.p_gt.z(), r.p_est.x(), r.p_est.y(), r.p_est.z(),
                r.v_gt.x(), r.v_gt.y(), r.v_gt.z(), r.v_est.x(), r.v_est.y(), r.v_est.z(),
                r.q_gt.w(), r.q_gt.x(), r.q_gt.y(), r.q_gt.z(),
                r.q_est.w(), r.q_est.x(), r.q_est.y(), r.q_est.z(),
                dp.norm(), dv.norm(), da.norm(),
                dp.x(), dp.y(), dp.z(), da.x(), da.y(), da.z(),
                r.bg_est.x(), r.bg_est.y(), r.bg_est.z(),
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
    {
        const auto path = joinPath(out_dir, "update_" + tag + ".csv");
        FILE *f = std::fopen(path.c_str(), "w");
        if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); return 1; }
        std::fprintf(f, "t,is_kf,n_lmk,win,"
                        "errp_prior,errp_post,errv_prior,errv_post,"
                        "erra_prior,erra_post,"
                        "dxp,dxq,dxv,sigma_p,sigma_q,sigma_v,"
                        "nis_mean,nis_dof,improve_p\n");

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
                            "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%zu,%d\n",
                static_cast<double>(L.timestamp - t0) * 1e-6,
                L.is_keyframe ? 1 : 0, L.n_lmk, L.win_size,
                ep0, ep1, ev0, ev1, ea0, ea1,
                L.dx_p_norm, L.dx_q_norm, L.dx_v_norm,
                std::sqrt(L.cov_p_trace), std::sqrt(L.cov_q_trace), std::sqrt(L.cov_v_trace),
                L.nis_mean, L.nis_dof,
                improved);
        }
        std::fclose(f);
        std::printf("wrote %s (%zu updates, %.1f%% improved position)\n",
                    path.c_str(), ekf.logs_.size(),
                    n_total ? 100.0 * (double)n_improve / (double)n_total : 0.0);
    }

    // ---- 写 landmark 真值(只写一次) ----
    if (tag == "base") {
        const auto path = joinPath(out_dir, "lmk.csv");
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
        double sp = 0, sv = 0, sa = 0, mp = 0;
        size_t n_imp = 0;
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
            if (e > mp) mp = e;
        }
        for (const auto &L : ekf.logs_) {
            // 只统计有 GT 对照的改善率(与 update csv 一致的口径在那里算过，这里重算简版)
            (void)L;
        }
        const double n = traj.empty() ? 1.0 : (double)traj.size();
        const double rmse_p = std::sqrt(sp / n);
        const double rmse_v = std::sqrt(sv / n);
        const double rmse_a = std::sqrt(sa / n);

        const auto path = joinPath(out_dir, "summary.csv");
        const bool exists = [&] {
            FILE *t = std::fopen(path.c_str(), "r");
            if (t) { std::fclose(t); return true; }
            return false;
        }();
        FILE *f = std::fopen(path.c_str(), "a");
        if (f) {
            if (!exists) std::fprintf(f, "tag,uv_var,proc_scale,rmse_p,rmse_v,rmse_att,max_err_p,t_cost,updates\n");
            std::fprintf(f, "%s,%.1f,%.4f,%.6f,%.6f,%.6f,%.6f,%.3f,%zu\n",
                         tag.c_str(), uv_var, proc_scale, rmse_p, rmse_v, rmse_a, mp,
                         (double)ekf.t_cost_ / (double)CLOCKS_PER_SEC,
                         ekf.posterior_times_);
            std::fclose(f);
        }
        (void)n_imp;
        std::printf("[%s] uv_var=%.1f  RMSE p=%.4f m  v=%.4f m/s  att=%.4f rad  max_p=%.4f m\n",
                    tag.c_str(), uv_var, rmse_p, rmse_v, rmse_a, mp);
    }

    return 0;
}
