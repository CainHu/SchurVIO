//
// Created by 许家仁 on 2025/8/26.
//

#ifndef VINSEKF_COMMON_H
#define VINSEKF_COMMON_H

#include "type.h"

namespace slam {
    static Mat3_3 hat(const Vec3 &v) {
        Mat3_3 m;
        m << TYPE(0), -v.z(), v.y(),
             v.z(), TYPE(0), -v.x(),
             -v.y(), v.x(), TYPE(0);
        return m;
    }

    static Vec3 quat2vec(const Quat &q) {
        if (q.w() < 0) {
            if (const TYPE v2 = q.vec().squaredNorm(); v2 < TYPE(1e-12)) {
                return q.vec() * TYPE(-2);
            } else {
                const TYPE v = std::sqrt(v2);
                return q.vec() * (TYPE(-2) * std::atan2(v, -q.w()) / v);
            }
        } else {
            if (const TYPE v2 = q.vec().squaredNorm(); v2 < TYPE(1e-12)) {
                return q.vec() * TYPE(2);
            } else {
                const TYPE v = std::sqrt(v2);
                return q.vec() * (TYPE(2) * std::atan2(v, q.w()) / v);
            }
        }
    }

    static Quat vec2quat(const Vec3 &v) {
        if (const TYPE v2 = v.squaredNorm(); v2 < TYPE(1e-12)) {
            return {std::sqrt(TYPE(1) - v2 / TYPE(4)), v(0) / TYPE(2), v(1) / TYPE(2), v(2) / TYPE(2)};
        } else {
            const TYPE angle = std::sqrt(v2);
            const TYPE half_angle = angle / TYPE(2);
            TYPE magnitude = std::sin(half_angle) / angle;
            return {std::cos(half_angle), v(0) * magnitude, v(1) * magnitude, v(2) * magnitude};
        }
    }

    // 相机观测结构体
    struct CameraData {
        Tus timestamp{0};

        // 观测到的特征点ID, 特征点在图像上的坐标(u,v)
        std::unordered_map<size_t, Vec2> measurements; // { {id, (u, v)}, ... }
    };

    // 传感器数据结构
    struct IMUData {
        Tus timestamp{0};

        Vec3 accel;    // 加速度 (m/s^2)
        Vec3 gyro;     // 角速度 (rad/s)
    };

    // INS 状态
    struct INSState {
        // 仿真器的世界系重力已知且初始化精确。关闭重力估计可以去掉与姿态、
        // 加速度计零偏之间的弱可观耦合；真实设备若需要在线估计重力，再改回 true。
        constexpr static bool ESTIMATE_GRAVITY = false;

        constexpr static int Q = 0;
        constexpr static int P = Q + 3;
        constexpr static int V = P + 3;
        constexpr static int BG = V + 3;
        constexpr static int BA = BG + 3;
        constexpr static int G = BA + (ESTIMATE_GRAVITY ? 3 : 0);
        constexpr static int SIZE = G + 3;

        constexpr static double STB_Q_INIT = 3e-2 * 1e-1;
        constexpr static double STB_P_INIT = 1e-0 * 1e-1;
        constexpr static double STB_V_INIT = 1e-1 * 1e-1;
        constexpr static double STB_BG_INIT = 5e-2 * 1e-1;
        constexpr static double STB_BA_INIT = 5e-1 * 1e-1;
        constexpr static double STB_G_INIT = 1e-2 * 1e-1;

        // 连续时间噪声密度。当前仿真标称值为 gyro=0.002 rad/s/sqrt(Hz)、
        // accel=0.02 m/s^2/sqrt(Hz)、bg_rw=0.0001、ba_rw=0.0005；这里保留
        // 约 1.5--2 倍裕量，避免模型失配时滤波器过度自信。
        constexpr static double STB_Q_PROC = 3e-3;
        constexpr static double STB_P_PROC = 3e-3 * 1e-1;
        constexpr static double STB_V_PROC = 3e-2;
        constexpr static double STB_BG_PROC = 2e-4;
        constexpr static double STB_BA_PROC = 1e-3;
        constexpr static double STB_G_PROC = 1e-3 * 1e-1;

        Tus timestamp{0};

        // R_wi：把 IMU/机体系向量旋转到世界系。仿真与视觉模型都按
        // p_w = R_wi p_i + p_wi 使用；不要把它解释成 R_iw。
        Quat orientation;
        Vec3 position;    // 全局坐标系下的位置 (m)
        Vec3 velocity;    // 全局坐标系下的速度 (m/s)

        // IMU零偏
        Vec3 gyro_bias;   // 陀螺仪零偏
        Vec3 accel_bias;  // 加速度计零偏

        // 重力向量
        Vec3 gravity;

        // 协方差矩阵
        Eigen::Matrix<TYPE, SIZE, SIZE> cov;

        // 过程方差
        Eigen::Vector<TYPE, SIZE> var_proc;

        // 初始方差
        Eigen::Vector<TYPE, SIZE> var_init;

        INSState() {
            orientation.setIdentity();
            position.setZero();
            velocity.setZero();
            gyro_bias.setZero();
            accel_bias.setZero();
            gravity = Vec3(0., 0., 9.81);

            var_proc.segment<3>(Q) = Vec3::Constant(STB_Q_PROC * STB_Q_PROC);
            var_proc.segment<3>(P) = Vec3::Constant(STB_P_PROC * STB_P_PROC);
            var_proc.segment<3>(V) = Vec3::Constant(STB_V_PROC * STB_V_PROC);
            var_proc.segment<3>(BG) = Vec3::Constant(STB_BG_PROC * STB_BG_PROC);
            var_proc.segment<3>(BA) = Vec3::Constant(STB_BA_PROC * STB_BA_PROC);
            if constexpr (ESTIMATE_GRAVITY) {
                var_proc.segment<3>(G) = Vec3::Constant(STB_G_PROC * STB_G_PROC);
            }

            var_init.segment<3>(Q) = Vec3::Constant(STB_Q_INIT * STB_Q_INIT);
            var_init.segment<3>(P) = Vec3::Constant(STB_P_INIT * STB_P_INIT);
            var_init.segment<3>(V) = Vec3::Constant(STB_V_INIT * STB_V_INIT);
            var_init.segment<3>(BG) = Vec3::Constant(STB_BG_INIT * STB_BG_INIT);
            var_init.segment<3>(BA) = Vec3::Constant(STB_BA_INIT * STB_BA_INIT);
            if constexpr (ESTIMATE_GRAVITY) {
                var_init.segment<3>(G) = Vec3::Constant(STB_G_INIT * STB_G_INIT);
            }

            cov = var_init.asDiagonal();
        }
    };

//    using State = INSState;

    // Augment 状态
    struct AugState {
        Tus timestamp{0};

        // clone 保存对应相机时刻的 R_wi（IMU 系到世界系）和 p_wi。
        Quat orientation;
        Vec3 position;    // 全局坐标系下的位置 (m)

        Mat6_6 cov;

        constexpr static int Q = 0;
        constexpr static int P = Q + 3;
        constexpr static int SIZE = P + 3;
    };

    // 外参
    struct ExtState {
        // 是否把外参也加入状态一起估计。
        // 目前为 false: 外参雅可比 J_ext 仍然保留在代码里(被 if constexpr 屏蔽,
        // 不参与运行但始终参与编译，不会腐烂)，但不进入量测方程。
        // 改为 true 时还需要:
        //   1) 把外参的 6 维加进 COV_SIZE 和协方差布局
        //   2) 在 updateState 里更新 q_ic / t_ic
        //   3) 把 J_EXT / J_ext 填进对应的 Jacobian 列
        constexpr static bool ESTIMATE_EXTRINSIC = false;

        constexpr static int Q = 0;
        constexpr static int P = Q + 3;
        constexpr static int SIZE = P + 3;

        // R_ic 把相机系向量旋转到 IMU 系；t_ic 是“IMU 原点指向相机原点”
        // 的向量，并在 IMU 系表达。因此 p_wc=p_wi+R_wi*t_ic。
        Quat q_ic = Quat::Identity();
        Vec3 t_ic = Vec3::Zero();
    };

    // 路标
    struct LmkState {
        Vec3 position;
        Mat3_3 cov = Mat3_3::Identity() * 1e-3;

        void updateState(const Vec3 &&dx) { position += dx; }
    };

    template <typename F>
    struct ExitHandler {
        explicit ExitHandler(F &&f) : f_(std::move(f)) {}
        ~ExitHandler() { f_(); }
        F f_;
    };
}

#endif //VINSEKF_COMMON_H
