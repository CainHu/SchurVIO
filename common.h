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
        Tus timestamp{0};

        Quat orientation; // 全局坐标系到机体坐标系的姿态
        Vec3 position;    // 全局坐标系下的位置 (m)
        Vec3 velocity;    // 全局坐标系下的速度 (m/s)

        // IMU零偏
        Vec3 gyro_bias;   // 陀螺仪零偏
        Vec3 accel_bias;  // 加速度计零偏

        // 重力向量
        Vec3 gravity;

        // 协方差矩阵
        Mat18_18 cov;

        // 过程方差
        Vec18 var_proc;

        // 初始方差
        Vec18 var_init;

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
            var_proc.segment<3>(G) = Vec3::Constant(STB_G_PROC * STB_G_PROC);

            var_init.segment<3>(Q) = Vec3::Constant(STB_Q_INIT * STB_Q_INIT);
            var_init.segment<3>(P) = Vec3::Constant(STB_P_INIT * STB_P_INIT);
            var_init.segment<3>(V) = Vec3::Constant(STB_V_INIT * STB_V_INIT);
            var_init.segment<3>(BG) = Vec3::Constant(STB_BG_INIT * STB_BG_INIT);
            var_init.segment<3>(BA) = Vec3::Constant(STB_BA_INIT * STB_BA_INIT);
            var_init.segment<3>(G) = Vec3::Constant(STB_G_INIT * STB_G_INIT);

            cov = var_init.asDiagonal();
        }

        constexpr static int Q = 0;
        constexpr static int P = Q + 3;
        constexpr static int V = P + 3;
        constexpr static int BG = V + 3;
        constexpr static int BA = BG + 3;
        constexpr static int G = BA + 3;
        constexpr static int SIZE = G + 3;

        constexpr static double STB_Q_INIT = 1e-1;
        constexpr static double STB_P_INIT = 1e-0;
        constexpr static double STB_V_INIT = 1e-1;
        constexpr static double STB_BG_INIT = 5e-2;
        constexpr static double STB_BA_INIT = 1e-1;
        constexpr static double STB_G_INIT = 1e-0;

        constexpr static double STB_Q_PROC = 4e-2;
        constexpr static double STB_P_PROC = 3e-3;
        constexpr static double STB_V_PROC = 2e-1;
        constexpr static double STB_BG_PROC = 5e-3;
        constexpr static double STB_BA_PROC = 1e-2;
        constexpr static double STB_G_PROC = 1e-3;
    };

//    using State = INSState;

    // Augment 状态
    struct AugState {
        Tus timestamp{0};

        Quat orientation; // 全局坐标系到机体坐标系的姿态
        Vec3 position;    // 全局坐标系下的位置 (m)

        Mat6_6 cov;

        constexpr static int Q = 0;
        constexpr static int P = Q + 3;
        constexpr static int SIZE = P + 3;
    };

    // 外参
    struct ExtState {
        Quat q_ic = Quat::Identity();
        Vec3 t_ic = Vec3::Zero();
    };

    // 路标
    struct LmkState {
        Vec3 position;
        Mat3_3 cov = Mat3_3::Identity();

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
