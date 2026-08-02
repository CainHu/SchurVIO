/**
 * @file schur_vins.cpp
 * @brief SchurVINS 的外层数据流、误差状态注入与名义状态设置。
 *
 * 本文件只保留算法入口：IMU 数据进入传播模块，相机数据依次执行状态克隆、
 * 视觉后验和状态注入。具体的 IMU、视觉、三角化与影子地图算法分别拆到独立
 * 编译单元，避免一个超大源文件同时承担调度、线性化、求解和地图维护职责。
 */

#include "schur_vins.h"

#include <iostream>
#include <stdexcept>

using namespace slam;

SchurVINS::SchurVINS(slam::Map &map) : map_(map) {
//    sfw_.resize(WIN_SIZE);
//    free_sfw_idx_.resize(WIN_SIZE);
//    for (size_t i = 0; i < WIN_SIZE; ++i) {
//        free_sfw_idx_.emplace_back(i);
//    }

    cov_.resize(COV_SIZE, COV_SIZE);
    cov_.setZero();
    cov_.topLeftCorner<INSState::SIZE, INSState::SIZE>() = state_.cov;

    Rll_.resize(COV_SIZE);
    Rll_.setOnes();
    Rll_ *= uv_var;
}

void SchurVINS::processFrame(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map) {
    ExitHandler exit([&] {
        cam_data_last_ = cam_data;
    });

    // 如果系统的 timestamp 还没被初始化，不进行更新
    if (!state_.timestamp) {
        return;
    }

    // 更新采样时间
    if (cam_data_last_.timestamp) {
        if (cam_data.timestamp > cam_data_last_.timestamp) {
            cam_ts_ = cam_data.timestamp - cam_data_last_.timestamp;
        } else {
            throw std::invalid_argument("CAM data timestamp is not increasing");
        }
    } else {
        cam_ts_ = CAM_TS;
    }

    // 时间同步：视觉残差必须在线性化于相机曝光时刻。若最后一个 IMU 样本早于
    // 相机时间，则用零阶保持的角速度/加速度补传播到相机时间戳。
    if (cam_data.timestamp < state_.timestamp) {
        // 只滞后半个 imu 采样周期, 则警告但继续进行
        if (cam_data.timestamp + (IMU_TS >> 1) > state_.timestamp) {
            std::cerr << "Warning: CAM data is older than state with"
                        << " state.timestamp = " << state_.timestamp
                        << " camera.timestamp = " << cam_data.timestamp
                        << std::endl;
        } else {
            throw std::invalid_argument("VIO data is older than current state too large.");
        }
    } else if (cam_data.timestamp > state_.timestamp){
        // 预测到 CAM 数据的时间
        IMUData dummy_imu;
        dummy_imu.timestamp = cam_data.timestamp;
        dummy_imu.accel = imu_data_last_.accel;
        dummy_imu.gyro = imu_data_last_.gyro;
        const auto dt = static_cast<TYPE>(cam_data.timestamp - state_.timestamp) * TYPE(1e-6);
        predict(dummy_imu, dt);
    }

    // 执行 Visual 更新
    const auto dt = static_cast<TYPE>(cam_ts_) * TYPE(1e-6);
    updateVisual(cam_data, lmk_map, dt);
}

void SchurVINS::updateState(const VecX &dx) {
    using I = INSState;
    using A = AugState;

    // 左乘姿态误差模型：R_true = Exp(delta theta) R_nominal。
    // 小角度向量经 vec2quat 转成单位四元数后左乘；其余欧氏状态直接相加。
    state_.orientation = (vec2quat(Eigen::Map<const Vec3>(dx.data() + I::Q)) * state_.orientation).normalized();
    state_.position += Eigen::Map<const Vec3>(dx.data() + I::P);
    state_.velocity += Eigen::Map<const Vec3>(dx.data() + I::V);
    state_.gyro_bias += Eigen::Map<const Vec3>(dx.data() + I::BG);
    state_.accel_bias += Eigen::Map<const Vec3>(dx.data() + I::BA);
    if constexpr (INSState::ESTIMATE_GRAVITY) {
        state_.gravity += Eigen::Map<const Vec3>(dx.data() + I::G);
    }
    for (size_t n = 0; n < map_.sfw.size(); ++n) {
        Frame *frame = map_.sfw[n];
        // n 是时间顺序，ordering 才是固定联合协方差中的物理块编号；任意删帧后
        // 两者通常不同，状态注入必须使用 ordering 才不会写错 clone。
        const size_t offset = I::SIZE + frame->ordering * A::SIZE;
        frame->q() = (vec2quat(Eigen::Map<const Vec3>(dx.data() + offset + A::Q)) *
                      frame->q()).normalized();
        frame->p() += Eigen::Map<const Vec3>(dx.data() + offset + A::P);
    }
}

void SchurVINS::setQPV(const Quat &q, const Vec3 &p, const Vec3 &v) {
    state_.orientation = q;
    state_.position = p;
    state_.velocity = v;

    Rnb_ = state_.orientation.toRotationMatrix();
}
