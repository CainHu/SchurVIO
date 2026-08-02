/**
 * @file schur_vins_imu.cpp
 * @brief IMU 名义状态积分、误差状态转移与协方差传播。
 *
 * 名义状态采用离散惯性运动模型：
 *   R_{k+1} = R_k Exp((omega_m-b_g) Delta t)，
 *   p_{k+1} = p_k + v_k Delta t
 *             + 1/2 (R_k(a_m-b_a)+g) Delta t^2，
 *   v_{k+1} = v_k + (R_k(a_m-b_a)+g) Delta t。
 *
 * 误差协方差按 P_{k+1}=F P_k F^T+Q_d 传播。代码直接更新 F 会影响的
 * 行列块，不显式构造完整大矩阵，从而保留所有 clone 交叉协方差并减少临时量。
 */

#include "schur_vins.h"

#include <iostream>
#include <stdexcept>

using namespace slam;

void SchurVINS::processIMU(const slam::IMUData &imu_data) {
    ExitHandler exit([&] {
        // 保存数据
        imu_data_last_ = imu_data;
    });

    // 第一帧用于初始化 timestamp
    if (!imu_data_last_.timestamp) {
        state_.timestamp = imu_data.timestamp;
        return;
    }

    // 计算采样时间
    if (imu_data.timestamp > imu_data_last_.timestamp) {
        imu_ts_ = imu_data.timestamp - imu_data_last_.timestamp;
    } else {
        std::cerr << "t1 = " << imu_data.timestamp << ", t2 = " << imu_data_last_.timestamp << std::endl;
        throw std::invalid_argument("IMU data timestamp is not increasing");
    }

    // 计算时间差
    if (imu_data.timestamp < state_.timestamp) {
        throw std::invalid_argument("IMU time lags behind the state time");
    } else if (imu_data.timestamp == state_.timestamp) {
        // 状态已是最新，无需更新
        return;
    }

    // 预测状态
    TYPE dt = static_cast<TYPE>(imu_data.timestamp - state_.timestamp) * TYPE(1e-6);
    predict(imu_data, dt);
}

void SchurVINS::predict(const slam::IMUData &imu_data, const double dt) {
    using I = INSState;
//    auto &cov = state_.cov;
    auto &&cov = cov_.topLeftCorner<INSState::SIZE, INSState::SIZE>();

    // 处理IMU数据（去除零偏）
    gyro_corr_= imu_data.gyro - state_.gyro_bias;
    accel_corr_ = imu_data.accel - state_.accel_bias;
    if constexpr (CONFIG_DEBUG) {
        Rnb_ = state_.orientation.toRotationMatrix();

        // 姿态更新
        const Vec3 delta_ang = gyro_corr_ * dt;
        state_.orientation *= vec2quat(delta_ang);
        state_.orientation.normalize();

        // 速度更新
        const Vec3 v_prev = state_.velocity;
        accel_corr_world_ = Rnb_ * accel_corr_;
        a_world_ = accel_corr_world_ + state_.gravity;
        state_.velocity += a_world_ * dt;

        // 位置更新
        state_.position += (state_.velocity + v_prev) * (0.5 * dt);
    } else {
        accel_corr_world_ = Rnb_ * accel_corr_;
        a_world_ = accel_corr_world_ + state_.gravity;

        // SO(3) 积分中的两个解析矩阵：
        //   J1(phi)=sum_{n>=0} phi^n/(n+1)!，用于积分旋转后的加速度；
        //   J2(phi)=sum_{n>=0} phi^n/(n+2)!，用于二次积分到位置。
        // 这里保留到二阶，phi=omega*dt 很小时比直接数值积分更稳定。
        const Vec3 delta_ang = gyro_corr_ * dt;
        const Mat3_3 J1 = Mat3_3::Identity() + hat(delta_ang / 2.) + (delta_ang / 6) * delta_ang.transpose();
        const Mat3_3 J2 = 0.5 * Mat3_3::Identity() + hat(delta_ang / 6.) + (delta_ang / 24.) * delta_ang.transpose();

        // 位置更新
        state_.position += (state_.velocity + state_.gravity * TYPE(0.5 * dt) + Rnb_ * (J2 * accel_corr_) * dt) * dt;

        // 速度更新
        state_.velocity += (Rnb_ * (J1 * accel_corr_) + state_.gravity) * dt;

        // 姿态更新
        state_.orientation *= vec2quat(delta_ang);
        state_.orientation.normalize();

        Rnb_ = state_.orientation.toRotationMatrix();
    }

    // 一阶误差状态转移的非零耦合块：
    //   delta theta+ = delta theta - R*dt*delta bg
    //   delta p+     = delta p + dt*delta v
    //   delta v+     = delta v - [R a]x*dt*delta theta - R*dt*delta ba
    // 若估计重力，还包含 delta v+ += dt*delta g。
    // 下列 nRdt/nRdv_X 正是这些 F 块的离散系数。
    const Mat3_3 nRdt = Rnb_ * (-dt);
    const Vec3 nRdv = accel_corr_world_ * (-dt);
    const Mat3_3 nRdv_X = hat(nRdv);

    if constexpr (USE_STABLE_COVARIANCE_PREDICTION) {
        using MatINS = Eigen::Matrix<TYPE, I::SIZE, I::SIZE>;
        using MatCross = Eigen::Matrix<TYPE, I::SIZE, COV_SIZE - I::SIZE>;
        MatINS A = MatINS::Identity();
        A.template block<3, 3>(I::Q, I::BG) = nRdt;
        A.template block<3, 3>(I::P, I::V) = Mat3_3::Identity() * dt;
        A.template block<3, 3>(I::V, I::Q) = nRdv_X;
        A.template block<3, 3>(I::V, I::BA) = nRdt;
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            A.template block<3, 3>(I::V, I::G) = Mat3_3::Identity() * dt;
        }

        // 合同变换保持半正定性：若 P >= 0，则 A*P*A^T >= 0。
        // 使用副本避免 Eigen 表达式在赋值时与 cov alias。
        const MatINS P_prev = cov.selfadjointView<Eigen::Upper>();
        const MatCross P_cross_prev =
            cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE);
        cov.noalias() = A * P_prev * A.transpose();
        cov = TYPE(0.5) * (cov + cov.transpose());
        cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE).noalias() =
            A * P_cross_prev;
        cov_.bottomLeftCorner(COV_SIZE - I::SIZE, I::SIZE) =
            cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE).transpose();
    } else if constexpr (CONFIG_DEBUG) {
        // 显式计算联合传播的顶部块 A * [P_ii P_ic]，但不构造 A。
        Eigen::Matrix<TYPE, I::SIZE, COV_SIZE> AP;

        AP.middleRows<3>(I::Q).noalias() = cov_.middleRows<3>(I::Q)
                                           + nRdt * cov_.middleRows<3>(I::BG);
        AP.middleRows<3>(I::P).noalias() = cov_.middleRows<3>(I::P)
                                           + dt * cov_.middleRows<3>(I::V);
        AP.middleRows<3>(I::V).noalias() = cov_.middleRows<3>(I::V)
                                           + nRdv_X * cov_.middleRows<3>(I::Q)
                                           + nRdt * cov_.middleRows<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            AP.middleRows<3>(I::V).noalias() += dt * cov_.middleRows<3>(I::G);
        }
        AP.middleRows<3>(I::BG).noalias() = cov_.middleRows<3>(I::BG);
        AP.middleRows<3>(I::BA).noalias() = cov_.middleRows<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            AP.middleRows<3>(I::G).noalias() = cov_.middleRows<3>(I::G);
        }

        cov.middleCols<3>(I::Q).noalias() = AP.middleCols<3>(I::Q)
                                            + AP.middleCols<3>(I::BG) * nRdt.transpose();
        cov.middleCols<3>(I::P).noalias() = AP.middleCols<3>(I::P)
                                            + AP.middleCols<3>(I::V) * dt;
        cov.middleCols<3>(I::V).noalias() = AP.middleCols<3>(I::V)
                                            + AP.middleCols<3>(I::Q) * nRdv_X.transpose()
                                            + AP.middleCols<3>(I::BA) * nRdt.transpose();
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov.middleCols<3>(I::V).noalias() += AP.middleCols<3>(I::G) * dt;
        }
        cov.middleCols<3>(I::BG).noalias() = AP.middleCols<3>(I::BG);
        cov.middleCols<3>(I::BA).noalias() = AP.middleCols<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov.middleCols<3>(I::G).noalias() = AP.middleCols<3>(I::G);
        }

        cov = 0.5 * (cov + cov.transpose());
        cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE) =
            AP.rightCols(COV_SIZE - I::SIZE);
        cov_.bottomLeftCorner(COV_SIZE - I::SIZE, I::SIZE) =
            cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE).transpose();
    } else {
        // 优化路径按两步完成 P+ = F P F^T：先就地更新完整列得到 P F^T，
        // 再更新 INS 顶部非平凡行得到 F P F^T。clone-clone 块不受 IMU 转移
        // 影响，INS-clone 交叉块最后由对称性恢复，因此无需构造 COV_SIZE 阶 F。
        cov_.middleCols<3>(I::P).noalias() += cov_.middleCols<3>(I::V) * dt;
        cov_.middleCols<3>(I::V).noalias() += cov_.middleCols<3>(I::Q) * nRdv_X.transpose()
                                              + cov_.middleCols<3>(I::BA) * nRdt.transpose();
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov_.middleCols<3>(I::V).noalias() += cov_.middleCols<3>(I::G) * dt;
        }
        cov_.middleCols<3>(I::Q).noalias() += cov_.middleCols<3>(I::BG) * nRdt.transpose();

        cov.leftCols<9>().middleRows<3>(I::P).noalias() += dt * cov.leftCols<9>().middleRows<3>(I::V);
        cov.leftCols<9>().middleRows<3>(I::V).noalias() += nRdv_X * cov.leftCols<9>().middleRows<3>(I::Q)
                                                           + nRdt * cov.leftCols<9>().middleRows<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov.leftCols<9>().middleRows<3>(I::V).noalias() += dt * cov.leftCols<9>().middleRows<3>(I::G);
        }
        cov.leftCols<9>().middleRows<3>(I::Q).noalias() += nRdt * cov.leftCols<9>().middleRows<3>(I::BG);

        cov.topRightCorner<9, I::SIZE - 9>().noalias() = cov.bottomLeftCorner<I::SIZE - 9, 9>().transpose();
        cov_.topRightCorner(I::SIZE, COV_SIZE - I::SIZE) =
            cov_.bottomLeftCorner(COV_SIZE - I::SIZE, I::SIZE).transpose();
    }

    // 连续时间噪声密度离散化为 Q_d≈Q_c*dt；proc_noise_scale_ 仅用于敏感度
    // 扫描。这里加到 INS 对角块，clone 没有独立过程噪声。
    cov += (state_.var_proc * (dt * proc_noise_scale_)).asDiagonal();

    // 更新时间戳
    state_.timestamp = imu_data.timestamp;
}
