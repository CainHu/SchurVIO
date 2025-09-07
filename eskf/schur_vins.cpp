//
// Created by 许家仁 on 2025/8/26.
//

#include "schur_vins.h"
#include <Eigen/Eigenvalues>
#include <algorithm>

using namespace slam;

SchurVINS::SchurVINS() {
    sfw_.resize(WIN_SIZE);
    free_sfw_idx_.resize(WIN_SIZE);
    for (size_t i = 0; i < WIN_SIZE; ++i) {
        free_sfw_idx_.emplace_back(i);
    }

    cov_.resize(COV_SIZE, COV_SIZE);
    cov_.setZero();
    cov_.topLeftCorner<INSState::SIZE, INSState::SIZE>() = state_.cov;

    Rll_.resize(COV_SIZE);
    Rll_.setOnes();
    Rll_ *= uv_var;

    state_.orientation = Quat(0., 0., 1., 0.);
    state_.position = Vec3(5., 0., 1.);
    state_.velocity = Vec3(0., 1., 0.);
}

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

    // 时间同步：预测到 Camera 数据的时间戳
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

        // 耦合量
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

    // 更新协方差
    const Mat3_3 nRdt = Rnb_ * (-dt);
    const Vec3 nRdv = accel_corr_world_ * (-dt);
    const Mat3_3 nRdv_X = hat(nRdv);
    if constexpr (CONFIG_DEBUG) {
        Mat18_18 AP;

        AP.middleRows<3>(I::Q).noalias() = cov.middleRows<3>(I::Q)
                                           + nRdt * cov.middleRows<3>(I::BG);
        AP.middleRows<3>(I::P).noalias() = cov.middleRows<3>(I::P)
                                           + dt * cov.middleRows<3>(I::V);
        AP.middleRows<3>(I::V).noalias() = cov.middleRows<3>(I::V)
                                           + nRdv_X * cov.middleRows<3>(I::Q)
                                           + nRdt * cov.middleRows<3>(I::BA)
                                           + dt * cov.middleRows<3>(I::G);
        AP.middleRows<3>(I::BG).noalias() = cov.middleRows<3>(I::BG);
        AP.middleRows<3>(I::BA).noalias() = cov.middleRows<3>(I::BA);
        AP.middleRows<3>(I::G).noalias() = cov.middleRows<3>(I::G);

        cov.middleCols<3>(I::Q).noalias() = AP.middleCols<3>(I::Q)
                                            + AP.middleCols<3>(I::BG) * nRdt.transpose();
        cov.middleCols<3>(I::P).noalias() = AP.middleCols<3>(I::P)
                                            + AP.middleCols<3>(I::V) * dt;
        cov.middleCols<3>(I::V).noalias() = AP.middleCols<3>(I::V)
                                            + AP.middleCols<3>(I::Q) * nRdv_X.transpose()
                                            + AP.middleCols<3>(I::BA) * nRdt.transpose()
                                            + AP.middleCols<3>(I::G) * dt;
        cov.middleCols<3>(I::BG).noalias() = AP.middleCols<3>(I::BG);
        cov.middleCols<3>(I::BA).noalias() = AP.middleCols<3>(I::BA);
        cov.middleCols<3>(I::G).noalias() = AP.middleCols<3>(I::G);

        cov = 0.5 * (cov + cov.transpose());
    } else {
        cov.middleCols<3>(I::P).noalias() += cov.middleCols<3>(I::V) * dt;
        cov.middleCols<3>(I::V).noalias() += cov.middleCols<3>(I::Q) * nRdv_X.transpose()
                                             + cov.middleCols<3>(I::BA) * nRdt.transpose()
                                             + cov.middleCols<3>(I::G) * dt;
        cov.middleCols<3>(I::Q).noalias() += cov.middleCols<3>(I::BG) * nRdt.transpose();

        cov.leftCols<9>().middleRows<3>(I::P).noalias() += dt * cov.leftCols<9>().middleRows<3>(I::V);
        cov.leftCols<9>().middleRows<3>(I::V).noalias() += nRdv_X * cov.leftCols<9>().middleRows<3>(I::Q)
                                                           + nRdt * cov.leftCols<9>().middleRows<3>(I::BA)
                                                           + dt * cov.leftCols<9>().middleRows<3>(I::G);
        cov.leftCols<9>().middleRows<3>(I::Q).noalias() += nRdt * cov.leftCols<9>().middleRows<3>(I::BG);

        cov.topRightCorner<9, I::SIZE - 9>().noalias() = cov.bottomLeftCorner<I::SIZE - 9, 9>().transpose();
    }

    // 叠加过程噪声
    cov += (state_.var_proc * dt).asDiagonal();

    // 更新时间戳
    state_.timestamp = imu_data.timestamp;
}

void SchurVINS::pushFrame(const CameraData &cam_data) {
    using A = AugState;

    if (free_sfw_idx_.empty()) {
        throw std::invalid_argument("no free space in sfw");
    }

    std::cout << "Input" << std::endl;
    std::cout << "q = " << state_.orientation << std::endl;
    std::cout << "p = " << state_.position.transpose() << std::endl;
    const auto idx = free_sfw_idx_.back();
    latest_free_sfw_idx_ = idx;
    sfw_[idx].first = AugState {
        .timestamp = state_.timestamp,
        .orientation = state_.orientation,
        .position = state_.position,
    };
    sfw_[idx].second = cam_data;
    free_sfw_idx_.pop_back();

    // 增广状态
    const auto i = INSState::SIZE + idx * A::SIZE;
    const auto j = (WIN_SIZE - (idx + 1)) * A::SIZE;
    if constexpr (CONFIG_DEBUG) {
        cov_.middleRows<A::SIZE>(i).noalias() = cov_.topRows<A::SIZE>();
        cov_.middleCols<A::SIZE>(i).noalias() = cov_.leftCols<A::SIZE>();
        cov_.block<A::SIZE, A::SIZE>(i, i).noalias() = cov_.topLeftCorner<A::SIZE, A::SIZE>();
    } else {
        cov_.middleRows<A::SIZE>(i).leftCols(i).noalias() = cov_.topRows<A::SIZE>().leftCols(i);
        cov_.middleRows<A::SIZE>(i).rightCols(j) = cov_.topRows<A::SIZE>().rightCols(j);

        cov_.middleCols<A::SIZE>(i).topRows(i).noalias() = cov_.leftCols<A::SIZE>().topRows(i);
        cov_.middleCols<A::SIZE>(i).bottomRows(j).noalias() = cov_.leftCols<A::SIZE>().bottomRows(j);

        cov_.block<A::SIZE, A::SIZE>(i, i).noalias() = cov_.topLeftCorner<A::SIZE, A::SIZE>();
    }

    std::cout << "Output" << std::endl;
}

const auto &SchurVINS::popFrame() {
    // TODO: 加入选择策略
    const auto idx = (latest_free_sfw_idx_ + 1) % WIN_SIZE;
    free_sfw_idx_.emplace_back(idx);
    return sfw_[idx];
}

void SchurVINS::updateMap(const slam::CameraData &cam_data) {

}

void SchurVINS::updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, const double dt) {
    using I = INSState;
    using A = AugState;

    // 加入滑窗, 增广状态
    pushFrame(cam_data);

    // 滑窗不满，不进行更新
    if (!free_sfw_idx_.empty()) {
        return;
    }

    // 处理 landmark
    std::vector<size_t> ids;
    ids.reserve(cam_data.measurements.size());
    for (const auto &it : cam_data.measurements) {
        const auto id = it.first;
        if (lmk_.find(id) == lmk_.end()) {
            // TODO: 通过三角化初始化出 landmark 的初始位置
            if (const auto &j = lmk_map.find(id); j != lmk_map.end()) {
                LmkState lmk_state;
                lmk_state.position = j->second;
                lmk_.emplace(id, lmk_state);
            }
        } else {
            ids.emplace_back(id);
        }
    }

    if (ids.empty()) {
        // 移除一帧
        popFrame();

        return;
    }

    std::cout << "11111" << std::endl;

    // Hessian 矩阵
    const auto lmk_size = LMK_SIZE * ids.size();
    MatXX Hpp(COV_SIZE, COV_SIZE);
    MatXX Hpl(COV_SIZE, lmk_size);
    MatXX Hll(lmk_size, lmk_size);
    Hpp.setZero();
    Hpl.setZero();
    Hll.setZero();

    // Gradient
    VecX gp(COV_SIZE);
    VecX gl(lmk_size);
    gp.setZero();
    gl.setZero();

    std::cout << "2222222" << std::endl;

    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i];
        // 遍历滑窗
        for (size_t m = 0; m < sfw_.size(); ++m) {
            const auto &frm = sfw_[m];
            auto &&aug_state = frm.first;
            auto &&meas = frm.second.measurements;
            const auto &&it = meas.find(id);
            if (it == meas.end()) {
                continue;
            }

            const auto Rwi = aug_state.orientation.toRotationMatrix();
            const auto Ric = ext_.q_ic.toRotationMatrix();
            const auto d_ij_w = lmk_[id].position - aug_state.position;
            const auto d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const auto d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const auto est = d_cj_c.head<2>() * inv_d;
            const auto err = it->second - est;

//            std::cout << "landmark[" << id << "]: err = " << err.transpose();
//            std::cout << ", gt = " << it->second.transpose();
//            std::cout << ", est = " << est.transpose() << std::endl;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                 TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            Mat2_3 J_lmk = J * (aug_state.orientation * ext_.q_ic).inverse().toRotationMatrix();

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);;
            J_pose.rightCols<3>().noalias() = -J_lmk;

            Mat2_6 J_ext;
            J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
            J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);

            const size_t lmk_index = LMK_SIZE * i;
            const size_t frm_index = INSState::SIZE + AugState::SIZE * m;

            Hpp.block<6, 6>(frm_index, frm_index).triangularView<Eigen::Upper>() += J_pose.transpose() * J_pose;
//            Hpp.block<6, 6>(frm_index, frm_index).triangularView<Eigen::StrictlyLower>() = Hpp.block<6, 6>(frm_index, frm_index).triangularView<Eigen::StrictlyUpper>().transpose();

            Hll.block<3, 3>(lmk_index, lmk_index).triangularView<Eigen::Upper>() += J_lmk.transpose() * J_lmk;
//            Hll.block<3, 3>(lmk_index, lmk_index).triangularView<Eigen::StrictlyLower>() = Hll.block<3, 3>(lmk_index, lmk_index).triangularView<Eigen::StrictlyUpper>().transpose();

            Hpl.block<6, 3>(frm_index, lmk_index) += J_pose.transpose() * J_lmk;

            gp.segment<6>(frm_index) += J_pose.transpose() * err;
            gl.segment<3>(lmk_index) += J_lmk.transpose() * err;
        }
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();
    Hll.triangularView<Eigen::StrictlyLower>() = Hll.triangularView<Eigen::StrictlyUpper>().transpose();

    std::cout << "3333333" << std::endl;

    MatXX tmp(COV_SIZE, LMK_SIZE);
    for (size_t i = 0; i < ids.size(); ++i) {
        auto index = i * LMK_SIZE;

        // STEP1: 对 Hll 求逆
        const Mat3_3 hll = Hll.block<LMK_SIZE, LMK_SIZE>(index, index);
        const Mat3_3 hll_inv = hll.completeOrthogonalDecomposition().pseudoInverse();

        // STEP2: 计算 Hpl * Hll^-1
        tmp = Hpl.middleCols<LMK_SIZE>(index) * hll_inv;

        // STEP3: 计算 Hpp - Hpl * Hll^-1 * Hpl^T
        Hpp.triangularView<Eigen::Upper>() -= tmp * Hpl.middleCols<LMK_SIZE>(index).transpose();

        // STEP4: 计算 gp - Hpl * Hll^-1 * gl
        gp -= tmp * gl.segment<LMK_SIZE>(index);
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();

    std::cout << "4444444" << std::endl;



    // [[ 更新 State ]]
    // 对 H 使用特征分解: H = V * λ * V^T
    // y = V * λ * V^T * x + V * λ * V^T * n
    // V^T * y = λ * V^T * x + λ * V^T * n
    // Cov[λ * V^T * n] = λ * V^T * Cov[n] * V * λ
    // 如果 Cov[n] = σ^2 * I,
    // 则 Cov[λ * V^T * n] = (σ * λ)^2
    // 所以 V^T * y = λ * V^T * x + λ * n, v ~ N[0, σ]
    VecX dx_p(COV_SIZE);
    dx_p.setZero();
    {
        auto &&cov = cov_;
        Eigen::SelfAdjointEigenSolver<decltype(Hpp)> es(Hpp);

        auto &&ep = gp;
//        VecX VTe = es.eigenvectors().transpose() * ep;

        // Step-0: 过滤掉特征值为0的值
        int zero_end = 0;
        while (std::abs(es.eigenvalues()(zero_end)) < 1e-6) {
            ++zero_end;
        }
//        std::cout << "ev = " << es.eigenvalues().transpose() << std::endl;
//        std::cout << "zero_end = " << zero_end << std::endl;

        // Step-1: 序贯
        for (; zero_end < COV_SIZE; ++zero_end) {
            const auto R = uv_var / dt;
            const auto hT = es.eigenvectors().col(zero_end);

            VecX PhT = cov * hT;
            TYPE var = hT.dot(PhT) + R;
            VecX K = PhT / var;
            cov -= K * PhT.transpose();

            PhT = cov * hT;
            cov.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();
            cov.triangularView<Eigen::StrictlyLower>() = cov.triangularView<Eigen::StrictlyUpper>().transpose();

//            // 更新误差 Ve
//            auto dx = K * (VTe(zero_end) / es.eigenvalues()(zero_end));
//            dx_p += dx;
//            VTe -= es.eigenvalues().asDiagonal() * (es.eigenvectors().transpose() * dx);

            auto e = hT.dot(ep / es.eigenvalues()(zero_end) - dx_p);
            dx_p += K * e;
        }
    }
    updateState(dx_p);
    std::cout << "dx_p = " << dx_p.transpose() << std::endl;



    // [[ 更新 Landmark ]]
    gl -= Hpl.transpose() * dx_p;
    for (size_t i = 0; i < ids.size(); ++i) {
        auto id = ids[i];
        auto index = i * LMK_SIZE;

        VecX dx_l(LMK_SIZE);
        dx_l.setZero();

        auto &&cov = lmk_[id].cov;
        auto &&hll = Hll.block<3, 3>(index, index);
        Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);

        auto &&el = gl.segment<3>(index);
//        VecX VTe = es.eigenvectors().transpose() * el;

        // Step-0: 过滤掉特征值为0的值
        int zero_end = 0;
        while (std::abs(es.eigenvalues()(zero_end)) < 1e-6) {
            ++zero_end;
        }

        // Step-1: 序贯
        for (; zero_end < LMK_SIZE; ++zero_end) {
            const auto R = uv_var / dt;
            const auto hT = es.eigenvectors().col(zero_end);

            VecX PhT = cov * hT;
            TYPE var = hT.dot(PhT) + R;
            VecX K = PhT / var;
            cov -= K * PhT.transpose();

            PhT = cov * hT;
            cov.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();
            cov.triangularView<Eigen::StrictlyLower>() = cov.triangularView<Eigen::StrictlyUpper>().transpose();

//            // 更新误差 Ve
//            auto dx = K * (VTe(zero_end) / es.eigenvalues()(zero_end));
//            dx_l += dx;
//            VTe -= es.eigenvalues().asDiagonal() * (es.eigenvectors().transpose() * dx);

            auto e = hT.dot(el / es.eigenvalues()(zero_end) - dx_l);
            dx_l += K * e;
        }

        lmk_[id].updateState(dx_l);
        std::cout << "id = " << id << ", dx_l = " << dx_l.transpose() << std::endl;
    }




//    std::cout << "ep = " << gp.transpose() << std::endl;
////    std::cout << "P:\n";
////    std::cout << cov_ << std::endl;
//
//    // 更新 state
//    // 量测方程为 gp = Hpp * x + Hpp * n
//    const auto R = uv_var / dt;
//    MatXX HP = Hpp * cov_;
//
//    MatXX S = cov_;
//    S.diagonal().array() += R;
//    S.triangularView<Eigen::Upper>() = Hpp * S.selfadjointView<Eigen::Upper>() * Hpp.transpose();
//    S.diagonal().array() += R;
//    S.triangularView<Eigen::StrictlyLower>() = S.triangularView<Eigen::StrictlyUpper>().transpose();
//
//    std::cout << "aaaa" << std::endl;
//
//    std::cout << "S: " << S.rows() << ", " << S.cols() << std::endl;
//    std::cout << "HP: " << HP.rows() << ", " << HP.cols() << std::endl;
//
////    Eigen::SelfAdjointEigenSolver<decltype(S)> es(S);
////    VecX KT = es.eigenvectors() * ((es.eigenvalues().array() > 0.).select(es.eigenvalues().array().inverse(), 0.).matrix().asDiagonal() * es.eigenvectors().transpose() * HP);
//
//    VecX KT = S.inverse() * HP;
//
//    std::cout << "bbbb" << std::endl;
//
//    std::cout << "KT: " << KT.rows() << ", " << KT.cols() << std::endl;
//    std::cout << "cov_: " << cov_.rows() << ", " << cov_.cols() << std::endl;
//
//    cov_ -= KT.transpose() * HP;
//    std::cout << "cccc" << std::endl;
//    cov_ = 0.5 * (cov_ + cov_.transpose());
//    std::cout << "dddd" << std::endl;
//    VecX dx = KT.transpose() * gp;
//    std::cout << "eeee" << std::endl;
//    updateState(dx);
//    std::cout << "dx = " << dx.transpose() << std::endl;



    std::cout << "Update" << std::endl;

//    // 化简为 Hpp^-1 * gp = e = x + n
//    // 使用序贯
//    auto ep = Hpp.colPivHouseholderQr().solve(gp);
//    Eigen::Vector<TYPE, COV_SIZE> K;
//    for (int n = 0; n < ep.rows(); ++n) {
//        K = cov_.col(n) / (cov_(n, n) + uv_var);
//        cov_.triangularView<Eigen::Upper>() -= K * cov_.row(n);
//        updateState(K * ep(n));
//    }
//
//    // 计算 dx
//    Eigen::Vector<TYPE, COV_SIZE> dx;
//    Eigen::Map<Vec3>(dx.data() + I::Q) = quat2vec(state_.orientation * q.inverse());
//    Eigen::Map<Vec3>(dx.data() + I::P) = state_.position - p;
//    Eigen::Map<Vec3>(dx.data() + I::V) = state_.velocity - v;
//    Eigen::Map<Vec3>(dx.data() + I::BG) = state_.gyro_bias - bg;
//    Eigen::Map<Vec3>(dx.data() + I::BA) = state_.accel_bias - ba;
//    Eigen::Map<Vec3>(dx.data() + I::G) = state_.gravity - g;
//    for (size_t n = 0; n < aug.size(); ++n) {
//        Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::Q) = quat2vec(sfw_[n].first.orientation * aug[n].orientation.inverse());
//        Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::P) = sfw_[n].first.position - aug[n].position;
//    }
//
//    // 计算 el
//    auto &el = gl;
//    el -= Hpl.transpose() * dx;
//
//    // 更新 landmark
//    // 量测方程为 el = gl - Hlp * dxp = Hll * dxl + n
//    // 使用序贯
//    for (size_t i = 0; i < ids.size(); ++i) {
//        auto &&lmk = lmk_[ids[i]];
//        auto &&el_i = el.segment<3>(3 * i);
//        auto &&Hll_i = Hll.block<3, 3>(3 * i, 3 * i);
//        Mat3_3 PHT_l = lmk.cov * Hll_i;;
//        Vec3 K_l = PHT_l * (Hll_i);
//
//        for (int n = 0; n < 3; ++n) {
//            PHT_l = lmk.cov * Hll_i.col(n);
//            K_l = PHT_l / (PHT_l.dot(Hll_i.col(n)) + lmk_var);
//            lmk.cov -= K_l * PHT_l.transpose();
//        }
//    }

    // 移除一帧
    popFrame();
}

void SchurVINS::updateState(auto &&dx) {
    using I = INSState;
    using A = AugState;

    state_.orientation = (vec2quat(Eigen::Map<Vec3>(dx.data() + I::Q)) * state_.orientation).normalized();
    state_.position += Eigen::Map<Vec3>(dx.data() + I::P);
    state_.velocity += Eigen::Map<Vec3>(dx.data() + I::V);
    state_.gyro_bias += Eigen::Map<Vec3>(dx.data() + I::BG);
    state_.accel_bias += Eigen::Map<Vec3>(dx.data() + I::BA);
    state_.gravity += Eigen::Map<Vec3>(dx.data() + I::G);
    for (size_t n = 0; n < sfw_.size(); ++n) {
        sfw_[n].first.orientation = (vec2quat(Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::Q)) * sfw_[n].first.orientation).normalized();
        sfw_[n].first.position += Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::P);
    }
}