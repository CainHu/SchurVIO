//
// Created by 许家仁 on 2025/8/26.
//

#include "schur_vins.h"
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <algorithm>

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
        Eigen::Matrix<TYPE, INSState::SIZE, INSState::SIZE> AP;

        AP.middleRows<3>(I::Q).noalias() = cov.middleRows<3>(I::Q)
                                           + nRdt * cov.middleRows<3>(I::BG);
        AP.middleRows<3>(I::P).noalias() = cov.middleRows<3>(I::P)
                                           + dt * cov.middleRows<3>(I::V);
        AP.middleRows<3>(I::V).noalias() = cov.middleRows<3>(I::V)
                                           + nRdv_X * cov.middleRows<3>(I::Q)
                                           + nRdt * cov.middleRows<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            AP.middleRows<3>(I::V).noalias() += dt * cov.middleRows<3>(I::G);
        }
        AP.middleRows<3>(I::BG).noalias() = cov.middleRows<3>(I::BG);
        AP.middleRows<3>(I::BA).noalias() = cov.middleRows<3>(I::BA);
        AP.middleRows<3>(I::G).noalias() = cov.middleRows<3>(I::G);

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
    } else {
        cov.middleCols<3>(I::P).noalias() += cov.middleCols<3>(I::V) * dt;
        cov.middleCols<3>(I::V).noalias() += cov.middleCols<3>(I::Q) * nRdv_X.transpose()
                                             + cov.middleCols<3>(I::BA) * nRdt.transpose();
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov.middleCols<3>(I::V).noalias() += cov.middleCols<3>(I::G) * dt;
        }
        cov.middleCols<3>(I::Q).noalias() += cov.middleCols<3>(I::BG) * nRdt.transpose();

        cov.leftCols<9>().middleRows<3>(I::P).noalias() += dt * cov.leftCols<9>().middleRows<3>(I::V);
        cov.leftCols<9>().middleRows<3>(I::V).noalias() += nRdv_X * cov.leftCols<9>().middleRows<3>(I::Q)
                                                           + nRdt * cov.leftCols<9>().middleRows<3>(I::BA);
        if constexpr (INSState::ESTIMATE_GRAVITY) {
            cov.leftCols<9>().middleRows<3>(I::V).noalias() += dt * cov.leftCols<9>().middleRows<3>(I::G);
        }
        cov.leftCols<9>().middleRows<3>(I::Q).noalias() += nRdt * cov.leftCols<9>().middleRows<3>(I::BG);

        cov.topRightCorner<9, I::SIZE - 9>().noalias() = cov.bottomLeftCorner<I::SIZE - 9, 9>().transpose();
    }

    // 叠加过程噪声
    cov += (state_.var_proc * dt).asDiagonal();

    // 更新时间戳
    state_.timestamp = imu_data.timestamp;
}

void SchurVINS::pushFrame(const CameraData &cam_data, bool is_keyframe) {
    using A = AugState;

    if (!is_keyframe) {
        // 非关键帧，不增广状态
        return;
    }

    // 关键帧：创建并加入滑窗
//    std::cout << "Find Key Frame" << std::endl;
    auto frm = map_.pushKeyFrame(cam_data.timestamp);
    frm->timestamp = state_.timestamp;
    frm->q() = state_.orientation;
    frm->p() = state_.position;

    // 增广状态
    auto idx = map_.getWinLatestIndex();
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

//    std::cout << "Output" << std::endl;
}

void SchurVINS::popFrame() {
//    // TODO: 加入选择策略
//    const auto idx = (latest_free_sfw_idx_ + 1) % WIN_SIZE;
//    free_sfw_idx_.emplace_back(idx);
//    return sfw_[idx];
    map_.popFrame();
}

void SchurVINS::updateMap(const slam::CameraData &cam_data) {

}

void SchurVINS::updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, const double dt) {
    using I = INSState;
    using A = AugState;

    // 判断是否为关键帧
    bool is_keyframe = map_.isKeyFrame(cam_data);

//    if (is_keyframe) {
//        std::cout << "Find Key Frame" << std::endl;
//    } else {
//        std::cout << "Not Key Frame" << std::endl;
//    }

    // 关键帧：加入滑窗并增广状态
    // 非关键帧：Zero-copy，什么都不做，观测在后面直接从 cam_data 读取
    if (is_keyframe) {
        pushFrame(cam_data, true);
        auto current_frame = map_.getWinLatestFrame();
        // 添加关键帧的观测到map
        map_.addObservations(current_frame, cam_data);
    }

    // 需要至少2帧才能进行视觉更新（用于三角化）
    size_t current_win_size = map_.sfw.size();
    if (current_win_size < 2) {
//        std::cout << "Sliding Window has " << current_win_size << " frame(s), skip visual update" << std::endl;
        return;
    }

//    std::cout << "Do vision update with " << current_win_size << " keyframes";
//    if (!is_keyframe) {
//        std::cout << " + 1 non-keyframe";
//    }
//    std::cout << std::endl;

    // 处理 landmark
//    std::vector<size_t> ids;
//    ids.reserve(cam_data.measurements.size());
//    for (const auto &it : cam_data.measurements) {
//        const auto id = it.first;
//        if (lmk_.find(id) == lmk_.end()) {
//            // TODO: 通过三角化初始化出 landmark 的初始位置
//            if (const auto &j = lmk_map.find(id); j != lmk_map.end()) {
//                LmkState lmk_state;
//                lmk_state.position = j->second;
//                lmk_.emplace(id, lmk_state);
//            }
//        } else {
//            ids.emplace_back(id);
//        }
//    }

//#define ONE_SHOT

    size_t num_obs = 0;
    static std::vector<std::pair<LandmarkID, Landmark*>> ids;
    ids.resize(map_.lmk_map.size());
    ids.clear();

    // 统计观测数量（只处理至少被2个关键帧观测到的landmark）
    for (const auto &it : map_.lmk_map) {
        const auto id = it.first;
        auto lmk = it.second;

        // 关键帧的观测数量
        size_t keyframe_obs = lmk->frm2fet.size();

        // 至少需要2个关键帧观测才能进行滑窗优化
        if (keyframe_obs > 1) {
            ids.emplace_back(id, lmk);
            num_obs += keyframe_obs;
            // TODO: 三角化
            if (!lmk->is_triangulated) {
                lmk->position = lmk_map.at(id);
                lmk->is_triangulated = true;
            }
        }
    }

    // std::cout << "There are " << ids.size() << " Triangulated Landmarks" << std::endl;

    if (ids.empty()) {
        // 关键帧且窗口满才移除一帧
        if (is_keyframe && map_.isWinFull()) {
            popFrame();
        }

        return;
    }


#define USE_QR
//#define USE_SCHUR
#if defined(USE_QR)
    auto t1 = clock();

    constexpr static size_t UV_SIZE = 2;
    MatXX J_POSE = MatXX::Zero(UV_SIZE * WIN_SIZE, AugState::SIZE * WIN_SIZE);
    MatXX J_EXT = MatXX::Zero(UV_SIZE * WIN_SIZE, AugState::SIZE);
    MatXX J_LMK = MatXX::Zero(UV_SIZE * WIN_SIZE, LMK_SIZE);
    VecX ERR = VecX::Zero(UV_SIZE * WIN_SIZE);
    std::vector<FrameOrder> pose_order;
    pose_order.reserve(WIN_SIZE);

    MatXX J_STATE = MatXX::Zero(UV_SIZE * num_obs, AugState::SIZE * WIN_SIZE);
    VecX E_STATE = VecX::Zero(UV_SIZE * num_obs);

    MatXX Q1Jp_s = MatXX::Zero(LMK_SIZE * ids.size(), AugState::SIZE * WIN_SIZE);
    VecX Q1e_s = VecX::Zero(LMK_SIZE * ids.size());
    MatXX RP_s = MatXX::Zero(LMK_SIZE * ids.size(), LMK_SIZE);

    // 遍历 landmark
    size_t row_idx = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i].first;
        auto lmk = ids[i].second;
        pose_order.clear();

        // 遍历关键帧的观测（只处理滑窗内的关键帧）
        for (auto &it : lmk->frm2fet) {
            const auto fet = it.second;
            const auto obs = fet->obs[0];
            const auto frm = fet->frame;

            const auto Rwi = frm->q().toRotationMatrix();
            const auto Ric = ext_.q_ic.toRotationMatrix();
            const auto d_ij_w = lmk->position - frm->p();
            const auto d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const auto d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const auto est = d_cj_c.head<2>() * inv_d;
            const auto err = obs->un_pt.head<2>() - est;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                    TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            Mat2_3 J_lmk = J * (frm->q() * ext_.q_ic).inverse().toRotationMatrix();

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);;
            J_pose.rightCols<3>().noalias() = -J_lmk;

            Mat2_6 J_ext;
            J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
            J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);

            const size_t row_start = UV_SIZE * pose_order.size();
            const size_t col_start = AugState::SIZE * pose_order.size();
            ERR.segment<2>(row_start) = err;
            J_LMK.middleRows<2>(row_start) = J_lmk;
            J_EXT.middleRows<2>(row_start) = J_ext;
            J_POSE.block<2, AugState::SIZE>(row_start, col_start) = J_pose;

            // 记录 J_POSE 中的 J_pose 在 state 中对应的 ordering
            pose_order.emplace_back(frm->ordering);
        }

        // 注意：非关键帧的观测暂不在这里处理，稍后单独更新 landmark
        // Measurement Equation: [J_POSE, J_LMK] * [dxp; dxl] = e
        // QR Decomposition: J_LMK = Q * [R; 0] * P^-1 = [Q1, Q2] * [R; 0] * P^-1
        // We have:
        //  Q1^T * [J_POSE, J_LMK] * dx = [Q1^T * J_POSE, R * P^-1] * [dxp; dxl]
        //                              = Q1^T * J_POSE * dxp + R * P^-1 * dxl
        //                              = Q1^T * e
        // And,
        //  Q2^T * [J_POSE, J_LMK] * dx = [Q2^T * J_POSE, 0] * [dxp; dxl]
        //                              = Q2^T * J_POSE * dxp
        //                              = Q2^T * e
        // 1) Use "Q2^T * J_POSE * dxp = Q2^T * e" to update dxp
        // 2) Then use "R * P^-1 * dxl = Q1^T * e - Q1^T * J_POSE * dxp" to update dxl
        const size_t row_end = UV_SIZE * pose_order.size();
        const size_t col_end = AugState::SIZE * pose_order.size();
        auto &&J_lmk = J_LMK.topRows(row_end);
        auto &&J_pose = J_POSE.topLeftCorner(row_end, col_end);
        auto &&qr_lmk = J_lmk.colPivHouseholderQr();
        auto &&Q = qr_lmk.householderQ();
        const MatXX R = qr_lmk.matrixR().topLeftCorner(LMK_SIZE, LMK_SIZE).template triangularView<Eigen::Upper>();
        auto &&P = qr_lmk.colsPermutation();

        // [Q1^T * e; Q2^T * e]
        auto &&QTe = Q.transpose() * ERR.head(row_end);
        auto &&Q1e = QTe.head(LMK_SIZE);
        auto &&Q2e = QTe.tail(Q.cols() - LMK_SIZE);

        // [Q1^T * J_POSE; Q2^T * J_POSE]
        auto &&QTJp = Q.transpose() * J_pose;
        auto &&Q1Jp = QTJp.topRows(LMK_SIZE);
        auto &&Q2Jp = QTJp.bottomRows(Q.cols() - LMK_SIZE);

        // 存储 Augment State 对应的 Jacobian
        for (size_t j = 0; j < pose_order.size(); ++j) {
            J_STATE.block(row_idx, AugState::SIZE * pose_order[j], Q2Jp.rows(), AugState::SIZE) = Q2Jp.middleCols(AugState::SIZE * j, AugState::SIZE);
        }
        E_STATE.segment(row_idx, Q2e.rows()) = Q2e;

        // 存储 Landmark 相关的信息
        for (size_t j = 0; j < pose_order.size(); ++j) {
            Q1Jp_s.block(LMK_SIZE * i, AugState::SIZE * pose_order[j], Q1Jp.rows(), AugState::SIZE) = Q1Jp.middleCols(AugState::SIZE * j, AugState::SIZE);
        }
        Q1e_s.segment(LMK_SIZE * i, LMK_SIZE) = Q1e;
        RP_s.middleRows(LMK_SIZE * i, LMK_SIZE) = R * P.transpose();

        row_idx += Q2Jp.rows();
    }
//    std::cout << "Update Finished" << std::endl;

    // 对 J_STATE 进行 QR 分解
    auto qr = J_STATE.colPivHouseholderQr();
//    auto Q_red = qr.householderQ() * MatXX::Identity(J_STATE.rows(), J_STATE.cols());
    VecX e_red = (qr.householderQ().transpose() * E_STATE).head(J_STATE.cols());
    const MatXX R_red = qr.matrixR().topLeftCorner(J_STATE.cols(), J_STATE.cols()).template triangularView<Eigen::Upper>();
    const MatXX H_red = R_red * qr.colsPermutation().transpose();

    // 序贯更新 State
    // Q2^T * J_POSE * dxp = Q2^T * e
    auto &&cov_p = cov_;
    VecX dx_p = VecX::Zero(COV_SIZE);
    for (size_t j = 0; j < e_red.rows(); ++j) {
        // 重构出量测矩阵 H
        VecX hT = VecX::Zero(INSState::SIZE + AugState::SIZE * WIN_SIZE);
        hT.tail(AugState::SIZE * WIN_SIZE) = H_red.row(j).transpose();

        TYPE r = uv_var / dt;
        VecX PhT = cov_p * hT;
        TYPE var = hT.dot(PhT) + r;
        VecX K = PhT / var;
        cov_p -= K * PhT.transpose();

        PhT = cov_p * hT;
        cov_p.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
        cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();

        // 修正 e
        auto e = e_red(j) - hT.dot( dx_p);
        dx_p += K * e;
    }
    updateState(dx_p);

    // 更新 Landmarks
    // R * P^-1 * dxl = Q1^T * e - Q1^T * J_POSE * dxp
    // 计算 (Q1^T * e) - (Q1^T * J_POSE) * dxp -> (Q1^T * e)
    Q1e_s -= Q1Jp_s * dx_p.tail(AugState::SIZE * WIN_SIZE);
    VecX dx_l = VecX::Zero(LMK_SIZE);
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i].first;
        auto lmk = ids[i].second;

        auto &&cov_l = lmk->cov_position;
        dx_l.setZero();

        // 计算 R * P^-1 -> RP
        auto &&RP = RP_s.middleRows(i * LMK_SIZE, LMK_SIZE);
        for (size_t j = 0; j < LMK_SIZE; ++j) {
            auto &&hT = RP.row(j).transpose();

            const auto r = uv_var / dt;
            VecX PhT = cov_l * hT;
            TYPE var = hT.dot(PhT) + r;
            VecX K = PhT / var;
            cov_l -= K * PhT.transpose();

            PhT = cov_l * hT;
            cov_l.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
            cov_l.triangularView<Eigen::StrictlyLower>() = cov_l.triangularView<Eigen::StrictlyUpper>().transpose();

            // 修正 e
            auto e = Q1e_s(i * LMK_SIZE + j) - hT.dot(dx_l);
            dx_l += K * e;
        }
        lmk->position += dx_l;
    }

    // 方案4（Zero-copy）：非关键帧的观测直接从 cam_data 读取来 refine landmark，
    // 不创建 Frame / Feature / Observation，也不写入 lmk_map 的持久关联。
    // 位姿直接用当前状态 state_，即"临时帧"的位姿。
    auto t_refine_1 = clock();
    if (!is_keyframe) {
        // 这些量对整帧都是常量，提到循环外
        const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
        const Mat3_3 Rwi_T = state_.orientation.toRotationMatrix().transpose();
        const Mat3_3 Rwc_T = (state_.orientation * ext_.q_ic).inverse().toRotationMatrix();
        const auto &p_wi = state_.position;
        const auto r = uv_var / dt;

        for (const auto &meas : cam_data.measurements) {
            const auto lmk_id = meas.first;

            // 只处理已被关键帧观测过的 landmark
            auto lmk_it = map_.lmk_map.find(lmk_id);
            if (lmk_it == map_.lmk_map.end()) {
                continue;
            }
            auto lmk = lmk_it->second;

            // 至少有 1 个关键帧观测，加上本帧观测才能约束
            if (lmk->frm2fet.empty()) {
                continue;
            }

            // TODO: 三角化
            if (!lmk->is_triangulated) {
                if (auto gt_it = lmk_map.find(lmk_id); gt_it != lmk_map.end()) {
                    lmk->position = gt_it->second;
                    lmk->is_triangulated = true;
                } else {
                    continue;
                }
            }

            // 计算残差和雅可比（观测 meas.second 直接取用，无中间结构）
            const Vec3 d_ij_w = lmk->position - p_wi;
            const Vec3 d_cj_i = Rwi_T * d_ij_w - ext_.t_ic;
            const Vec3 d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const Vec2 est = d_cj_c.head<2>() * inv_d;
            const Vec2 err = meas.second - est;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                    TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            const Mat2_3 J_lmk = J * Rwc_T;

            // 序贯 EKF 更新 landmark（只更新 landmark，不约束 pose）
            auto &&cov_l = lmk->cov_position;
            Vec3 dx_l = Vec3::Zero();

            for (size_t j = 0; j < 2; ++j) {  // 2个残差分量（u, v）
                const Vec3 hT = J_lmk.row(j).transpose();

                Vec3 PhT = cov_l * hT;
                const TYPE var = hT.dot(PhT) + r;
                const Vec3 K = PhT / var;
                cov_l -= K * PhT.transpose();

                PhT = cov_l * hT;
                cov_l.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
                cov_l.triangularView<Eigen::StrictlyLower>() = cov_l.triangularView<Eigen::StrictlyUpper>().transpose();

                const auto e = err(j) - hT.dot(dx_l);
                dx_l += K * e;
            }
            lmk->position += dx_l;
        }
    }
    auto t_refine_2 = clock();
    t_refine_cost_ += t_refine_2 - t_refine_1;
    n_lmk_total_ += ids.size();


    auto t2 = clock();
    t_cost_ += t2 - t1;
    ++posterior_times_;

#elif defined(USE_SCHUR)
    auto t1 = clock();

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

    // 遍历 landmarks
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i].first;
        auto lmk = ids[i].second;

        // 遍历 landmark 的 所有 observations
        for (auto &it : lmk->frm2fet) {
#ifdef ONE_SHOT
            if (it.first != curr_frame_id) {
                continue;
            }
#endif

            const auto fet = it.second;
            const auto obs = fet->obs[0];
            const auto frm = fet->frame;

            const auto Rwi = frm->q().toRotationMatrix();
            const auto Ric = ext_.q_ic.toRotationMatrix();
            const auto d_ij_w = lmk->position - frm->p();
            const auto d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const auto d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const auto est = d_cj_c.head<2>() * inv_d;
            const auto err = obs->un_pt.head<2>() - est;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                    TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            Mat2_3 J_lmk = J * (frm->q() * ext_.q_ic).inverse().toRotationMatrix();

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);;
            J_pose.rightCols<3>().noalias() = -J_lmk;

            Mat2_6 J_ext;
            J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
            J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);

            const size_t lmk_index = LMK_SIZE * i;
            const size_t frm_index = INSState::SIZE + AugState::SIZE * frm->ordering;

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

    // 计算 schur 补
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

#if 1
//    std::cout << "Update State" << std::endl;
    // [[ 更新 State ]]
    // 对 H 使用特征分解: H = V * λ * V^T
    // y = V * λ * V^T * x + V * sqrt(λ) * V^T * n
    // V^T * y = λ * V^T * x + sqrt(λ) * V^T * n
    // Cov[λ * V^T * n] = sqrt(λ) * V^T * Cov[n] * V * sqrt(λ)
    // 如果 Cov[n] = σ^2 * I,
    // 则 Cov[λ * V^T * n] = σ^2 * λ
    // 所以 V^T * y = λ * V^T * x + sqrt(λ) * n, v ~ N[0, σ]
    // 进一步有 λ^-1 * V^T * y = V^T * x + sqrt(λ)^-1 * n, n ~ N[0, σ]
    // 记 w = sqrt(λ)^-1 * n, n ~ N[0, σ]
    // 则有 Cov[w] = σ^2 * λ^-1
    // 序贯 V.col(i)^T * y / λ(i) = V.col(i)^T * x + w(i), var[w] = σ^2 / λ(i)
    VecX dx_p(COV_SIZE);
    dx_p.setZero();
    {
        auto &&cov_p = cov_;
        Eigen::SelfAdjointEigenSolver<decltype(Hpp)> es(Hpp);

        auto &&ep = gp;
//        VecX VTe = es.eigenvectors().transpose() * ep;

        // Step-0: 过滤掉特征值为0的值
        int zero_end = 0;
        for (; zero_end < COV_SIZE; ++zero_end) {
            if (es.eigenvalues()(zero_end) > 1e-6 * es.eigenvalues()(COV_SIZE - 1)) {
                break;
            }
        }
//        while (es.eigenvalues()(zero_end) < 1e-6) {
//            ++zero_end;
//        }
//        std::cout << "State Update: zero_end = " << zero_end << std::endl;
        if (zero_end == COV_SIZE) {
            std::cerr << "eigen value = " << es.eigenvalues().transpose() << std::endl;
        }

        // Step-1: 序贯
        for (; zero_end < COV_SIZE; ++zero_end) {
            const auto R = uv_var / es.eigenvalues()(zero_end) / dt;
            const auto hT = es.eigenvectors().col(zero_end);

            VecX PhT = cov_p * hT;
            TYPE var = hT.dot(PhT) + R;
            VecX K = PhT / var;
            cov_p -= K * PhT.transpose();

            PhT = cov_p * hT;
            cov_p.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();
            cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();

//            // 更新误差 Ve
//            auto dx = K * (VTe(zero_end) / es.eigenvalues()(zero_end));
//            dx_p += dx;
//            VTe -= es.eigenvalues().asDiagonal() * (es.eigenvectors().transpose() * dx);

            auto e = hT.dot(ep / es.eigenvalues()(zero_end) - dx_p);
            dx_p += K * e;
        }
    }
    updateState(dx_p);
//    std::cout << "Update State Finished" << std::endl;

//    std::cout << "Update Landmark" << std::endl;
    // [[ 更新 Landmark ]]
    gl -= Hpl.transpose() * dx_p;
    for (size_t i = 0; i < ids.size(); ++i) {
        auto id = ids[i].first;
        auto lmk = ids[i].second;
        auto index = i * LMK_SIZE;

        VecX dx_l(LMK_SIZE);
        dx_l.setZero();

        auto &&cov_p = lmk->cov_position;
        auto &&hll = Hll.block<3, 3>(index, index);
        Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);

        auto &&el = gl.segment<3>(index);
//        VecX VTe = es.eigenvectors().transpose() * el;

        // Step-0: 过滤掉特征值为0的值
        int zero_end = 0;
        for (; zero_end < LMK_SIZE; ++zero_end) {
            if (es.eigenvalues()(zero_end) > 1e-6 * es.eigenvalues()(LMK_SIZE - 1)) {
                break;
            }
        }
//        while (zero_end < LMK_SIZE && es.eigenvalues()(zero_end) < 1e-6) {
//            ++zero_end;
//        }
#ifdef ONE_SHOT
#else
//        if (zero_end != 0) {
//            std::cerr << "zero_end = " << zero_end << ", eigen value = " << es.eigenvalues().transpose() << std::endl;
//        }
#endif
        if (zero_end == LMK_SIZE) {
            std::cerr << "id = " << id << ", eigen value = " << es.eigenvalues().transpose() << std::endl;
        }

        // Step-1: 序贯
        for (; zero_end < LMK_SIZE; ++zero_end) {
            const auto R = uv_var / es.eigenvalues()(zero_end) / dt;
            const auto hT = es.eigenvectors().col(zero_end);

            VecX PhT = cov_p * hT;
            TYPE var = hT.dot(PhT) + R;
            VecX K = PhT / var;
            cov_p -= K * PhT.transpose();

            PhT = cov_p * hT;
            cov_p.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();
            cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();

//            // 更新误差 Ve
//            auto dx = K * (VTe(zero_end) / es.eigenvalues()(zero_end));
//            dx_l += dx;
//            VTe -= es.eigenvalues().asDiagonal() * (es.eigenvectors().transpose() * dx);

            auto e = hT.dot(el / es.eigenvalues()(zero_end) - dx_l);
            dx_l += K * e;
        }

        lmk->position += dx_l;
    }
//    std::cout << "Update Landmark Finished" << std::endl;
#else
    // 更新 state
    // 量测方程为 gp = Hpp * x + Hpp * n
    VecX dx_p;
    {
//        const auto R = uv_var / dt;
//        MatXX HP = Hpp * cov_;
//
//        MatXX S = cov_;
//        S.diagonal().array() += R;
//        S.triangularView<Eigen::Upper>() = Hpp * S.selfadjointView<Eigen::Upper>() * Hpp.transpose();
//        S.diagonal().array() += 1e-3 * R;
//        S.triangularView<Eigen::StrictlyLower>() = S.triangularView<Eigen::StrictlyUpper>().transpose();
//
//
////    Eigen::SelfAdjointEigenSolver<decltype(S)> es(S);
////    MatXX KT = es.eigenvectors() * ((es.eigenvalues().array() > 0.).select(es.eigenvalues().array().inverse(), 0.).matrix().asDiagonal() * es.eigenvectors().transpose() * HP);
//
//        MatXX KT = S.inverse() * HP;
////    MatXX KT = S.fullPivLu().solve(HP);
//
//        cov_ -= KT.transpose() * HP;
//        cov_ = 0.5 * (cov_ + cov_.transpose());
//
//        dx_p = KT.transpose() * gp;
//        updateState(dx_p);
////        std::cout << "dx = " << dx_p.transpose() << std::endl;

        const auto R = uv_var / dt;
        MatXX PHT = cov_ * Hpp;
        MatXX S = PHT;
        S.diagonal().array() += R;
        MatXX KT = S.inverse() * cov_;

        cov_ -= PHT * KT;
        cov_ = 0.5 * (cov_ + cov_.transpose());

        dx_p = KT.transpose() * gp;
        updateState(dx_p);
    }

    gl -= Hpl.transpose() * dx_p;
    // 更新 landmark
    // 量测方程为 gl = Hll * x + Hll * n
    for (size_t i = 0; i < ids.size(); ++i) {
        auto id = ids[i].first;
        auto lmk = ids[i].second;
        auto index = i * LMK_SIZE;

        auto &&el = gl.segment<3>(index);
        auto &&cov_p = lmk->cov_position;
        auto &&hll = Hll.block<3, 3>(index, index);

//        const auto R = uv_var / dt;
//        MatXX HP = hll * cov_p;
//
//        MatXX S = cov_p;
//        S.diagonal().array() += R;
//        S.triangularView<Eigen::Upper>() = hll * S.selfadjointView<Eigen::Upper>() * hll.transpose();
//        S.diagonal().array() += 1e-3 * R;
//        S.triangularView<Eigen::StrictlyLower>() = S.triangularView<Eigen::StrictlyUpper>().transpose();
//
//        MatXX KT = S.inverse() * HP;
////    MatXX KT = S.fullPivLu().solve(HP);
//
//        cov_p -= KT.transpose() * HP;
//        cov_p = 0.5 * (cov_p + cov_p.transpose());
//
//        VecX dx_l = KT.transpose() * el;

        const auto R = uv_var / dt;
        MatXX PHT = cov_p * hll;

        MatXX S = PHT;
        S.diagonal().array() += R;
        MatXX KT = S.inverse() * cov_p;

        cov_p -= PHT * KT;
        cov_p = 0.5 * (cov_p + cov_p.transpose());

        VecX dx_l = KT.transpose() * el;

        lmk->position += dx_l;
//        std::cout << "id = " << id << ", dx_l = " << dx_l.transpose() << std::endl;
    }
#endif

    // TODO: 更新完后需要固定最老帧率，除非用的是FEJ或OC

    auto t2 = clock();
    t_cost_ += t2 - t1;
    ++posterior_times_;

#else

#endif

    // 方案4：非关键帧无任何临时数据结构，无需清理

    // 移除一帧（仅当是关键帧且窗口已满时才pop滑窗）
    if (is_keyframe && map_.isWinFull()) {
        popFrame();
    }
}

void SchurVINS::updateState(auto &&dx) {
    using I = INSState;
    using A = AugState;

    state_.orientation = (vec2quat(Eigen::Map<Vec3>(dx.data() + I::Q)) * state_.orientation).normalized();
    state_.position += Eigen::Map<Vec3>(dx.data() + I::P);
    state_.velocity += Eigen::Map<Vec3>(dx.data() + I::V);
    state_.gyro_bias += Eigen::Map<Vec3>(dx.data() + I::BG);
    state_.accel_bias += Eigen::Map<Vec3>(dx.data() + I::BA);
    if constexpr (INSState::ESTIMATE_GRAVITY) {
        state_.gravity += Eigen::Map<Vec3>(dx.data() + I::G);
    }
    for (size_t n = 0; n < map_.sfw.size(); ++n) {
//        std::cout << "n = " << n << ", order = " << map_.sfw[n]->ordering << std::endl;
        map_.sfw[n]->q() = (vec2quat(Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::Q)) * map_.sfw[n]->q()).normalized();
        map_.sfw[n]->p() += Eigen::Map<Vec3>(dx.data() + I::SIZE + n * A::SIZE + A::P);
    }
}

void SchurVINS::setQPV(const Quat &q, const Vec3 &p, const Vec3 &v) {
    state_.orientation = q;
    state_.position = p;
    state_.velocity = v;

    Rnb_ = state_.orientation.toRotationMatrix();
}