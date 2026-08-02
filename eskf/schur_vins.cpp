//
// Created by 许家仁 on 2025/8/26.
//

#include "schur_vins.h"
#include "rdvio_constraints.h"
#include "rdvio_scheduler.h"
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

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

SchurVINS::TriangulationResult
SchurVINS::triangulateLandmark(const Landmark &landmark) const {
    const auto started = std::chrono::steady_clock::now();
    TriangulationResult result;

    auto finish = [&](const TriangulationStatus status) {
        result.status = status;
        result.elapsed_us = static_cast<TYPE>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - started).count()) * TYPE(1e-3);
        return result;
    };

    struct View {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        const Frame *frame{};
        Vec2 measurement{Vec2::Zero()};
        Vec3 camera_center{Vec3::Zero()};
        Vec3 bearing_world{Vec3::Zero()};
        Mat3_3 Rwc{Mat3_3::Identity()};
        Mat6_6 relative_pose_covariance{Mat6_6::Zero()};
    };

    std::vector<View, Eigen::aligned_allocator<View>> views;
    views.reserve(landmark.frm2fet.size());
    const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();

    // 先把数据结构中的观测转换成统一的几何量：世界系相机中心 C_i 和单位视线 d_i。
    // 后续线性初始化、视差判断和非线性重投影都复用它们，避免循环中重复做外参变换。
    for (const auto &[frame_id, feature] : landmark.frm2fet) {
        (void)frame_id;
        if (!feature || !feature->frame || !feature->obs[0]) {
            continue;
        }
        const auto *frame = feature->frame;
        const Vec3 bearing_camera = feature->obs[0]->un_pt.normalized();
        if (!bearing_camera.allFinite() || bearing_camera.z() <= TYPE(0)) {
            continue;
        }

        View view;
        view.frame = frame;
        view.measurement = feature->obs[0]->un_pt.head<2>();
        const Mat3_3 Rwi = frame->q().toRotationMatrix();
        view.Rwc.noalias() = Rwi * Ric;
        view.camera_center.noalias() = frame->p() + Rwi * ext_.t_ic;
        view.bearing_world.noalias() = view.Rwc * bearing_camera;
        view.bearing_world.normalize();
        views.emplace_back(view);
    }

    result.observation_count = views.size();
    if (views.size() < 2) {
        return finish(TriangulationStatus::InsufficientViews);
    }

    // Sort chronologically. Physical covariance slots are reused by compact
    // schedulers and therefore are not a valid temporal ordering.
    std::sort(views.begin(), views.end(), [](const View &lhs, const View &rhs) {
        return lhs.frame->timestamp < rhs.frame->timestamp;
    });

    // 两条世界系单位视线的夹角 theta_ij=acos(d_i^T d_j) 决定深度条件数。
    // 使用所有观测对的最大夹角，而不是只比较首末帧，可兼容轨迹回头或观测中断。
    TYPE max_parallax = TYPE(0);
    size_t best_view_i = 0;
    size_t best_view_j = 1;
    for (size_t i = 0; i + 1 < views.size(); ++i) {
        for (size_t j = i + 1; j < views.size(); ++j) {
            const TYPE cosine = std::clamp(
                views[i].bearing_world.dot(views[j].bearing_world), TYPE(-1), TYPE(1));
            const TYPE parallax = std::acos(cosine);
            if (parallax > max_parallax) {
                max_parallax = parallax;
                best_view_i = i;
                best_view_j = j;
            }
        }
    }
    result.max_parallax_deg = max_parallax * TYPE(180) / std::numbers::pi_v<TYPE>;
    if (result.max_parallax_deg < triangulation_min_parallax_deg) {
        return finish(TriangulationStatus::LowParallax);
    }

    // 最大视差观测对对深度最敏感。在这两个候选中选择位姿协方差 trace 更小的
    // clone 作锚点；锚点仅用于不确定度传播，不会被当成额外量测。
    auto poseCovarianceTrace = [&](const size_t view_index) {
        const size_t state_index = INSState::SIZE +
            AugState::SIZE * views[view_index].frame->ordering;
        return cov_.block<6, 6>(state_index, state_index).trace();
    };
    const size_t anchor_view = poseCovarianceTrace(best_view_i)
        <= poseCovarianceTrace(best_view_j) ? best_view_i : best_view_j;
    const size_t anchor_index = INSState::SIZE +
        AugState::SIZE * views[anchor_view].frame->ordering;
    const Mat6_6 anchor_covariance = cov_.block<6, 6>(anchor_index, anchor_index);

    // 使用 clone 间交叉块构造相对协方差：
    // P(δx_i-δx_a)=P_ii+P_aa-P_ia-P_ai，可抵消共同模态。
    for (auto &view : views) {
        const size_t pose_index = INSState::SIZE +
                                  AugState::SIZE * view.frame->ordering;
        view.relative_pose_covariance =
            cov_.block<6, 6>(pose_index, pose_index) + anchor_covariance
            - cov_.block<6, 6>(pose_index, anchor_index)
            - cov_.block<6, 6>(anchor_index, pose_index);
        view.relative_pose_covariance = TYPE(0.5) *
            (view.relative_pose_covariance + view.relative_pose_covariance.transpose());
    }

    // 射线最小二乘初始化：点 P 到射线 (C_i,d_i) 的垂直残差为
    //   e_i=(I-d_i d_i^T)(P-C_i)。
    // 最小化 sum ||e_i||^2 后的正规方程为
    //   sum(I-d_i d_i^T) P = sum(I-d_i d_i^T) C_i。
    // 当所有视线近似平行时，沿视线方向的特征值趋近 0，因此求解前必须检查条件数。
    Mat3_3 ray_hessian = Mat3_3::Zero();
    Vec3 ray_gradient = Vec3::Zero();
    for (const auto &view : views) {
        const Mat3_3 normal_projector =
            Mat3_3::Identity() - view.bearing_world * view.bearing_world.transpose();
        ray_hessian.noalias() += normal_projector;
        ray_gradient.noalias() += normal_projector * view.camera_center;
    }
    ray_hessian = TYPE(0.5) * (ray_hessian + ray_hessian.transpose());

    Eigen::SelfAdjointEigenSolver<Mat3_3> ray_es(ray_hessian);
    if (ray_es.info() != Eigen::Success || !ray_es.eigenvalues().allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }
    const TYPE ray_min = ray_es.eigenvalues().minCoeff();
    const TYPE ray_max = ray_es.eigenvalues().maxCoeff();
    if (!(ray_max > TYPE(0)) || ray_min <= ray_max * TYPE(1e-8)) {
        return finish(TriangulationStatus::IllConditioned);
    }
    result.condition_number = ray_max / ray_min;
    Vec3 position = ray_es.eigenvectors()
                  * ray_es.eigenvalues().cwiseInverse().asDiagonal()
                  * ray_es.eigenvectors().transpose() * ray_gradient;
    if (!position.allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }

    const TYPE image_variance = std::max(
        triangulation_uv_std * triangulation_uv_std, TYPE(1e-12));

    // 构造带相对位姿不确定度的重投影信息。不构造所有 clone 的稠密联合残差
    // 协方差，以控制每个 landmark 的初始化开销；所得是相对锚点条件下的工程近似。
    auto accumulateReprojection = [&](const Vec3 &point,
                                      Mat3_3 &information,
                                      Vec3 &gradient,
                                      TYPE &squared_error,
                                      TYPE &normalized_error) {
        information.setZero();
        gradient.setZero();
        squared_error = TYPE(0);
        normalized_error = TYPE(0);

        for (const auto &view : views) {
            const Vec3 d_camera = view.Rwc.transpose() * (point - view.camera_center);
            if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
                return false;
            }

            const TYPE inv_depth = TYPE(1) / d_camera.z();
            const TYPE inv_depth2 = inv_depth * inv_depth;
            const Vec2 estimate = d_camera.head<2>() * inv_depth;
            const Vec2 residual = view.measurement - estimate;
            if (!residual.allFinite()) {
                return false;
            }

            // 归一化针孔投影 pi([x,y,z])=[x/z,y/z] 的雅可比；随后分别链式得到
            // 对世界点和 clone 位姿误差 [delta_theta,delta_p] 的雅可比。
            Mat2_3 J_projection;
            J_projection << inv_depth, TYPE(0), -d_camera.x() * inv_depth2,
                            TYPE(0), inv_depth, -d_camera.y() * inv_depth2;
            const Mat2_3 J_landmark = J_projection * view.Rwc.transpose();

            const Vec3 d_world = point - view.frame->p();
            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_landmark * hat(d_world);
            J_pose.rightCols<3>().noalias() = -J_landmark;

            // S_i = sigma_uv^2 I + J_pose P_rel J_pose^T。
            // 这里用相对 clone 协方差，避免把所有帧共有的全局 gauge 不确定度反复计入。
            Mat2_2 residual_covariance =
                J_pose * view.relative_pose_covariance * J_pose.transpose();
            residual_covariance = TYPE(0.5) *
                                  (residual_covariance + residual_covariance.transpose());
            residual_covariance.diagonal().array() += image_variance;
            if (!residual_covariance.allFinite()) {
                return false;
            }

            // 防止协方差仅因浮点舍入出现极小负特征值；真实的大负值会在后续条件检查失败。
            const TYPE trace = residual_covariance.trace();
            const TYPE discriminant = std::sqrt(std::max(
                TYPE(0),
                (residual_covariance(0, 0) - residual_covariance(1, 1)) *
                (residual_covariance(0, 0) - residual_covariance(1, 1)) +
                TYPE(4) * residual_covariance(0, 1) * residual_covariance(0, 1)));
            const TYPE min_eigenvalue = TYPE(0.5) * (trace - discriminant);
            if (min_eigenvalue < image_variance * TYPE(0.1)) {
                residual_covariance.diagonal().array() +=
                    image_variance * TYPE(0.1) - min_eigenvalue;
            }

            const TYPE determinant = residual_covariance.determinant();
            if (!(determinant > image_variance * image_variance * TYPE(1e-6)) ||
                !std::isfinite(determinant)) {
                return false;
            }
            const Mat2_2 weight_matrix = residual_covariance.inverse();
            // 在白化残差上使用 3-sigma Huber。正常观测保持二次代价；异常观测的
            // 影响随 1/||r|| 衰减，但仍保留方向信息，最终再由重投影硬门限判成失败。
            const TYPE mahalanobis2 = std::max(TYPE(0), residual.dot(weight_matrix * residual));
            const TYPE mahalanobis = std::sqrt(mahalanobis2);
            const TYPE huber_weight = mahalanobis > TYPE(3) ? TYPE(3) / mahalanobis : TYPE(1);

            information.noalias() +=
                J_landmark.transpose() * (huber_weight * weight_matrix) * J_landmark;
            gradient.noalias() +=
                J_landmark.transpose() * (huber_weight * weight_matrix) * residual;
            squared_error += residual.squaredNorm();
            normalized_error += mahalanobis2;
        }
        information = TYPE(0.5) * (information + information.transpose());
        return information.allFinite() && gradient.allFinite();
    };

    Mat3_3 information;
    Vec3 gradient;
    TYPE squared_error = TYPE(0);
    TYPE normalized_error = TYPE(0);
    // 以射线解为初值做最多 5 次 Gauss-Newton。当前 residual=measurement-estimate，
    // 且 J_landmark=d(estimate)/dP，所以正规方程是 H*delta=J^T W residual，更新 P+=delta。
    for (size_t iteration = 0; iteration < 5; ++iteration) {
        if (!accumulateReprojection(position, information, gradient,
                                    squared_error, normalized_error)) {
            return finish(TriangulationStatus::NegativeDepth);
        }
        Eigen::LDLT<Mat3_3> ldlt(information);
        if (ldlt.info() != Eigen::Success ||
            ldlt.vectorD().minCoeff() <= ldlt.vectorD().maxCoeff() * TYPE(1e-10)) {
            return finish(TriangulationStatus::IllConditioned);
        }
        const Vec3 increment = ldlt.solve(gradient);
        if (!increment.allFinite() || increment.norm() > TYPE(100)) {
            return finish(TriangulationStatus::IllConditioned);
        }
        position += increment;
        if (increment.norm() < TYPE(1e-6)) {
            break;
        }
    }

    if (!accumulateReprojection(position, information, gradient,
                                squared_error, normalized_error)) {
        return finish(TriangulationStatus::NegativeDepth);
    }
    result.reprojection_rmse = std::sqrt(
        squared_error / static_cast<TYPE>(TYPE(2) * views.size()));
    if (result.reprojection_rmse > triangulation_max_reprojection_rmse) {
        return finish(TriangulationStatus::HighReprojectionError);
    }

    Eigen::SelfAdjointEigenSolver<Mat3_3> information_es(information);
    if (information_es.info() != Eigen::Success ||
        !information_es.eigenvalues().allFinite()) {
        return finish(TriangulationStatus::IllConditioned);
    }
    const TYPE information_min = information_es.eigenvalues().minCoeff();
    const TYPE information_max = information_es.eigenvalues().maxCoeff();
    if (!(information_min > TYPE(0)) || information_min <= information_max * TYPE(1e-12)) {
        return finish(TriangulationStatus::IllConditioned);
    }
    result.condition_number = information_max / information_min;

    // 条件信息矩阵给出 P_l|poses = Lambda^-1；若白化残差的 chi^2/dof>1，
    // 再按该比例膨胀协方差，防止模型失配时给出虚假的高精度初值。
    const TYPE degrees_of_freedom = std::max<TYPE>(
        TYPE(1), TYPE(2) * static_cast<TYPE>(views.size()) - TYPE(3));
    const TYPE covariance_scale = std::max(TYPE(1), normalized_error / degrees_of_freedom);
    Vec3 covariance_eigenvalues =
        covariance_scale * information_es.eigenvalues().cwiseInverse();
    covariance_eigenvalues = covariance_eigenvalues.cwiseMax(TYPE(1e-10));
    if (std::sqrt(covariance_eigenvalues.maxCoeff()) > triangulation_max_position_std) {
        return finish(TriangulationStatus::ExcessiveUncertainty);
    }

    result.position = position;
    const Mat3_3 conditional_covariance = information_es.eigenvectors()
                                          * covariance_eigenvalues.asDiagonal()
                                          * information_es.eigenvectors().transpose();

    // position 存在世界坐标系中，因此在相对锚点条件协方差之外，还要传播锚点本身的
    // 绝对位姿不确定度。质量门限在上面只检查 conditional_covariance，避免全局 gauge
    // 不确定度把几何上可靠的点拒绝掉。
    Mat3_6 J_anchor_world;
    J_anchor_world.leftCols<3>() = -hat(position - views[anchor_view].frame->p());
    J_anchor_world.rightCols<3>().setIdentity();
    result.covariance = conditional_covariance
                      + J_anchor_world * anchor_covariance * J_anchor_world.transpose();
    result.covariance = TYPE(0.5) * (result.covariance + result.covariance.transpose());
    return finish(TriangulationStatus::Success);
}

void SchurVINS::logTriangulationAttempt(
        Landmark &landmark,
        const TriangulationResult &result,
        const Tus timestamp,
        const std::unordered_map<size_t, Vec3> &ground_truth) {
    if (!enable_logging_) {
        return;
    }

    TriangulationLog log;
    log.timestamp = timestamp;
    log.id = landmark.id;
    log.status = result.status;
    log.observation_count = result.observation_count;
    log.max_parallax_deg = result.max_parallax_deg;
    log.condition_number = result.condition_number;
    log.reprojection_rmse = result.reprojection_rmse;
    log.elapsed_us = result.elapsed_us;
    if (result.status == TriangulationStatus::Success) {
        log.initial_position = result.position;
        log.latest_position = result.position;
        log.initial_covariance = result.covariance;
        log.latest_covariance = result.covariance;
        log.initial_cov_trace = result.covariance.trace();
    }
    if (const auto gt = ground_truth.find(landmark.id); gt != ground_truth.end()) {
        log.ground_truth = gt->second;
        log.has_ground_truth = true;
        if (result.status == TriangulationStatus::Success) {
            const Vec3 error = result.position - gt->second;
            Eigen::LDLT<Mat3_3> ldlt(result.covariance);
            if (ldlt.info() == Eigen::Success && ldlt.vectorD().minCoeff() > TYPE(0)) {
                log.initial_nees = std::max(TYPE(0), error.dot(ldlt.solve(error)));
            }
        }
    }
    triangulation_logs_.emplace_back(log);
    if (result.status == TriangulationStatus::Success) {
        landmark.triangulation_log_index = triangulation_logs_.size() - 1;
    }
}

void SchurVINS::recordLandmarkRefinement(const Landmark &landmark, bool shadow) {
    if (!enable_logging_ || landmark.triangulation_log_index >= triangulation_logs_.size()) {
        return;
    }
    auto &log = triangulation_logs_[landmark.triangulation_log_index];
    if (shadow) {
        log.shadow_initialized = landmark.shadow_initialized;
        log.shadow_position = landmark.shadow_position;
        log.shadow_covariance = landmark.shadow_cov_position;
        ++log.shadow_refinement_count;
    } else {
        log.latest_position = landmark.position;
        log.latest_covariance = landmark.cov_position;
        ++log.refinement_count;
    }
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
        // 先就地计算 P * F^T。改动完整列可同时传播 P_ii 与 P_ci，
        // 之后只需计算 P_ii 的非平凡行，P_ic 由 P_ci^T 恢复。
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

    // 叠加过程噪声
    cov += (state_.var_proc * (dt * proc_noise_scale_)).asDiagonal();

    // 更新时间戳
    state_.timestamp = imu_data.timestamp;
}

void SchurVINS::pushFrame(const CameraData &cam_data, bool is_keyframe) {
    using A = AugState;

    if (!is_keyframe && !schedulerAugmentsEveryImage()) {
        // Legacy mode keeps the original keyframe-only clone policy.
        return;
    }

    // Create either a persistent keyframe clone or a temporal clone according
    // to the compile-time visual scheduler.
    auto frm = map_.pushFrame(cam_data.timestamp, is_keyframe);
    frm->timestamp = state_.timestamp;
    frm->q() = state_.orientation;
    frm->p() = state_.position;
    // First-estimate Jacobian reference. The nominal pose will continue to be
    // corrected, while this copy remains fixed for the OC nullspace basis.
    frm->record_to_state_fej();

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

void SchurVINS::popFrame(const size_t chronological_index) {
//    // TODO: 加入选择策略
//    const auto idx = (latest_free_sfw_idx_ + 1) % WIN_SIZE;
//    free_sfw_idx_.emplace_back(idx);
//    return sfw_[idx];
    map_.popFrame(chronological_index);
}

void SchurVINS::updateMap(const slam::CameraData &cam_data) {

}

void SchurVINS::updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, const double dt) {
    bool is_keyframe = map_.isKeyFrame(cam_data);
    bool is_rotation_frame = false;
    TYPE rdvio_misalignment_deg = TYPE(180);
    uint8_t rdvio_case = 0;

    if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
        const RDVIOParameters parameters{
            rdvio_rotation_threshold_deg,
            rdvio_min_common_tracks,
            rdvio_subframe_size,
            rdvio_rotation_compression_trigger,
            schedulerRetainedCloneCount()};
        const RDVIOFrameDecision decision = decideRDVIOFrame(
            map_, cam_data, state_.orientation, ext_.q_ic,
            is_keyframe, parameters);
        is_keyframe = decision.is_keyframe;
        is_rotation_frame = decision.is_rotation_frame;
        rdvio_misalignment_deg = decision.misalignment_deg;
        rdvio_case = static_cast<uint8_t>(decision.transition);
        if (decision.promote_previous_to_keyframe && !map_.sfw.empty()) {
            map_.sfw[map_.sfw.size() - 1]->is_key_frame = true;
        }
        if (decision.transition != RDVIOCase::None) {
            ++n_rdvio_cases_[rdvio_case];
        }
        if (is_rotation_frame) {
            ++n_rdvio_rotation_frames_;
        } else {
            ++n_rdvio_normal_frames_;
        }
    }
    const bool store_current_frame = is_keyframe || schedulerAugmentsEveryImage();
    Frame *current_frame = nullptr;
    if (store_current_frame) {
        pushFrame(cam_data, is_keyframe);
        current_frame = map_.getWinLatestFrame();
        current_frame->is_rotation_frame = is_rotation_frame;
        current_frame->rdvio_case = rdvio_case;
        current_frame->rdvio_misalignment_deg = rdvio_misalignment_deg;
        map_.addObservations(current_frame, cam_data);
    }

    const size_t current_win_size = map_.sfw.size();
    std::vector<size_t> frames_to_remove;
    size_t rdvio_compressed_frames = 0;
    if constexpr (visual_update_scheduler == VisualUpdateScheduler::Legacy) {
        if (is_keyframe && map_.isWinFull()) {
            frames_to_remove.push_back(0);
        }
    } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::SchurVINS) {
        if (current_win_size > schedulerRetainedCloneCount()) {
            // Retain the newest image and then prefer the two newest keyframes.
            // Any remaining slot is filled by the newest temporal frame. This
            // yields the paper's compact 2-keyframe + recent-frame update set.
            std::vector<bool> keep(current_win_size, false);
            keep.back() = true;
            size_t kept = 1;
            for (size_t i = current_win_size; i > 0 && kept <
                 schedulerRetainedCloneCount(); --i) {
                const size_t index = i - 1;
                if (!keep[index] && map_.sfw[index]->is_key_frame) {
                    keep[index] = true;
                    ++kept;
                }
            }
            for (size_t i = current_win_size; i > 0 && kept <
                 schedulerRetainedCloneCount(); --i) {
                const size_t index = i - 1;
                if (!keep[index]) {
                    keep[index] = true;
                    ++kept;
                }
            }
            for (size_t i = 0; i < current_win_size; ++i) {
                if (!keep[i]) {
                    frames_to_remove.push_back(i);
                    break;
                }
            }
        }
    } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::MSCKF) {
        if (current_win_size > schedulerRetainedCloneCount()) {
            frames_to_remove.push_back(0);
        }
    } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::VINSMono) {
        if (current_win_size > schedulerRetainedCloneCount()) {
            const size_t second_newest = current_win_size - 2;
            frames_to_remove.push_back(map_.sfw[second_newest]->is_key_frame
                ? size_t(0) : second_newest);
        }
    } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
        // The ESKF has no between-clone preintegration factors to concatenate;
        // consuming tracks touching removed clones preserves their lifecycle.
        const RDVIOParameters parameters{
            rdvio_rotation_threshold_deg,
            rdvio_min_common_tracks,
            rdvio_subframe_size,
            rdvio_rotation_compression_trigger,
            schedulerRetainedCloneCount()};
        RDVIOFrameRemovalPlan removal_plan =
            planRDVIOFrameRemovals(map_, parameters);
        frames_to_remove = std::move(removal_plan.chronological_indices);
        rdvio_compressed_frames = removal_plan.compressed_frame_count;
    }

    std::sort(frames_to_remove.begin(), frames_to_remove.end());
    frames_to_remove.erase(
        std::unique(frames_to_remove.begin(), frames_to_remove.end()),
        frames_to_remove.end());
    std::vector<FrameID> frame_ids_to_remove;
    frame_ids_to_remove.reserve(frames_to_remove.size());
    for (const size_t index : frames_to_remove) {
        frame_ids_to_remove.push_back(map_.sfw[index]->id);
    }

    std::vector<LandmarkID> tracks_to_consume;
    size_t one_shot_tracks_used = 0;
    ExitHandler schedule_finalizer([&] {
        if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            n_tracks_consumed_ += tracks_to_consume.size();
            n_tracks_dropped_ += tracks_to_consume.size() - one_shot_tracks_used;
            for (const LandmarkID id : tracks_to_consume) {
                map_.removeLandmark(id);
            }
        }
        if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
            n_rdvio_compressed_frames_ += rdvio_compressed_frames;
        }
        for (auto index = frames_to_remove.rbegin();
             index != frames_to_remove.rend(); ++index) {
            popFrame(*index);
        }
    });

    if (current_win_size < 2) {
        ++n_visual_updates_skipped_;
        return;
    }

    auto observedInFrame = [](const Landmark &landmark, const Frame *frame) {
        return frame && landmark.frm2fet.find(frame->id) != landmark.frm2fet.end();
    };
    auto isSchurVinsActiveTrack = [&](const Landmark &landmark) {
        const size_t recent_count = std::min<size_t>(2, map_.sfw.size());
        for (size_t offset = 0; offset < recent_count; ++offset) {
            if (observedInFrame(landmark, map_.sfw[map_.sfw.size() - 1 - offset])) {
                return true;
            }
        }
        return false;
    };

    size_t num_obs = 0;
    static std::vector<std::pair<LandmarkID, Landmark*>> ids;
    ids.resize(map_.lmk_map.size());
    ids.clear();
    std::vector<Landmark *> rotation_only_tracks;

    for (const auto &[id, lmk] : map_.lmk_map) {
        const size_t observation_count = lmk->frm2fet.size();
        bool schedule_track = true;
        bool consume_track = false;

        if constexpr (visual_update_scheduler == VisualUpdateScheduler::SchurVINS) {
            schedule_track = isSchurVinsActiveTrack(*lmk);
        } else if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            const bool observed_current = observedInFrame(*lmk, current_frame);
            const bool lost = current_frame && !observed_current;
            const bool touches_marginalized_clone = std::any_of(
                frame_ids_to_remove.begin(), frame_ids_to_remove.end(),
                [&](const FrameID frame_id) {
                    return lmk->frm2fet.find(frame_id) != lmk->frm2fet.end();
                });
            const bool reached_track_limit =
                observation_count >= schedulerRetainedCloneCount();
            consume_track = lost || touches_marginalized_clone || reached_track_limit;
            schedule_track = consume_track;
            if (consume_track) {
                tracks_to_consume.push_back(id);
            }
        }

        if (!schedule_track || observation_count <= 1) {
            continue;
        }

        if (!lmk->is_triangulated &&
            !(visual_update_scheduler == VisualUpdateScheduler::RDVIO &&
              current_frame && current_frame->is_rotation_frame) &&
            lmk->last_triangulation_obs_count < observation_count) {
            lmk->last_triangulation_obs_count = observation_count;
            if (landmark_initialization_mode_ == LandmarkInitializationMode::GroundTruth) {
                const auto truth = lmk_map.find(id);
                if (truth != lmk_map.end()) {
                    lmk->position = truth->second;
                    lmk->cov_position = Mat3_3::Identity() * TYPE(1e-4);
                    lmk->shadow_position = lmk->position;
                    lmk->shadow_cov_position = lmk->cov_position;
                    lmk->shadow_initialized = true;
                    lmk->is_triangulated = true;
                }
            } else {
                const auto triangulation = triangulateLandmark(*lmk);
                logTriangulationAttempt(*lmk, triangulation, cam_data.timestamp, lmk_map);
                if (triangulation.status == TriangulationStatus::Success) {
                    lmk->position = triangulation.position;
                    lmk->cov_position = triangulation.covariance;
                    if (landmark_initialization_mode_ ==
                        LandmarkInitializationMode::TriangulationWithOraclePosition) {
                        const auto truth = lmk_map.find(id);
                        if (truth != lmk_map.end()) {
                            lmk->position = truth->second;
                        }
                    }
                    lmk->shadow_position = lmk->position;
                    lmk->shadow_cov_position = lmk->cov_position;
                    lmk->shadow_initialized = true;
                    lmk->is_triangulated = true;
                }
            }
        }

        if (lmk->is_triangulated) {
            ids.emplace_back(id, lmk);
            num_obs += observation_count;
            if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
                ++one_shot_tracks_used;
            }
        } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
            if (consume_track && observation_count > 1) {
                rotation_only_tracks.push_back(lmk);
            }
        }
    }

    const bool has_rdvio_zero_translation =
        visual_update_scheduler == VisualUpdateScheduler::RDVIO &&
        current_frame && current_frame->is_rotation_frame &&
        map_.sfw.size() >= 2;
    if (ids.empty() && rotation_only_tracks.empty() &&
        !has_rdvio_zero_translation) {
        ++n_visual_updates_skipped_;
        return;
    }

    // Repeated-window legacy modes interpret uv_var as an information-density
    // parameter and integrate it with camera dt. A SchurVINS paper-style batch
    // and especially an MSCKF one-shot track instead represent actual image
    // samples, so their covariance is the normalized image variance itself.
    const TYPE image_variance = std::max(
        triangulation_uv_std * triangulation_uv_std, TYPE(1e-12));
    const TYPE visual_batch_variance = [&] {
        if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            return image_variance * std::max(msckf_visual_noise_scale, TYPE(1));
        } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::SchurVINS) {
            return image_variance;
        } else {
            return uv_var / std::max(TYPE(dt), TYPE(1e-6));
        }
    }();


// 当前项目的统一验证/报告路径。QR 实现保留用于历史 A/B 对照，但默认不编译执行。
// 后续视觉算法修改应先保证 USE_SCHUR 路径正确，再按需单独回归 QR。
//#define USE_QR
#define USE_SCHUR
#if defined(USE_QR) && defined(USE_SCHUR)
#error "USE_QR and USE_SCHUR are mutually exclusive"
#endif
#if defined(USE_QR)
    auto t1 = clock();

    constexpr static size_t UV_SIZE = 2;
    MatXX J_POSE = MatXX::Zero(UV_SIZE * WIN_SIZE, AugState::SIZE * WIN_SIZE);
    // 外参雅可比: 仅在估计外参时才分配(见 ExtState::ESTIMATE_EXTRINSIC)
    MatXX J_EXT = MatXX::Zero(ExtState::ESTIMATE_EXTRINSIC ? UV_SIZE * WIN_SIZE : 0,
                              ExtState::ESTIMATE_EXTRINSIC ? ExtState::SIZE : 0);
    MatXX J_LMK = MatXX::Zero(UV_SIZE * WIN_SIZE, LMK_SIZE);
    VecX ERR = VecX::Zero(UV_SIZE * WIN_SIZE);
    std::vector<FrameOrder> pose_order;
    pose_order.reserve(WIN_SIZE);

    MatXX J_STATE = MatXX::Zero(UV_SIZE * num_obs, AugState::SIZE * WIN_SIZE);
    VecX E_STATE = VecX::Zero(UV_SIZE * num_obs);

    MatXX Q1Jp_s = MatXX::Zero(LMK_SIZE * ids.size(), AugState::SIZE * WIN_SIZE);
    VecX Q1e_s = VecX::Zero(LMK_SIZE * ids.size());
    MatXX RP_s = MatXX::Zero(LMK_SIZE * ids.size(), LMK_SIZE);

    auto t_stage1 = clock();

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

            const size_t row_start = UV_SIZE * pose_order.size();
            const size_t col_start = AugState::SIZE * pose_order.size();
            ERR.segment<2>(row_start) = err;
            J_LMK.middleRows<2>(row_start) = J_lmk;
            J_POSE.block<2, AugState::SIZE>(row_start, col_start) = J_pose;

            // 外参雅可比: 保留代码但默认不运行(外参目前不在状态里，J_EXT 无人读取)。
            // 用 if constexpr 而非 #ifdef，这样它始终参与语法/类型检查，不会腐烂。
            if constexpr (ExtState::ESTIMATE_EXTRINSIC) {
                Mat2_6 J_ext;
                J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
                J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);
                J_EXT.middleRows<2>(row_start) = J_ext;
            }

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

    auto t_stage2 = clock();
    t_perlmk_qr_ += t_stage2 - t_stage1;

    // 对 J_STATE 进行 QR 分解
    //
    // 优化点1: 用 householderQr 取代 colPivHouseholderQr。
    //   列选主元对 (2*num_obs) x 180 的矩阵开销很大，而这里不需要 rank-revealing:
    //   即使 J_STATE 降秩，R 的对应行会趋于 0，序贯更新时 hT ~ 0 => K ~ 0，
    //   该行自然不贡献修正量，退化是平滑的。同时省掉了 R * P^T 这次乘法。
    //
    // 优化点2: 把 E_STATE 拼成增广矩阵 [J_STATE, E_STATE] 一起分解，
    //   R 的最后一列前 n 行即 (Q^T * E_STATE).head(n)，省掉单独应用一次
    //   Householder 序列。同时只对前 row_idx 行分解（后面是未填充的零行）。
    //   注: 实测这两点对总耗时无可测量的影响(提速几乎全部来自优化点1)，
    //   保留是因为省了一趟 O(m*n) 运算且代码更紧凑，不是性能考虑。
    const size_t n_cols = AugState::SIZE * WIN_SIZE;
    const size_t m_eff = row_idx;
    const size_t n_eff = std::min(m_eff, n_cols);

    MatXX JE = MatXX::Zero(m_eff, n_cols + 1);
    JE.leftCols(n_cols) = J_STATE.topRows(m_eff);
    JE.col(n_cols) = E_STATE.head(m_eff);

    auto qr = JE.householderQr();
    auto &&QR = qr.matrixQR();

    // R_red: n_eff x n_cols 的上三角部分；H_red 不再需要乘 permutation
    const MatXX H_red = QR.topLeftCorner(n_eff, n_cols).template triangularView<Eigen::Upper>();

    // e_red: 增广列的前 n_eff 行，即 (Q^T * E_STATE).head(n_eff)
    const VecX e_red = QR.col(n_cols).head(n_eff);

    auto t_stage3 = clock();
    t_bigqr_ += t_stage3 - t_stage2;

    // 序贯更新 State
    // Q2^T * J_POSE * dxp = Q2^T * e
    auto &&cov_p = cov_;
    VecX dx_p = VecX::Zero(COV_SIZE);
    VecX hT = VecX::Zero(INSState::SIZE + AugState::SIZE * WIN_SIZE);
    for (size_t j = 0; j < n_eff; ++j) {
        // 重构出量测矩阵 H（INS 部分恒为 0，只需重写 tail）
        hT.tail(AugState::SIZE * WIN_SIZE) = H_red.row(j).transpose();

        TYPE r = visual_batch_variance;
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

    auto t_stage4 = clock();
    t_seq_state_ += t_stage4 - t_stage3;
    n_seq_rows_ += n_eff;

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

            const auto r = visual_batch_variance;
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
        recordLandmarkRefinement(*lmk);
    }

    // 方案4（Zero-copy）：非关键帧的观测直接从 cam_data 读取来 refine landmark，
    // 不创建 Frame / Feature / Observation，也不写入 lmk_map 的持久关联。
    // 位姿直接用当前状态 state_，即"临时帧"的位姿。
    auto t_refine_1 = clock();
    t_lmk_update_ += t_refine_1 - t_stage4;
    if (!is_keyframe) {
        // 这些量对整帧都是常量，提到循环外
        const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
        const Mat3_3 Rwi_T = state_.orientation.toRotationMatrix().transpose();
        const Mat3_3 Rwc_T = (state_.orientation * ext_.q_ic).inverse().toRotationMatrix();
        const auto &p_wi = state_.position;
        const auto r = visual_batch_variance;

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

            if (!lmk->is_triangulated) {
                continue;
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
            recordLandmarkRefinement(*lmk);
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

    // [数据采集] 记录视觉更新【前】的先验状态
    UpdateLog log{};
    if (enable_logging_) {
        log.timestamp = cam_data.timestamp;
        log.p_prior = state_.position;
        log.v_prior = state_.velocity;
        log.q_prior = state_.orientation;
        log.n_lmk = ids.size();
        log.win_size = map_.sfw.size();
        log.is_keyframe = is_keyframe;
        log.is_rotation_frame = is_rotation_frame;
        log.rdvio_case = rdvio_case;
        log.rdvio_misalignment_deg = rdvio_misalignment_deg;
    }

    // Hessian 矩阵
    //
    // 优化: Hll 是块对角矩阵(landmark 之间没有直接耦合，只通过 pose 间接耦合)，
    //   原本按 lmk_size x lmk_size 稠密分配 = (3*326)^2 ~ 96万个 double,
    //   而实际只用到对角线上 326 个 3x3 块 = 2934 个。
    //   改成只存对角块: lmk_size x 3，省掉 99.7% 的分配和清零。
    const auto lmk_size = LMK_SIZE * ids.size();
    MatXX Hpp(COV_SIZE, COV_SIZE);
    MatXX Hpl(COV_SIZE, lmk_size);
    Eigen::Matrix<TYPE, Eigen::Dynamic, LMK_SIZE> Hll_diag(lmk_size, LMK_SIZE);
    Hpp.setZero();
    Hpl.setZero();
    Hll_diag.setZero();

    // Gradient
    VecX gp(COV_SIZE);
    VecX gl(lmk_size);
    gp.setZero();
    gl.setZero();
    std::vector<size_t> valid_observations_per_landmark(ids.size(), 0);
    // Local landmark increment -> world XYZ increment. For anchored modes this
    // is also used to back-transform Schur landmark corrections and covariance
    // updates, while Landmark::position itself remains world XYZ.
    std::vector<Mat3_3> landmark_parameter_to_world(
        ids.size(), Mat3_3::Identity());
    std::vector<Frame *> landmark_anchor_frames(ids.size(), nullptr);

    // 优化: 外参相关量对整帧是常量，提到所有循环外(原本每个观测都重算一次)
    const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
    size_t observations_used = 0;
    size_t observations_downweighted = 0;
    size_t observations_rejected = 0;
    size_t observations_new = 0;
    size_t observations_reused = 0;
    size_t rotation_only_constraints = 0;

    if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
        const RDVIOConstraintStatistics rdvio_statistics =
            accumulateRDVIOConstraints(
                map_, rotation_only_tracks, Ric,
                has_rdvio_zero_translation,
                enforce_observability_constraint_, visual_batch_variance,
                rdvio_zero_translation_std, visual_hard_reprojection_limit,
                Hpp, gp);
        observations_used += rdvio_statistics.observations_used;
        observations_new += rdvio_statistics.new_observations;
        n_new_observations_ += rdvio_statistics.new_observations;
        one_shot_tracks_used += rdvio_statistics.tracks_used;
        rotation_only_constraints += rdvio_statistics.rotation_constraints;
        n_rdvio_rotation_constraints_ +=
            rdvio_statistics.rotation_constraints;
        n_rdvio_zero_translation_constraints_ +=
            rdvio_statistics.zero_translation_constraints;
    }

    // 单个观测无法在消元 landmark 后约束位姿。先暂存每个点的第一条有效观测，
    // 只有第二条到来后才一起写入全局 Hessian；否则 Hpp 会错误地把该点当成固定地图点。
    struct LinearizedObservation {
        Mat2_6 J_pose;
        Mat2_6 J_anchor_pose{Mat2_6::Zero()};
        Mat2_3 J_landmark;
        Vec2 residual;
        TYPE weight{TYPE(1)};
        size_t frame_index{0};
        size_t anchor_frame_index{0};
        bool has_anchor_pose{false};
        Observation *observation{};
    };
    std::vector<LinearizedObservation> first_observation(ids.size());
    auto accumulate_observation = [&](const size_t landmark_index,
                                      const LinearizedObservation &linearized) {
        Hpp.block<6, 6>(linearized.frame_index, linearized.frame_index)
            .triangularView<Eigen::Upper>() +=
                linearized.weight * linearized.J_pose.transpose() * linearized.J_pose;
        if (linearized.has_anchor_pose) {
            Hpp.block<6, 6>(linearized.anchor_frame_index,
                            linearized.anchor_frame_index)
                .triangularView<Eigen::Upper>() +=
                    linearized.weight * linearized.J_anchor_pose.transpose() *
                    linearized.J_anchor_pose;
            if (linearized.frame_index < linearized.anchor_frame_index) {
                Hpp.block<6, 6>(linearized.frame_index,
                                linearized.anchor_frame_index).noalias() +=
                    linearized.weight * linearized.J_pose.transpose() *
                    linearized.J_anchor_pose;
            } else {
                Hpp.block<6, 6>(linearized.anchor_frame_index,
                                linearized.frame_index).noalias() +=
                    linearized.weight * linearized.J_anchor_pose.transpose() *
                    linearized.J_pose;
            }
        }
        Hll_diag.middleRows<3>(landmark_index).triangularView<Eigen::Upper>() +=
            linearized.weight * linearized.J_landmark.transpose() * linearized.J_landmark;
        Hpl.block<6, 3>(linearized.frame_index, landmark_index).noalias() +=
            linearized.weight * linearized.J_pose.transpose() * linearized.J_landmark;
        if (linearized.has_anchor_pose) {
            Hpl.block<6, 3>(linearized.anchor_frame_index, landmark_index).noalias() +=
                linearized.weight * linearized.J_anchor_pose.transpose() *
                linearized.J_landmark;
        }
        gp.segment<6>(linearized.frame_index).noalias() +=
            linearized.weight * linearized.J_pose.transpose() * linearized.residual;
        if (linearized.has_anchor_pose) {
            gp.segment<6>(linearized.anchor_frame_index).noalias() +=
                linearized.weight * linearized.J_anchor_pose.transpose() *
                linearized.residual;
        }
        gl.segment<3>(landmark_index).noalias() +=
            linearized.weight * linearized.J_landmark.transpose() * linearized.residual;
        ++observations_used;
        observations_downweighted += linearized.weight < TYPE(1) ? 1 : 0;
        if (linearized.observation) {
            if (linearized.observation->visual_update_count == 0) {
                ++observations_new;
                ++n_new_observations_;
            } else {
                ++observations_reused;
                ++n_reused_observations_;
            }
            ++linearized.observation->visual_update_count;
        }
    };

    // 遍历 landmarks
    for (size_t i = 0; i < ids.size(); ++i) {
        auto lmk = ids[i].second;
        const size_t lmk_index = LMK_SIZE * i;

        const LandmarkParameterizationLinearization parameterization =
            linearizeLandmarkParameterization(
                *lmk, Ric, ext_.t_ic, enforce_observability_constraint_,
                landmark_parameterization);
        if (!parameterization.valid) {
            continue;
        }
        landmark_parameter_to_world[i] = parameterization.parameter_to_world;
        landmark_anchor_frames[i] = parameterization.anchor;

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

            // A structureless MSCKF track is a one-shot measurement batch. If
            // a sample reaches the linearizer twice, reject it before H/g so a
            // lifecycle bug cannot silently make the filter overconfident.
            if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
                if (obs && obs->visual_update_count != 0) {
                    ++n_duplicate_observations_blocked_;
                    ++observations_rejected;
                    continue;
                }
            }

            const Mat3_3 Rwi = frm->q().toRotationMatrix();
            const Vec3 d_ij_w = lmk->position - frm->p();
            const Vec3 d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const Vec3 d_cj_c = Ric.transpose() * d_cj_i;
            if (!d_cj_c.allFinite() || d_cj_c.z() <= TYPE(0.05)) {
                ++observations_rejected;
                continue;
            }
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const Vec2 est = d_cj_c.head<2>() * inv_d;
            const Vec2 err = obs->un_pt.head<2>() - est;
            const TYPE residual_norm = err.norm();
            if (!std::isfinite(residual_norm) ||
                residual_norm > visual_hard_reprojection_limit) {
                ++observations_rejected;
                continue;
            }
            const TYPE huber_delta = std::max(
                visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
            const TYPE robust_weight = residual_norm > huber_delta
                ? huber_delta / residual_norm : TYPE(1);

            // Residuals are evaluated at the current estimate. With the
            // observability constraint enabled, Jacobians use the first
            // estimate of each clone (FEJ). The persistent landmark is
            // relinearized at its current estimate so long tracks do not keep a
            // stale depth; using one common landmark point for all observations
            // still preserves the same joint gauge nullspace.
            const Mat3_3 Rwi_jac = enforce_observability_constraint_
                ? frm->q_fej().toRotationMatrix()
                : Rwi;
            const Vec3 d_ij_w_jac = enforce_observability_constraint_
                ? lmk->position - frm->p_fej()
                : d_ij_w;
            const Vec3 d_cj_i_jac = Rwi_jac.transpose() * d_ij_w_jac - ext_.t_ic;
            const Vec3 d_cj_c_jac = Ric.transpose() * d_cj_i_jac;
            if (!d_cj_c_jac.allFinite() || d_cj_c_jac.z() <= TYPE(0.05)) {
                ++observations_rejected;
                continue;
            }
            const TYPE inv_d_jac = TYPE(1) / d_cj_c_jac.z();
            const TYPE inv_d2_jac = inv_d_jac * inv_d_jac;

            Mat2_3 J;
            J << inv_d_jac, TYPE(0), -d_cj_c_jac.x() * inv_d2_jac,
                    TYPE(0), inv_d_jac, -d_cj_c_jac.y() * inv_d2_jac;

            // Rwc^T = (Rwi * Ric)^T = Ric^T * Rwi^T，复用已算好的 Rwi/Ric,
            // 避免再做一次四元数乘法 + 求逆 + toRotationMatrix
            Mat2_3 J_lmk_world;
            J_lmk_world.noalias() = J * (Ric.transpose() * Rwi_jac.transpose());
            Mat2_3 J_lmk;
            J_lmk.noalias() = J_lmk_world * landmark_parameter_to_world[i];

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk_world * hat(d_ij_w_jac);
            J_pose.rightCols<3>().noalias() = -J_lmk_world;

            Mat2_6 J_anchor_pose = Mat2_6::Zero();
            size_t anchor_frame_index = 0;
            bool has_anchor_pose = false;
            if constexpr (isAnchoredLandmarkParameterization(
                              landmark_parameterization)) {
                const Frame *anchor = landmark_anchor_frames[i];
                J_anchor_pose = landmarkAnchorPoseJacobian(
                    J_lmk_world, *lmk, *anchor,
                    enforce_observability_constraint_);
                anchor_frame_index = INSState::SIZE +
                    AugState::SIZE * anchor->ordering;
                const size_t observing_frame_index = INSState::SIZE +
                    AugState::SIZE * frm->ordering;
                if (anchor_frame_index == observing_frame_index) {
                    // A feature represented in its own camera frame is
                    // invariant to a common motion of that camera and point.
                    J_pose += J_anchor_pose;
                } else {
                    has_anchor_pose = true;
                }
            }

            // 外参雅可比: 保留代码但默认不运行(外参目前不在状态里，算了也没人读)。
            // 用 if constexpr 而非 #ifdef，这样它始终参与语法/类型检查，不会腐烂。
            // 开启 ESTIMATE_EXTRINSIC 时还需把 J_ext 累加进 Hpp/Hpl/gp 的外参块。
            if constexpr (ExtState::ESTIMATE_EXTRINSIC) {
                Mat2_6 J_ext;
                J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
                J_ext.leftCols<3>().noalias() =
                    -J_ext.rightCols<3>() * hat(d_cj_i_jac);
            }

            const size_t frm_index = INSState::SIZE + AugState::SIZE * frm->ordering;
            LinearizedObservation current{
                J_pose, J_anchor_pose, J_lmk, err, robust_weight, frm_index,
                anchor_frame_index, has_anchor_pose, obs};
            auto &valid_count = valid_observations_per_landmark[i];
            if (valid_count == 0) {
                first_observation[i] = current;
                valid_count = 1;
                continue;
            }
            if (valid_count == 1) {
                accumulate_observation(lmk_index, first_observation[i]);
            }
            accumulate_observation(lmk_index, current);
            ++valid_count;
        }
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();
    // Hll_diag 的每个 3x3 块单独对称化
    for (size_t i = 0; i < ids.size(); ++i) {
        auto &&h = Hll_diag.middleRows<LMK_SIZE>(i * LMK_SIZE);
        h.triangularView<Eigen::StrictlyLower>() = h.triangularView<Eigen::StrictlyUpper>().transpose();
    }

    // Build the FEJ nullspace basis for diagnostics and the optional projection
    // below. Before Schur elimination a gauge vector contains both pose and
    // landmark components; projecting only the pose block would break the joint
    // normal equation. Any pose-only projection must therefore be delayed until
    // the landmark components have been eliminated.
    constexpr int OC_DIM = 4;
    Eigen::Matrix<TYPE, Eigen::Dynamic, OC_DIM> oc_basis(COV_SIZE, OC_DIM);
    oc_basis.setZero();
    TYPE oc_leak_before = TYPE(0);
    TYPE oc_leak_after = TYPE(0);
    if (map_.sfw.size() > 0) {
        Vec3 gravity_axis = state_.gravity;
        if (!gravity_axis.allFinite() || gravity_axis.norm() < TYPE(1e-8)) {
            gravity_axis = Vec3::UnitZ();
        } else {
            gravity_axis.normalize();
        }
        const Vec3 anchor_position = map_.sfw[0]->p_fej();
        for (size_t frame_number = 0; frame_number < map_.sfw.size(); ++frame_number) {
            const auto frame = map_.sfw[frame_number];
            const size_t offset = INSState::SIZE + AugState::SIZE * frame->ordering;
            oc_basis.block<3, 3>(offset + AugState::P, 0).setIdentity();
            oc_basis.block<3, 1>(offset + AugState::Q, 3) = gravity_axis;
            oc_basis.block<3, 1>(offset + AugState::P, 3) =
                -hat(frame->p_fej() - anchor_position) * gravity_axis;
        }
    }

    auto t_sc1 = clock();
    t_build_H_ += t_sc1 - t1;

    // 计算 schur 补
    //
    // 注: 试过利用 Hpl 的块稀疏性(只在被观测帧的块上运算)，实测反而变慢
    //   (2.60s -> 3.49s)。原因是每个 landmark 平均被 14.8 个关键帧观测到
    //   (WIN_SIZE = 30)，稀疏度只有 2 倍，而 K*K ~ 219 次 6x6 小块乘法的
    //   标量索引开销超过了省下的乘零。稠密 GEMM 的向量化更划算，保持原样。
    MatXX tmp(COV_SIZE, LMK_SIZE);
    for (size_t i = 0; i < ids.size(); ++i) {
        if (valid_observations_per_landmark[i] < 2) {
            continue;
        }
        auto index = i * LMK_SIZE;

        // STEP1: 对 Hll 求逆
        const Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
        // Hll inverse, Schur matrix and Schur gradient must share one retained
        // eigenspace. Mixing COD here with an unrelated threshold downstream
        // can leave gradient energy in a direction already removed from H.
        Eigen::SelfAdjointEigenSolver<Mat3_3> hll_es(hll);
        if (hll_es.info() != Eigen::Success ||
            !hll_es.eigenvalues().allFinite()) {
            continue;
        }
        const TYPE hll_max = hll_es.eigenvalues().maxCoeff();
        if (!(hll_max > TYPE(0))) {
            continue;
        }
        const TYPE hll_threshold = hll_rank_relative_threshold_ * hll_max;
        Vec3 hll_inverse = Vec3::Zero();
        const Vec3 hll_gradient_coeff =
            hll_es.eigenvectors().transpose() * gl.segment<LMK_SIZE>(index);
        TYPE discarded_gradient_sq = TYPE(0);
        size_t discarded_directions = 0;
        TYPE hll_min_retained = std::numeric_limits<TYPE>::infinity();
        for (size_t direction = 0; direction < LMK_SIZE; ++direction) {
            if (hll_es.eigenvalues()(direction) > hll_threshold) {
                hll_inverse(direction) = TYPE(1) / hll_es.eigenvalues()(direction);
                hll_min_retained = std::min(
                    hll_min_retained, hll_es.eigenvalues()(direction));
            } else {
                discarded_gradient_sq += hll_gradient_coeff(direction) *
                                         hll_gradient_coeff(direction);
                ++discarded_directions;
            }
        }
        const TYPE discarded_gradient_ratio = std::sqrt(discarded_gradient_sq) /
            std::max(hll_gradient_coeff.norm(), TYPE(1e-15));
        ++n_hll_rank_tests_;
        n_hll_discarded_directions_ += discarded_directions;
        hll_discarded_gradient_ratio_sum_ += discarded_gradient_ratio;
        hll_discarded_gradient_ratio_max_ =
            std::max(hll_discarded_gradient_ratio_max_, discarded_gradient_ratio);
        if (std::isfinite(hll_min_retained) && hll_min_retained > TYPE(0)) {
            const TYPE effective_condition = hll_max / hll_min_retained;
            ++n_hll_condition_tests_;
            hll_effective_condition_sum_ += effective_condition;
            hll_effective_condition_max_ =
                std::max(hll_effective_condition_max_, effective_condition);
        }
        const Mat3_3 hll_inv = hll_es.eigenvectors() * hll_inverse.asDiagonal()
                             * hll_es.eigenvectors().transpose();

        // STEP2: 计算 Hpl * Hll^-1
        tmp.noalias() = Hpl.middleCols<LMK_SIZE>(index) * hll_inv;

        // STEP3: 计算 Hpp - Hpl * Hll^-1 * Hpl^T
        Hpp.triangularView<Eigen::Upper>() -= tmp * Hpl.middleCols<LMK_SIZE>(index).transpose();

        // STEP4: 计算 gp - Hpl * Hll^-1 * gl
        gp.noalias() -= tmp * gl.segment<LMK_SIZE>(index);
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();

    // If hard projection succeeds, these are the only information directions
    // consumed by the state update. No second rank decision is allowed later.
    MatXX projected_state_basis;
    VecX projected_state_diagonal;
    VecX projected_state_rhs;
    bool has_consistent_projection = false;

    // Diagnose FEJ-nullspace leakage on the Schur-reduced pose system. The
    // optional minimum projection Π = I - N(N^T N)^-1N^T enforces Hpp*N_fej=0,
    // but remains disabled by default because projecting gp was experimentally
    // harmful near the numerically truncated Schur nullspace (see docs).
    if (map_.sfw.size() > 0) {
        const TYPE hpp_norm = std::max(Hpp.norm(), TYPE(1e-15));
        const TYPE basis_norm = std::max(oc_basis.norm(), TYPE(1e-15));
        oc_leak_before = (Hpp * oc_basis).norm() / (hpp_norm * basis_norm);

        if (enforce_observability_constraint_ && project_observability_constraint_) {
            // Prior whitening makes the projection dimensionless before mixing
            // attitude (rad) and position (m) components.
            MatXX prior_covariance = TYPE(0.5) * (cov_ + cov_.transpose());
            Eigen::LLT<MatXX> prior_llt(prior_covariance);
            if (prior_llt.info() != Eigen::Success) {
                const TYPE jitter = TYPE(1e-12) * std::max(
                    prior_covariance.diagonal().cwiseAbs().maxCoeff(), TYPE(1));
                prior_covariance.diagonal().array() += jitter;
                prior_llt.compute(prior_covariance);
            }
            if (prior_llt.info() == Eigen::Success) {
                const MatXX prior_sqrt = prior_llt.matrixL();
                const MatXX nullspace_whitened =
                    prior_sqrt.triangularView<Eigen::Lower>().solve(oc_basis);
                Eigen::ColPivHouseholderQR<MatXX> nullspace_qr(nullspace_whitened);
                nullspace_qr.setThreshold(TYPE(1e-10));
                if (nullspace_qr.rank() == OC_DIM) {
                    const MatXX q_null = nullspace_qr.householderQ() *
                        MatXX::Identity(COV_SIZE, OC_DIM);
                    MatXX hpp_whitened =
                        prior_sqrt.transpose() * Hpp * prior_sqrt;
                    VecX gp_whitened = prior_sqrt.transpose() * gp;

                    // Pi*H*Pi and Pi*g, Pi=I-Qn*Qn^T. No dense projector.
                    hpp_whitened.noalias() -=
                        q_null * (q_null.transpose() * hpp_whitened);
                    hpp_whitened.noalias() -=
                        (hpp_whitened * q_null) * q_null.transpose();
                    hpp_whitened = TYPE(0.5) *
                        (hpp_whitened + hpp_whitened.transpose());
                    gp_whitened.noalias() -=
                        q_null * (q_null.transpose() * gp_whitened);

                    Eigen::SelfAdjointEigenSolver<MatXX> hpp_es(hpp_whitened);
                    if (hpp_es.info() == Eigen::Success &&
                        hpp_es.eigenvalues().allFinite()) {
                        const TYPE hpp_max = hpp_es.eigenvalues().maxCoeff();
                        const TYPE hpp_threshold =
                            hpp_rank_relative_threshold_ * hpp_max;
                        const VecX gradient_coeff =
                            hpp_es.eigenvectors().transpose() * gp_whitened;
                        std::vector<Eigen::Index> retained;
                        retained.reserve(COV_SIZE);
                        TYPE discarded_gradient_sq = TYPE(0);
                        for (Eigen::Index direction = 0;
                             direction < static_cast<Eigen::Index>(COV_SIZE);
                             ++direction) {
                            if (hpp_es.eigenvalues()(direction) > hpp_threshold) {
                                retained.push_back(direction);
                            } else {
                                discarded_gradient_sq += gradient_coeff(direction) *
                                                         gradient_coeff(direction);
                            }
                        }

                        const TYPE discarded_gradient_ratio =
                            std::sqrt(discarded_gradient_sq) /
                            std::max(gradient_coeff.norm(), TYPE(1e-15));
                        ++n_hpp_rank_tests_;
                        n_hpp_discarded_directions_ += COV_SIZE - retained.size();
                        hpp_discarded_gradient_ratio_sum_ += discarded_gradient_ratio;
                        hpp_discarded_gradient_ratio_max_ = std::max(
                            hpp_discarded_gradient_ratio_max_, discarded_gradient_ratio);

                        if (hpp_max > TYPE(0) && !retained.empty()) {
                            MatXX retained_vectors(COV_SIZE, retained.size());
                            projected_state_diagonal.resize(retained.size());
                            projected_state_rhs.resize(retained.size());
                            for (size_t column = 0; column < retained.size(); ++column) {
                                const Eigen::Index direction = retained[column];
                                retained_vectors.col(column) =
                                    hpp_es.eigenvectors().col(direction);
                                projected_state_diagonal(column) =
                                    hpp_es.eigenvalues()(direction);
                                projected_state_rhs(column) = gradient_coeff(direction);
                            }
                            // v_i^T*z = (L^-T*v_i)^T*dx.
                            projected_state_basis = prior_sqrt.transpose()
                                .triangularView<Eigen::Upper>()
                                .solve(retained_vectors);
                            has_consistent_projection = true;
                            ++n_oc_projections_;

                            const TYPE whitened_hpp_norm =
                                std::max(hpp_whitened.norm(), TYPE(1e-15));
                            const TYPE whitened_basis_norm =
                                std::max(nullspace_whitened.norm(), TYPE(1e-15));
                            oc_leak_after =
                                (hpp_whitened * nullspace_whitened).norm() /
                                (whitened_hpp_norm * whitened_basis_norm);
                        }
                    }
                }
            }
        }

        if (!has_consistent_projection) {
            const TYPE projected_hpp_norm = std::max(Hpp.norm(), TYPE(1e-15));
            oc_leak_after = (Hpp * oc_basis).norm() /
                            (projected_hpp_norm * basis_norm);
        }
        oc_max_leak_before_ = std::max(oc_max_leak_before_, oc_leak_before);
        oc_max_leak_after_ = std::max(oc_max_leak_after_, oc_leak_after);
    }

    auto t_sc2 = clock();
    t_schur_ += t_sc2 - t_sc1;

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
    TYPE nis_sum = TYPE(0);
    size_t nis_count = 0;
    {
        auto &&cov_p = cov_;
        auto &&ep = gp;

        // 序贯更新需要各标量量测互不相关，即把 Cov[e] = σ²·H 对角化。
        // 特征分解和 LDLT 都能做到，区别只在用哪组基:
        //
        //   特征分解 H = V·λ·V^T:  Cov[V^T·e] = σ²·V^T·H·V = σ²·λ      (对角)
        //   LDLT     H = L·D·L^T:  Cov[L^-1·e] = σ²·L^-1·H·L^-T = σ²·D  (对角)
        //
        // 两者都给出 COV_SIZE 个独立标量量测:
        //   特征分解: h_i = V.col(i),  z_i = (V^T·gp)_i / λ_i,  R_i = σ²/λ_i
        //   LDLT:     h_i = M.col(i),  z_i = (M^-1·gp)_i / D_i, R_i = σ²/D_i
        //     其中 M = P^T·L (Eigen 的 LDLT 带主元置换: A = P^T·L·D·L^T·P)
        //
        // 注意两者并非逐位等价 —— 用的是不同的基，中间量不同，
        // 但都是同一个信息矩阵的合法分解，最终后验应当一致(数值误差内)。
        VecX H_BASIS_diag(COV_SIZE);   // λ 或 D
        MatXX H_BASIS(COV_SIZE, COV_SIZE);  // V 或 M
        VecX rhs(COV_SIZE);            // V^T·gp 或 M^-1·gp

        auto t_e0 = clock();
        if (has_consistent_projection) {
            H_BASIS.setZero();
            H_BASIS_diag.setZero();
            rhs.setZero();
            const Eigen::Index retained_count = projected_state_diagonal.size();
            H_BASIS.leftCols(retained_count) = projected_state_basis;
            H_BASIS_diag.head(retained_count) = projected_state_diagonal;
            rhs.head(retained_count) = projected_state_rhs;
        } else if constexpr (USE_LDLT_FOR_HPP) {
            Eigen::LDLT<MatXX> ldlt(Hpp);

            // M = P^T · L，使得 Hpp = M · D · M^T
            H_BASIS = ldlt.transpositionsP().transpose()
                    * MatXX(ldlt.matrixL());
            H_BASIS_diag = ldlt.vectorD();

            // rhs = M^-1 · gp，用三角回代而不是显式求逆:
            //   M·rhs = gp  =>  P^T·L·rhs = gp  =>  L·rhs = P·gp
            rhs = ldlt.transpositionsP() * ep;
            ldlt.matrixL().solveInPlace(rhs);
        } else {
            Eigen::SelfAdjointEigenSolver<MatXX> es(Hpp);
            H_BASIS = es.eigenvectors();
            H_BASIS_diag = es.eigenvalues();
            rhs.noalias() = H_BASIS.transpose() * ep;
        }
        t_eig_decomp_ += clock() - t_e0;

        // Step-0: 过滤掉(近似)为 0 的对角元。
        //   特征分解: eigenvalues 已升序排列，找到第一个足够大的即可
        //   LDLT:     D 无序，必须逐个判断，所以下面用 skip 而不是起始下标
        const TYPE d_max = H_BASIS_diag.maxCoeff();
        const TYPE d_thresh = has_consistent_projection
            ? TYPE(0)
            : hpp_rank_relative_threshold_ * d_max;
        if (d_max <= TYPE(0)) {
            std::cerr << "Hpp is not positive: max diag = " << d_max << std::endl;
        }

        // Step-1: 序贯
        //
        // 优化: cov_p 全程保持对称，所以
        //   1) cov_p * hT 用 selfadjointView 做对称矩阵-向量乘 (只读一半)
        //   2) 两次 rank-1/rank-2 更新只写上三角，循环内不再重建下三角
        //      (原本每次迭代都做一次 198x198 的 StrictlyLower = StrictlyUpper^T)
        //   3) 下三角在循环结束后统一恢复一次
        // 数学上完全等价: 中间过程只有 selfadjointView 在读 cov_p，它只看上三角。
        VecX PhT(COV_SIZE);
        VecX K(COV_SIZE);
        const Eigen::Index basis_direction_count = has_consistent_projection
            ? projected_state_diagonal.size()
            : static_cast<Eigen::Index>(COV_SIZE);
        for (Eigen::Index i = 0; i < basis_direction_count; ++i) {
            const auto d = H_BASIS_diag(i);
            if (d <= d_thresh) {
                ++n_skipped_;           // 诊断: 被判定为零空间的方向数
                if (d < TYPE(0)) {
                    ++n_negative_;      // 诊断: 严格为负(Hpp 不定)的方向数
                }
                continue;   // 零空间方向，不提供信息
            }

            const auto R = visual_batch_variance / d;
            const auto hT = H_BASIS.col(i);

            PhT.noalias() = cov_p.selfadjointView<Eigen::Upper>() * hT;
            const TYPE var = hT.dot(PhT) + R;
            // 量测 z_i = rhs(i)/d，残差 = z_i - h_i^T·dx。
            // 序贯更新已经把伪量测噪声对角化，因此 e^2/var 可以直接累加为 NIS。
            const TYPE e = rhs(i) / d - hT.dot(dx_p);
            if (enable_logging_ && var > TYPE(0) && std::isfinite(var) && std::isfinite(e)) {
                nis_sum += e * e / var;
                ++nis_count;
            }
            K.noalias() = PhT / var;
            cov_p.triangularView<Eigen::Upper>() -= K * PhT.transpose();

            PhT.noalias() = cov_p.selfadjointView<Eigen::Upper>() * hT;
            cov_p.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();

            dx_p.noalias() += K * e;
        }
        cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();
    }
    updateState(dx_p);
//    std::cout << "Update State Finished" << std::endl;

    // [数据采集] 记录视觉更新【后】的后验状态与修正量
    if (enable_logging_) {
        using I = INSState;
        log.p_post = state_.position;
        log.v_post = state_.velocity;
        log.q_post = state_.orientation;
        log.bg_post = state_.gyro_bias;
        log.ba_post = state_.accel_bias;
        log.g_post = state_.gravity;
        log.dx_q_norm = dx_p.segment<3>(I::Q).norm();
        log.dx_p_norm = dx_p.segment<3>(I::P).norm();
        log.dx_v_norm = dx_p.segment<3>(I::V).norm();
        log.cov_q_trace = cov_.diagonal().segment<3>(I::Q).sum();
        log.cov_p_trace = cov_.diagonal().segment<3>(I::P).sum();
        log.cov_v_trace = cov_.diagonal().segment<3>(I::V).sum();
        log.nis_mean = nis_count ? nis_sum / static_cast<TYPE>(nis_count) : TYPE(0);
        log.nis_dof = nis_count;
        log.n_obs_used = observations_used;
        log.n_obs_downweighted = observations_downweighted;
        log.n_obs_rejected = observations_rejected;
        log.n_obs_new = observations_new;
        log.n_obs_reused = observations_reused;
        log.n_tracks_consumed = schedulerConsumesTracksOnce()
            ? tracks_to_consume.size() : 0;
        log.oc_leak_before = oc_leak_before;
        log.oc_leak_after = oc_leak_after;
        log.rotation_only_constraints = rotation_only_constraints;
        logs_.emplace_back(log);
    }

    auto t_sc3 = clock();
    t_eig_state_ += t_sc3 - t_sc2;

//    std::cout << "Update Landmark" << std::endl;
    // [[ 更新 Landmark ]]
    //
    // The old IndependentEkf path is retained only as an experiment/reference:
    // it repeatedly treats the same window observations as new measurements of
    // an independent landmark even though P_xl is not stored. Fixed,
    // Retriangulate, and SchurBackSubstitution avoid that false independence.
    const LandmarkUpdateMode landmark_mode = schedulerConsumesTracksOnce()
        ? LandmarkUpdateMode::Fixed
        : (refine_landmarks_ ? landmark_update_mode_ : LandmarkUpdateMode::Fixed);
    const bool independent_landmark_mode =
        landmark_mode == LandmarkUpdateMode::IndependentEkf ||
        landmark_mode == LandmarkUpdateMode::IndependentEkfInflated ||
        landmark_mode == LandmarkUpdateMode::IndependentEkfAdaptive;
    if (independent_landmark_mode ||
        (landmark_mode == LandmarkUpdateMode::SchurBackSubstitution && is_keyframe)) {
        gl.noalias() -= Hpl.transpose() * dx_p;
    }

    const Mat3_3 Ric_landmark = ext_.q_ic.toRotationMatrix();
    auto landmarkReprojectionCost = [&](const Landmark &landmark,
                                        const Vec3 &position) -> TYPE {
        const TYPE huber_delta = std::max(
            visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
        TYPE cost = TYPE(0);
        size_t count = 0;
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            (void)frame_id;
            if (!feature || !feature->frame || !feature->obs[0]) {
                continue;
            }
            const auto *frame = feature->frame;
            const Mat3_3 Rwi = frame->q().toRotationMatrix();
            const Vec3 d_camera = Ric_landmark.transpose() *
                (Rwi.transpose() * (position - frame->p()) - ext_.t_ic);
            if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
                return std::numeric_limits<TYPE>::infinity();
            }
            const Vec2 estimate = d_camera.head<2>() / d_camera.z();
            const TYPE residual_norm =
                (feature->obs[0]->un_pt.head<2>() - estimate).norm();
            if (!std::isfinite(residual_norm)) {
                return std::numeric_limits<TYPE>::infinity();
            }
            cost += residual_norm <= huber_delta
                ? TYPE(0.5) * residual_norm * residual_norm
                : huber_delta * (residual_norm - TYPE(0.5) * huber_delta);
            ++count;
        }
        return count >= 2 ? cost / static_cast<TYPE>(count)
                          : std::numeric_limits<TYPE>::infinity();
    };

    // Schur 路径的量测只来自持久关键帧。若本次没有为该点加入新的关键帧观测，
    // 再次修正只是在重复求解几乎相同的批次，会放大相关量测的重复使用并浪费计算。
    auto observedInCurrentKeyframe = [&](const Landmark &landmark) {
        if (!is_keyframe) {
            return false;
        }
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            (void)frame_id;
            if (feature && feature->frame &&
                feature->frame->timestamp == cam_data.timestamp) {
                return true;
            }
        }
        return false;
    };

    if (landmark_mode == LandmarkUpdateMode::Retriangulate && is_keyframe) {
        for (const auto &[id, lmk] : ids) {
            (void)id;
            if (!observedInCurrentKeyframe(*lmk)) {
                continue;
            }
            ++n_lmk_update_attempts_;
            const TYPE cost_before = landmarkReprojectionCost(*lmk, lmk->position);
            const TriangulationResult triangulation = triangulateLandmark(*lmk);
            if (triangulation.status != TriangulationStatus::Success) {
                continue;
            }
            const TYPE cost_after =
                landmarkReprojectionCost(*lmk, triangulation.position);
            if (!std::isfinite(cost_after) || cost_after > cost_before + TYPE(1e-15)) {
                continue;
            }
            lmk->position = triangulation.position;
            lmk->cov_position = triangulation.covariance;
            ++n_lmk_update_accepted_;
            ++n_lmk_retriangulation_success_;
            lmk_reprojection_cost_reduction_ += cost_before - cost_after;
            recordLandmarkRefinement(*lmk);
        }
    } else if (landmark_mode == LandmarkUpdateMode::SchurBackSubstitution && is_keyframe) {
        for (size_t i = 0; i < ids.size(); ++i) {
            if (valid_observations_per_landmark[i] < 2) {
                continue;
            }
            auto lmk = ids[i].second;
            if (!observedInCurrentKeyframe(*lmk)) {
                continue;
            }
            const size_t index = i * LMK_SIZE;
            const Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
            const Vec3 el = gl.segment<LMK_SIZE>(index);
            ++n_lmk_update_attempts_;

            Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);
            if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
                continue;
            }
            const TYPE hll_max = es.eigenvalues().maxCoeff();
            if (!(hll_max > TYPE(0))) {
                continue;
            }
            const TYPE threshold = TYPE(1e-6) * hll_max;
            const Vec3 inverse = (es.eigenvalues().array() > threshold)
                .select(es.eigenvalues().array().inverse(), TYPE(0));
            const Vec3 parameter_increment = es.eigenvectors() * inverse.asDiagonal()
                                           * es.eigenvectors().transpose() * el;
            const Vec3 increment = landmark_parameter_to_world[i] * parameter_increment;
            if (!increment.allFinite() || increment.norm() > TYPE(100)) {
                continue;
            }

            const TYPE cost_before = landmarkReprojectionCost(*lmk, lmk->position);
            TYPE step = TYPE(1);
            bool accepted = false;
            for (size_t line_search = 0; line_search < 5; ++line_search) {
                const Vec3 candidate = lmk->position + step * increment;
                const TYPE cost_after = landmarkReprojectionCost(*lmk, candidate);
                if (std::isfinite(cost_after) && cost_after <= cost_before + TYPE(1e-15)) {
                    lmk->position = candidate;
                    ++n_lmk_update_accepted_;
                    lmk_reprojection_cost_reduction_ += cost_before - cost_after;
                    recordLandmarkRefinement(*lmk);
                    accepted = true;
                    break;
                }
                step *= TYPE(0.5);
            }
            (void)accepted;
        }
    } else if (independent_landmark_mode) {
        for (size_t i = 0; i < ids.size(); ++i) {
            if (valid_observations_per_landmark[i] < 2) {
                continue;
            }
            auto id = ids[i].first;
            auto lmk = ids[i].second;
            const size_t index = i * LMK_SIZE;
            Vec3 dx_l = Vec3::Zero();
            auto &&cov_p = lmk->cov_position;
            Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
            Vec3 el = gl.segment<LMK_SIZE>(index);
            ++n_lmk_update_attempts_;

            // The persistent covariance is stored in world XYZ. Convert the
            // local anchored normal equation back to that basis before the
            // independent-map experiment consumes it.
            if constexpr (isAnchoredLandmarkParameterization(
                              landmark_parameterization)) {
                if (!transformLandmarkNormalToWorld(
                        landmark_parameter_to_world[i], hll, el)) {
                    continue;
                }
            }

            if (landmark_mode != LandmarkUpdateMode::IndependentEkf) {
                TYPE inflation_scale = TYPE(1);
                if (landmark_mode == LandmarkUpdateMode::IndependentEkfAdaptive) {
                    const TYPE excess_nis = std::max(
                        TYPE(0), lmk->independent_nis_ema - TYPE(1));
                    inflation_scale = std::clamp(
                        landmark_adaptive_inflation_gain_ * excess_nis,
                        TYPE(0), landmark_adaptive_inflation_max_scale_);
                }
                cov_p.diagonal().array() +=
                    landmark_process_noise_density_ * std::max(TYPE(dt), TYPE(0)) *
                    inflation_scale;
            }

            Vec3 hll_diag;
            Mat3_3 hll_basis;
            Vec3 hll_rhs;
            if constexpr (USE_LDLT_FOR_HLL) {
                Eigen::LDLT<Mat3_3> ldlt(hll);
                hll_basis = ldlt.transpositionsP().transpose() * Mat3_3(ldlt.matrixL());
                hll_diag = ldlt.vectorD();
                hll_rhs = ldlt.transpositionsP() * el;
                ldlt.matrixL().solveInPlace(hll_rhs);
            } else {
                Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);
                hll_basis = es.eigenvectors();
                hll_diag = es.eigenvalues();
                hll_rhs.noalias() = hll_basis.transpose() * el;
            }

            const TYPE hll_max = hll_diag.maxCoeff();
            const TYPE hll_thresh = hll_rank_relative_threshold_ * hll_max;
            if (hll_max <= TYPE(0)) {
                std::cerr << "Hll not positive: id = " << id
                          << ", diag = " << hll_diag.transpose() << std::endl;
                continue;
            }
            TYPE landmark_nis_sum = TYPE(0);
            size_t landmark_nis_count = 0;
            for (size_t j = 0; j < LMK_SIZE; ++j) {
                const TYPE d = hll_diag(j);
                if (d <= hll_thresh) {
                    continue;
                }
                const TYPE R = visual_batch_variance / d;
                const Vec3 hT = hll_basis.col(j);
                Vec3 PhT = cov_p * hT;
                const TYPE var = hT.dot(PhT) + R;
                const TYPE e = hll_rhs(j) / d - hT.dot(dx_l);
                if (var > TYPE(0) && std::isfinite(var) && std::isfinite(e)) {
                    landmark_nis_sum += e * e / var;
                    ++landmark_nis_count;
                }
                const Vec3 K = PhT / var;
                cov_p -= K * PhT.transpose();
                PhT = cov_p * hT;
                cov_p.triangularView<Eigen::Upper>() +=
                    (K * R - PhT) * K.transpose();
                cov_p.triangularView<Eigen::StrictlyLower>() =
                    cov_p.triangularView<Eigen::StrictlyUpper>().transpose();
                dx_l += K * e;
            }
            if (landmark_nis_count > 0) {
                const TYPE batch_nis = landmark_nis_sum /
                    static_cast<TYPE>(landmark_nis_count);
                const TYPE alpha = std::clamp(
                    landmark_nis_ema_alpha_, TYPE(0), TYPE(1));
                lmk->independent_nis_ema =
                    (TYPE(1) - alpha) * lmk->independent_nis_ema +
                    alpha * batch_nis;
            }
            lmk->position += dx_l;
            ++n_lmk_update_accepted_;
            recordLandmarkRefinement(*lmk);
        }
    }

    // Detached map post-processor. It consumes only the newest keyframe
    // measurement and never writes Landmark::position/cov_position, so its
    // deliberately approximate independent covariance cannot alter ESKF state.
    if (enable_shadow_landmark_postprocessor_ && is_keyframe &&
        !schedulerConsumesTracksOnce()) {
        const TYPE image_variance = triangulation_uv_std * triangulation_uv_std;
        const TYPE huber_delta = std::max(
            visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
        for (const auto &[id, lmk] : ids) {
            (void)id;
            Feature *newest_feature = nullptr;
            for (const auto &[frame_id, feature] : lmk->frm2fet) {
                (void)frame_id;
                if (feature && feature->frame && feature->obs[0] &&
                    feature->frame->timestamp == cam_data.timestamp) {
                    newest_feature = feature;
                    break;
                }
            }
            if (!newest_feature) {
                continue;
            }
            if (!lmk->shadow_initialized) {
                lmk->shadow_position = lmk->position;
                lmk->shadow_cov_position = lmk->cov_position;
                lmk->shadow_nis_ema = TYPE(1);
                lmk->shadow_initialized = true;
            }

            TYPE inflation_scale = shadow_landmark_adaptive_inflation_
                ? TYPE(0)
                : TYPE(1);
            if (shadow_landmark_adaptive_inflation_) {
                const TYPE excess_nis =
                    std::max(TYPE(0), lmk->shadow_nis_ema - TYPE(1));
                inflation_scale = std::clamp(
                    landmark_adaptive_inflation_gain_ * excess_nis,
                    TYPE(0), landmark_adaptive_inflation_max_scale_);
            }
            lmk->shadow_cov_position.diagonal().array() +=
                landmark_process_noise_density_ * std::max(TYPE(dt), TYPE(0)) *
                inflation_scale;

            const auto *frame = newest_feature->frame;
            const Mat3_3 Rwi = frame->q().toRotationMatrix();
            const Mat3_3 Rwc = Rwi * Ric_landmark;
            const Vec3 camera_center = frame->p() + Rwi * ext_.t_ic;
            const Vec3 d_camera =
                Rwc.transpose() * (lmk->shadow_position - camera_center);
            if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
                continue;
            }
            const TYPE inv_depth = TYPE(1) / d_camera.z();
            const TYPE inv_depth2 = inv_depth * inv_depth;
            const Vec2 estimate = d_camera.head<2>() * inv_depth;
            const Vec2 residual =
                newest_feature->obs[0]->un_pt.head<2>() - estimate;
            if (!residual.allFinite() ||
                residual.norm() > visual_hard_reprojection_limit) {
                continue;
            }

            Mat2_3 projection_jacobian;
            projection_jacobian <<
                inv_depth, TYPE(0), -d_camera.x() * inv_depth2,
                TYPE(0), inv_depth, -d_camera.y() * inv_depth2;
            const Mat2_3 landmark_jacobian =
                projection_jacobian * Rwc.transpose();
            const TYPE robust_weight = residual.norm() > huber_delta
                ? huber_delta / residual.norm()
                : TYPE(1);
            Mat2_2 measurement_covariance = Mat2_2::Identity() *
                (image_variance / std::max(robust_weight, TYPE(1e-6)));
            // Pose uncertainty is part of a detached landmark measurement.
            // Omitting it made the shadow map appear precise while its error
            // was actually dominated by clone-pose uncertainty.
            Mat2_6 pose_jacobian;
            pose_jacobian.leftCols<3>().noalias() =
                landmark_jacobian * hat(lmk->shadow_position - frame->p());
            pose_jacobian.rightCols<3>().noalias() = -landmark_jacobian;
            const size_t frame_offset =
                INSState::SIZE + AugState::SIZE * frame->ordering;
            const Eigen::Matrix<TYPE, 6, 6> frame_covariance =
                cov_.block<6, 6>(frame_offset, frame_offset);
            measurement_covariance.noalias() +=
                pose_jacobian * frame_covariance * pose_jacobian.transpose();
            measurement_covariance = TYPE(0.5) *
                (measurement_covariance + measurement_covariance.transpose());
            Mat2_2 innovation_covariance =
                landmark_jacobian * lmk->shadow_cov_position *
                landmark_jacobian.transpose() + measurement_covariance;
            innovation_covariance = TYPE(0.5) *
                (innovation_covariance + innovation_covariance.transpose());
            Eigen::LDLT<Mat2_2> innovation_ldlt(innovation_covariance);
            if (innovation_ldlt.info() != Eigen::Success ||
                innovation_ldlt.vectorD().minCoeff() <= TYPE(0)) {
                continue;
            }

            TYPE normalized_innovation = std::max(
                TYPE(0), residual.dot(innovation_ldlt.solve(residual)) / TYPE(2));
            if (shadow_landmark_adaptive_inflation_ &&
                normalized_innovation > TYPE(1)) {
                // Innovation-adaptive fading acts immediately on the current
                // inconsistent observation, unlike an EMA-only scheme that
                // can react only at the next keyframe.
                const TYPE fading_factor = std::clamp(
                    TYPE(1) + landmark_adaptive_inflation_gain_ *
                        (normalized_innovation - TYPE(1)),
                    TYPE(1), landmark_adaptive_inflation_max_scale_);
                lmk->shadow_cov_position *= fading_factor;
                innovation_covariance =
                    landmark_jacobian * lmk->shadow_cov_position *
                    landmark_jacobian.transpose() + measurement_covariance;
                innovation_covariance = TYPE(0.5) *
                    (innovation_covariance + innovation_covariance.transpose());
                innovation_ldlt.compute(innovation_covariance);
                if (innovation_ldlt.info() != Eigen::Success ||
                    innovation_ldlt.vectorD().minCoeff() <= TYPE(0)) {
                    continue;
                }
                normalized_innovation = std::max(
                    TYPE(0), residual.dot(innovation_ldlt.solve(residual)) / TYPE(2));
            }
            const TYPE alpha = std::clamp(
                landmark_nis_ema_alpha_, TYPE(0), TYPE(1));
            lmk->shadow_nis_ema =
                (TYPE(1) - alpha) * lmk->shadow_nis_ema +
                alpha * normalized_innovation;

            const Mat3_2 gain = lmk->shadow_cov_position *
                landmark_jacobian.transpose() *
                innovation_ldlt.solve(Mat2_2::Identity());
            lmk->shadow_position.noalias() += gain * residual;
            const Mat3_3 identity_minus_kh =
                Mat3_3::Identity() - gain * landmark_jacobian;
            lmk->shadow_cov_position =
                identity_minus_kh * lmk->shadow_cov_position *
                    identity_minus_kh.transpose() +
                gain * measurement_covariance * gain.transpose();
            lmk->shadow_cov_position = TYPE(0.5) *
                (lmk->shadow_cov_position + lmk->shadow_cov_position.transpose());
            recordLandmarkRefinement(*lmk, true);
        }
    }
//    std::cout << "Update Landmark Finished" << std::endl;

    auto t_sc4 = clock();
    t_eig_lmk_ += t_sc4 - t_sc3;
    n_lmk_total_ += ids.size();
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

        const auto R = visual_batch_variance;
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
        const Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);

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

        const auto R = visual_batch_variance;
        MatXX PHT = cov_p * hll;

        MatXX S = PHT;
        S.diagonal().array() += R;
        MatXX KT = S.inverse() * cov_p;

        cov_p -= PHT * KT;
        cov_p = 0.5 * (cov_p + cov_p.transpose());

        VecX dx_l = KT.transpose() * el;

        lmk->position += dx_l;
        recordLandmarkRefinement(*lmk);
//        std::cout << "id = " << id << ", dx_l = " << dx_l.transpose() << std::endl;
    }
#endif

    // TODO: 更新完后需要固定最老帧率，除非用的是FEJ或OC

    auto t2 = clock();
    t_cost_ += t2 - t1;
    ++posterior_times_;

#else

#endif

    // Track consumption and scheduler-specific clone removal are handled by
    // schedule_finalizer so every early-return path has identical lifecycle.
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
        Frame *frame = map_.sfw[n];
        const size_t offset = I::SIZE + frame->ordering * A::SIZE;
        frame->q() = (vec2quat(Eigen::Map<Vec3>(dx.data() + offset + A::Q)) *
                      frame->q()).normalized();
        frame->p() += Eigen::Map<Vec3>(dx.data() + offset + A::P);
    }
}

void SchurVINS::setQPV(const Quat &q, const Vec3 &p, const Vec3 &v) {
    state_.orientation = q;
    state_.position = p;
    state_.velocity = v;

    Rnb_ = state_.orientation.toRotationMatrix();
}
