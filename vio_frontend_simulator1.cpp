//
// Created by Cain on 2025/9/8.
//

#include "vio_frontend_simulator1.h"

static std::random_device rd;
std::mt19937 VIOFrontendSimulator1::random_generator_(0123);

// 生成圆环形分布的特征点
void VIOFrontendSimulator1::generateCircularFeatures() {
    size_t id = 0;
    feature_positions_.clear();
    feature_positions_.reserve(num_features_ << 1);

    // 如果没有指定圆环半径，默认添加一个
    if (ring_radii_.empty()) {
        ring_radii_.push_back(0.5 * trajectory_radius_);
    }

    // 计算每个圆环应分配的特征点数量
    size_t features_per_ring = num_features_ / ring_radii_.size();
    size_t remaining = num_features_ % ring_radii_.size();

    // 为每个圆环生成特征点
    for (size_t r = 0; r < ring_radii_.size(); ++r) {
        size_t count = features_per_ring + (r < remaining ? 1 : 0);
        double radius = ring_radii_[r];

        // 在圆环上均匀分布特征点
        for (size_t i = 0; i < count; ++i) {
            // 随机角度（也可以均匀分布）
            std::uniform_real_distribution<double> dist_theta(ring_min_theta_, ring_max_theta_);
            double theta = dist_theta(random_generator_);  // 方位角（绕z轴）

            // 球坐标转笛卡尔坐标
            double x = radius * cos(theta);
            double y = radius * sin(theta);

            std::uniform_real_distribution<double> dist_z(radius * tan(ring_min_phi_), radius * tan(ring_max_phi_));
            double z = dist_z(random_generator_);

            // 平移到圆环中心
            Eigen::Vector3d pos = ring_center_ + Eigen::Vector3d(x, y, z);
            feature_positions_.emplace(id++, pos);

//            std::cout << "pos = " << pos.transpose() << std::endl;
        }
    }
}

// 生成真实轨迹（圆周运动）
std::vector<State> VIOFrontendSimulator1::generateGroundTruth() const {
    std::vector<State> ground_truth;

    double total_time = trajectory_duration_;
    double dt = 1.0 / imu_rate_;  // 按IMU频率生成真实状态
    auto dt_us = static_cast<uint64_t>(dt * 1e6);
    size_t num_steps = static_cast<size_t>(total_time / dt) + 1;

    // 初始状态
    State current;
    current.timestamp = 0;
    current.ba = Eigen::Vector3d::Zero();  // 初始加速度计偏置
    current.bg = Eigen::Vector3d::Zero();  // 初始陀螺仪偏置

    // 生成后续状态
    for (size_t i = 0; i < num_steps; ++i) {
        current.timestamp += dt_us;

        // 计算当前时刻的角度（圆周运动）
        double angle = (trajectory_speed_ / trajectory_radius_) * (static_cast<double>(current.timestamp) * 1e-6);

        // 更新位置（圆周运动）
        current.p.x() = trajectory_radius_ * cos(angle);
        current.p.y() = trajectory_radius_ * sin(angle);
        current.p.z() = 1.0;  // 保持1m高度

        // 更新速度（切线方向）
        current.v.x() = -trajectory_speed_ * sin(angle);
        current.v.y() = trajectory_speed_ * cos(angle);
        current.v.z() = 0.0;

        // 更新姿态（始终面向运动方向）
        Eigen::Vector3d forward_dir(-sin(angle), cos(angle), 0.0);  // 前进方向
        Eigen::Vector3d down_dir(0.0, 0.0, 1.0);  // 向上方向
        Eigen::Vector3d right_dir = down_dir.cross(forward_dir).normalized();

        // 构建旋转矩阵
        Eigen::Matrix3d R;
        R.col(0) = down_dir;
        R.col(1) = forward_dir;
        R.col(2) = right_dir;
        current.q = Eigen::Quaterniond(R);

        // 模拟IMU偏置缓慢变化（随机游走）
        std::normal_distribution<double> ba_noise(0, imu_acc_bias_noise_std_ * sqrt(dt));
        std::normal_distribution<double> bg_noise(0, imu_gyro_bias_noise_std_ * sqrt(dt));
        current.ba.x() += ba_noise(random_generator_);
        current.ba.y() += ba_noise(random_generator_);
        current.ba.z() += ba_noise(random_generator_);
        current.bg.x() += bg_noise(random_generator_);
        current.bg.y() += bg_noise(random_generator_);
        current.bg.z() += bg_noise(random_generator_);

        ground_truth.push_back(current);
    }

    return ground_truth;
}

// 生成IMU测量数据
std::vector<ImuData> VIOFrontendSimulator1::generateImuData(const std::vector<State>& ground_truth) const {
    std::vector<ImuData> imu_data;

    // 重力加速度
    Eigen::Vector3d g(0, 0, 9.81);

    for (size_t i = 1; i < ground_truth.size(); ++i) {
        const State& prev = ground_truth[i - 1];
        const State& curr = ground_truth[i];
        uint64_t dt_us = curr.timestamp - prev.timestamp;
        auto dt = static_cast<double>(dt_us) * 1e-6;

        ImuData data;
        data.timestamp = curr.timestamp;

        // 计算理想加速度（机体坐标系）
        Eigen::Vector3d acc_ideal = curr.q.inverse() * ((curr.v - prev.v) / dt - g);
        // 添加偏置和噪声
        std::normal_distribution<double> acc_noise(0, imu_acc_noise_std_);
        data.accel = acc_ideal + curr.ba + Eigen::Vector3d(acc_noise(random_generator_),
                                                           acc_noise(random_generator_),
                                                           acc_noise(random_generator_));

        // 计算理想角速度（机体坐标系）
        Eigen::Quaterniond dq = prev.q.inverse() * curr.q;
//        Eigen::Vector3d gyro_ideal = 2.0 * dq.vec() / dt;
//        if (dq.w() < 0) gyro_ideal = -gyro_ideal;  // 确保最短路径
        Eigen::Vector3d gyro_ideal = slam::quat2vec(dq) / dt;
        // 添加偏置和噪声
        std::normal_distribution<double> gyro_noise(0, imu_gyro_noise_std_);
        data.gyro = gyro_ideal + curr.bg + Eigen::Vector3d(gyro_noise(random_generator_),
                                                           gyro_noise(random_generator_),
                                                           gyro_noise(random_generator_));

        imu_data.push_back(data);
    }

    return imu_data;
}

// 生成相机测量数据
std::vector<CameraData> VIOFrontendSimulator1::generateCameraData(const std::vector<State>& ground_truth) const {
    std::vector<CameraData> camera_data;

    double camera_dt = 1.0 / camera_rate_;
    auto camera_dt_us = static_cast<uint64_t>(camera_dt * 1e6);
    uint64_t next_camera_time = 0;

    // 相机内参矩阵
    Eigen::Matrix3d K;
    K << camera_fx_, 0, camera_cx_,
            0, camera_fy_, camera_cy_,
            0, 0, 1;

    for (const auto& state : ground_truth) {
        if (state.timestamp >= next_camera_time) {
            CameraData data;
            data.timestamp = state.timestamp;

            // 对每个特征点，检查是否在视野内
            for (const auto &it : feature_positions_) {
                const auto id = it.first;
                const auto &P_w = it.second;

                // 转换到相机坐标系
                Eigen::Vector3d P_c = state.q.inverse() * (P_w - state.p);

                // 检查是否在相机前方
                if (P_c.z() <= 0) continue;

                // 投影到图像平面
                Eigen::Vector2d uv = (K * (P_c / P_c.z())).head<2>();

                // 检查是否在图像范围内 (假设图像大小640x480)
                if (uv.x() < 0 || uv.x() >= 2. * camera_cx_ || uv.y() < 0 || uv.y() >= 2. * camera_cy_) {
//                    std::cout << "uv: " << uv.transpose() << std::endl;
                    continue;
                }

                // 添加噪声
                std::normal_distribution<double> noise(0, camera_noise_std_);
                uv.x() += noise(random_generator_);
                uv.y() += noise(random_generator_);

//                // 添加到观测数据
//                data.measurements.emplace(id, uv);

                // 添加归一化平面的数据
                Eigen::Vector2d un_pt {
                        (uv.x() - camera_cx_) / camera_fx_,
                        (uv.y() - camera_cy_) / camera_fy_
                };
                data.measurements.emplace(id, un_pt);
            }

            camera_data.push_back(data);
            next_camera_time += camera_dt_us;
        }
    }

    return camera_data;
}