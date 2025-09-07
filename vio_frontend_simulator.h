//
// Created by 许家仁 on 2025/8/24.
//

#pragma once

#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include "common.h"

// 状态结构体：位置、姿态、速度、IMU偏置
struct State {
    double timestamp;
    Eigen::Vector3d p;  // 位置
    Eigen::Quaterniond q;  // 姿态（w,x,y,z）
    Eigen::Vector3d v;  // 速度
    Eigen::Vector3d ba;  // 加速度计偏置
    Eigen::Vector3d bg;  // 陀螺仪偏置
};

using ImuData = slam::IMUData;
using CameraData = slam::CameraData;

// VIO前端模拟器类
class VIOFrontendSimulator {
private:
    // 模拟器参数
    double imu_rate_ = 200.0;  // IMU采样频率(Hz)
    double camera_rate_ = 20.0;  // 相机采样频率(Hz)
    double imu_acc_noise_std_ = 0.01;  // 加速度计噪声标准差(m/s²)
    double imu_gyro_noise_std_ = 0.01;  // 陀螺仪噪声标准差(rad/s)
    double imu_acc_bias_noise_std_ = 0.001;  // 加速度计偏置噪声标准差(m/s²)
    double imu_gyro_bias_noise_std_ = 0.001;  // 陀螺仪偏置噪声标准差(rad/s)
    double camera_fx_ = 230.0;  // 相机内参
    double camera_fy_ = 230.0;
    double camera_cx_ = 320.0*4;
    double camera_cy_ = 240.0*4;
    double camera_noise_std_ = 1.0;  // 相机测量噪声标准差(像素)
    double trajectory_radius_ = 5.0;  // 轨迹半径(m)
    double trajectory_speed_ = 1.0;  // 运动速度(m/s)
    double trajectory_duration_ = 40.0;  // 轨迹持续时间(s)

    // 特征点参数（环形分布）
    size_t num_features_ = 10000;  // 特征点总数
    std::vector<double> ring_radii_ = {6.0, 8.0, 10.0, 12.0};  // 多个同心圆环的半径(m)
    Eigen::Vector3d ring_center_ = Eigen::Vector3d(0, 0, 0.);  // 圆环中心位置(默认在地面上方1.5m)
    double ring_min_theta_ = 0;  // 角度范围（弧度）
    double ring_max_theta_ = 2 * M_PI;
    double ring_min_phi_ = -M_PI/6;  // 极角范围（略微向上和向下，避免完全在同一平面）
    double ring_max_phi_ = M_PI/6;

    std::unordered_map<size_t, Eigen::Vector3d> feature_positions_; // 特征点3D位置

    static std::mt19937 random_generator_;  // 随机数生成器

    // 生成真实轨迹
    [[nodiscard]] std::vector<State> generateGroundTruth() const;

    // 生成IMU测量数据
    [[nodiscard]] std::vector<ImuData> generateImuData(const std::vector<State>& ground_truth) const;

    // 生成相机测量数据
    [[nodiscard]] std::vector<CameraData> generateCameraData(const std::vector<State>& ground_truth) const;

    // 生成圆环形分布的特征点
    void generateCircularFeatures();

public:
    VIOFrontendSimulator() {
        // 生成特征点
        generateCircularFeatures();
    }

    // 设置IMU噪声参数
    void setImuNoise(double acc_noise_std, double gyro_noise_std,
                     double acc_bias_noise_std, double gyro_bias_noise_std) {
        imu_acc_noise_std_ = acc_noise_std;
        imu_gyro_noise_std_ = gyro_noise_std;
        imu_acc_bias_noise_std_ = acc_bias_noise_std;
        imu_gyro_bias_noise_std_ = gyro_bias_noise_std;
    }

    // 设置轨迹参数
    void setTrajectoryParams(double radius, double speed, double duration) {
        trajectory_radius_ = radius;
        trajectory_speed_ = speed;
        trajectory_duration_ = duration;
    }

    // 设置环形特征点参数
    void setCircularFeaturesParams(size_t num_features,
                                   const std::vector<double>& radii,
                                   const Eigen::Vector3d& center = Eigen::Vector3d(0, 0, 1.5)) {
        num_features_ = num_features;
        ring_radii_ = radii;
        ring_center_ = center;
        generateCircularFeatures();  // 重新生成特征点
    }

    // 生成所有模拟数据
    void generateData(std::vector<ImuData>& imu_data,
                      std::vector<CameraData>& camera_data,
                      std::vector<State>& ground_truth) const {
        ground_truth = generateGroundTruth();
        imu_data = generateImuData(ground_truth);
        camera_data = generateCameraData(ground_truth);
    }

    // 获取特征点真实位置
    const auto& getFeaturePositions() const {
        return feature_positions_;
    }
};
