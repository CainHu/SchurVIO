#pragma once

#include "vio_frontend_simulator.h"

#include <Eigen/Dense>
#include <random>
#include <unordered_map>
#include <vector>

// 补充两类更有代表性的轨迹：三维螺旋（充分激励）和往返启停（低激励/静止段）。
// 坐标约定与现有模拟器一致：q 将相机/IMU 坐标旋转到世界坐标，光轴为相机 +Z，
// 世界重力为 (0, 0, +9.81)。
class VIORepresentativeSimulator {
public:
    enum class Trajectory {
        Helix3D,
        StopGo
    };

    explicit VIORepresentativeSimulator(Trajectory trajectory);

    void setImuNoise(double acc_noise_density,
                     double gyro_noise_density,
                     double acc_bias_random_walk,
                     double gyro_bias_random_walk);
    void setDuration(double duration_seconds) { duration_ = duration_seconds; }
    void setFeatureCount(size_t count);

    void generateData(std::vector<ImuData> &imu_data,
                      std::vector<CameraData> &camera_data,
                      std::vector<State> &ground_truth) const;

    [[nodiscard]] const auto &getFeaturePositions() const { return feature_positions_; }
    [[nodiscard]] double getCameraFocalLength() const { return camera_fx_; }
    [[nodiscard]] double getCameraNoiseStd() const { return camera_noise_std_; }

private:
    [[nodiscard]] std::vector<State> generateGroundTruth() const;
    [[nodiscard]] std::vector<ImuData> generateImuData(const std::vector<State> &ground_truth) const;
    [[nodiscard]] std::vector<CameraData> generateCameraData(const std::vector<State> &ground_truth) const;
    [[nodiscard]] Eigen::Quaterniond lookAt(const Eigen::Vector3d &position,
                                            const Eigen::Vector3d &target,
                                            double roll) const;
    void generateFeatures();

    Trajectory trajectory_;
    double imu_rate_{200.0};
    double camera_rate_{20.0};
    double duration_{60.0};
    double imu_acc_noise_density_{0.02};
    double imu_gyro_noise_density_{0.002};
    double imu_acc_bias_random_walk_{0.0005};
    double imu_gyro_bias_random_walk_{0.0001};
    double fov_{2.0 * M_PI / 3.0};
    double camera_cx_{320.0};
    double camera_cy_{320.0};
    double camera_fx_{camera_cx_ / std::tan(0.5 * fov_)};
    double camera_fy_{camera_cy_ / std::tan(0.5 * fov_)};
    double camera_noise_std_{1.0};
    size_t feature_count_{1000};
    std::unordered_map<size_t, Eigen::Vector3d> feature_positions_;

};
