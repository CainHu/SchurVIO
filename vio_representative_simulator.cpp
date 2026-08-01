#include "vio_representative_simulator.h"

#include <algorithm>
#include <cmath>

VIORepresentativeSimulator::VIORepresentativeSimulator(const Trajectory trajectory)
    : trajectory_(trajectory) {
    generateFeatures();
}

void VIORepresentativeSimulator::setImuNoise(const double acc_noise_density,
                                              const double gyro_noise_density,
                                              const double acc_bias_random_walk,
                                              const double gyro_bias_random_walk) {
    imu_acc_noise_density_ = acc_noise_density;
    imu_gyro_noise_density_ = gyro_noise_density;
    imu_acc_bias_random_walk_ = acc_bias_random_walk;
    imu_gyro_bias_random_walk_ = gyro_bias_random_walk;
}

void VIORepresentativeSimulator::setFeatureCount(const size_t count) {
    feature_count_ = count;
    generateFeatures();
}

void VIORepresentativeSimulator::generateFeatures() {
    feature_positions_.clear();
    feature_positions_.reserve(feature_count_ * 2);
    std::mt19937 feature_generator(0x5A11u);

    if (trajectory_ == Trajectory::Helix3D) {
        std::uniform_real_distribution<double> theta(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> radius(1.5, 5.0);
        std::uniform_real_distribution<double> z(-2.5, 2.5);
        for (size_t id = 0; id < feature_count_; ++id) {
            const double a = theta(feature_generator);
            const double r = radius(feature_generator);
            feature_positions_.emplace(
                id, Eigen::Vector3d(r * std::cos(a), r * std::sin(a), 1.5 + z(feature_generator)));
        }
    } else {
        std::uniform_real_distribution<double> x(-8.0, 8.0);
        std::uniform_real_distribution<double> y(-1.0, 12.0);
        std::uniform_real_distribution<double> z(-1.5, 4.5);
        for (size_t id = 0; id < feature_count_; ++id) {
            feature_positions_.emplace(id, Eigen::Vector3d(
                x(feature_generator), y(feature_generator), z(feature_generator)));
        }
    }
}

Eigen::Quaterniond VIORepresentativeSimulator::lookAt(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &target,
        const double roll) const {
    Eigen::Vector3d z_axis = (target - position).normalized();
    Eigen::Vector3d down_reference(0.0, 0.0, 1.0);
    if (std::abs(z_axis.dot(down_reference)) > 0.95) {
        down_reference = Eigen::Vector3d(0.0, 1.0, 0.0);
    }
    Eigen::Vector3d x_axis = down_reference.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d Rwc;
    Rwc.col(0) = x_axis;
    Rwc.col(1) = y_axis;
    Rwc.col(2) = z_axis;
    Rwc *= Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return Eigen::Quaterniond(Rwc).normalized();
}

std::vector<State> VIORepresentativeSimulator::generateGroundTruth() const {
    const double dt = 1.0 / imu_rate_;
    const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6);
    const size_t steps = static_cast<size_t>(duration_ / dt) + 1;
    std::vector<State> ground_truth;
    ground_truth.reserve(steps);
    std::mt19937 bias_generator(0x5A12u);

    State state{};
    state.timestamp = 0;
    state.ba.setZero();
    state.bg.setZero();
    std::normal_distribution<double> ba_walk(0.0, imu_acc_bias_random_walk_ * std::sqrt(dt));
    std::normal_distribution<double> bg_walk(0.0, imu_gyro_bias_random_walk_ * std::sqrt(dt));

    for (size_t i = 0; i < steps; ++i) {
        state.timestamp += dt_us;
        const double t = static_cast<double>(state.timestamp) * 1e-6;

        if (trajectory_ == Trajectory::Helix3D) {
            constexpr double radius = 6.0;
            constexpr double omega = 0.20;
            const double angle = omega * t;
            state.p = Eigen::Vector3d(radius * std::cos(angle),
                                      radius * std::sin(angle),
                                      1.5 + std::sin(0.5 * angle));
            state.v = Eigen::Vector3d(-radius * omega * std::sin(angle),
                                       radius * omega * std::cos(angle),
                                       0.5 * omega * std::cos(0.5 * angle));
            const Eigen::Vector3d target(0.0, 0.0, 1.5 + 0.2 * std::sin(0.25 * angle));
            state.q = lookAt(state.p, target, 0.15 * std::sin(0.7 * angle));
        } else {
            constexpr double period = 20.0;
            constexpr double move_time = 6.0;
            const double phase = std::fmod(t, period);
            double x = -5.0;
            double vx = 0.0;
            if (phase < move_time) {
                const double u = phase / move_time;
                x = -5.0 + 5.0 * (1.0 - std::cos(M_PI * u));
                vx = 5.0 * M_PI / move_time * std::sin(M_PI * u);
            } else if (phase < 10.0) {
                x = 5.0;
            } else if (phase < 10.0 + move_time) {
                const double u = (phase - 10.0) / move_time;
                x = 5.0 - 5.0 * (1.0 - std::cos(M_PI * u));
                vx = -5.0 * M_PI / move_time * std::sin(M_PI * u);
            }
            state.p = Eigen::Vector3d(x, -6.0, 1.5);
            state.v = Eigen::Vector3d(vx, 0.0, 0.0);
            state.q = lookAt(state.p, Eigen::Vector3d(0.0, 3.0, 1.5),
                             0.08 * std::sin(2.0 * M_PI * t / period));
        }

        state.ba += Eigen::Vector3d(ba_walk(bias_generator),
                                    ba_walk(bias_generator),
                                    ba_walk(bias_generator));
        state.bg += Eigen::Vector3d(bg_walk(bias_generator),
                                    bg_walk(bias_generator),
                                    bg_walk(bias_generator));
        ground_truth.emplace_back(state);
    }
    return ground_truth;
}

std::vector<ImuData> VIORepresentativeSimulator::generateImuData(
        const std::vector<State> &ground_truth) const {
    std::vector<ImuData> imu_data;
    imu_data.reserve(ground_truth.size());
    const Eigen::Vector3d gravity(0.0, 0.0, 9.81);
    std::mt19937 imu_generator(0x5A13u);
    const double white_noise_scale = use_legacy_white_noise_discretization_
                                     ? 1.0 / std::sqrt(imu_rate_)
                                     : std::sqrt(imu_rate_);
    std::normal_distribution<double> acc_noise(0.0, imu_acc_noise_density_ * white_noise_scale);
    std::normal_distribution<double> gyro_noise(0.0, imu_gyro_noise_density_ * white_noise_scale);

    for (size_t i = 1; i < ground_truth.size(); ++i) {
        const State &previous = ground_truth[i - 1];
        const State &current = ground_truth[i];
        const double dt = static_cast<double>(current.timestamp - previous.timestamp) * 1e-6;
        ImuData data;
        data.timestamp = current.timestamp;
        const Eigen::Vector3d acceleration_world = (current.v - previous.v) / dt;
        data.accel = current.q.inverse() * (acceleration_world - gravity) + current.ba;
        data.accel += Eigen::Vector3d(acc_noise(imu_generator),
                                      acc_noise(imu_generator),
                                      acc_noise(imu_generator));

        const Eigen::Quaterniond dq = previous.q.inverse() * current.q;
        data.gyro = slam::quat2vec(dq) / dt + current.bg;
        data.gyro += Eigen::Vector3d(gyro_noise(imu_generator),
                                     gyro_noise(imu_generator),
                                     gyro_noise(imu_generator));
        imu_data.emplace_back(data);
    }
    return imu_data;
}

std::vector<CameraData> VIORepresentativeSimulator::generateCameraData(
        const std::vector<State> &ground_truth) const {
    std::vector<CameraData> camera_data;
    std::mt19937 camera_generator(0x5A14u);
    const uint64_t camera_dt_us = static_cast<uint64_t>(1e6 / camera_rate_);
    uint64_t next_camera_time = 0;
    std::normal_distribution<double> image_noise(0.0, camera_noise_std_);

    for (const auto &state : ground_truth) {
        if (state.timestamp < next_camera_time) {
            continue;
        }
        CameraData data;
        data.timestamp = state.timestamp;
        for (const auto &[id, point_world] : feature_positions_) {
            const Eigen::Vector3d point_camera = state.q.inverse() * (point_world - state.p);
            if (point_camera.z() <= 0.05) {
                continue;
            }
            Eigen::Vector2d pixel(camera_fx_ * point_camera.x() / point_camera.z() + camera_cx_,
                                  camera_fy_ * point_camera.y() / point_camera.z() + camera_cy_);
            if (pixel.x() < 0.0 || pixel.x() >= 2.0 * camera_cx_ ||
                pixel.y() < 0.0 || pixel.y() >= 2.0 * camera_cy_) {
                continue;
            }
            pixel.x() += image_noise(camera_generator);
            pixel.y() += image_noise(camera_generator);
            data.measurements.emplace(id, Eigen::Vector2d(
                (pixel.x() - camera_cx_) / camera_fx_,
                (pixel.y() - camera_cy_) / camera_fy_));
        }
        camera_data.emplace_back(std::move(data));
        next_camera_time += camera_dt_us;
    }
    return camera_data;
}

void VIORepresentativeSimulator::generateData(std::vector<ImuData> &imu_data,
                                               std::vector<CameraData> &camera_data,
                                               std::vector<State> &ground_truth) const {
    ground_truth = generateGroundTruth();
    imu_data = generateImuData(ground_truth);
    camera_data = generateCameraData(ground_truth);
}
