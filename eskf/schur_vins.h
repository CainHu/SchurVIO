//
// Created by 许家仁 on 2025/8/26.
//

#ifndef VINSEKF_SCHUR_VINS_H
#define VINSEKF_SCHUR_VINS_H

#include "../common.h"
//#include "../data_structure/map.h"

namespace slam {
    constexpr static auto operator""_s(unsigned long long s) {
        return static_cast<Tus>(s * 1000000);
    }

    constexpr static auto operator""_ms(unsigned long long ms) {
        return static_cast<Tus>(ms * 1000);
    }

    constexpr static auto operator""_us(unsigned long long us) {
        return static_cast<Tus>(us);
    }

    constexpr static auto operator""_s(long double s) {
        return static_cast<Tus>(s * 1e6);
    }

    constexpr static auto operator""_ms(long double ms) {
        return static_cast<Tus>(ms * 1e3);
    }

    constexpr static auto operator""_us(long double us) {
        return static_cast<Tus>(us);
    }

    constexpr static bool CONFIG_DEBUG = true;
    constexpr static Tus IMU_TS = 5000;
    constexpr static Tus CAM_TS = 50000;

    class SchurVINS {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        explicit SchurVINS();

        // 处理IMU数据(预测步骤)
        void processIMU(const IMUData &imu_data);

        void processFrame(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map);

    protected:
        // 状态预测
        void predict(const IMUData& imu_data, double dt);

        //
        void pushFrame(const CameraData &cam_data);
        const auto & popFrame();

        // 更新 map
        void updateMap(const CameraData &cam_data);

        // 视觉更新
        void updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, double dt);

        void updateState(auto &&dx);

//    private:
    public:
        INSState state_;

        Vec3 gyro_corr_;
        Vec3 accel_corr_;
        Vec3 accel_corr_world_;
        Mat3_3 Rnb_;
        Vec3 a_world_;

        IMUData imu_data_last_;
        CameraData cam_data_last_;

        Tus imu_ts_{IMU_TS};
        Tus cam_ts_{CAM_TS};

        constexpr static size_t WIN_SIZE = 7;
        constexpr static size_t COV_SIZE = INSState::SIZE + WIN_SIZE * AugState::SIZE;
        size_t latest_free_sfw_idx_{0};
        std::vector<size_t> free_sfw_idx_;
        std::vector<std::pair<AugState, CameraData>> sfw_;
        std::unordered_map<size_t, LmkState> lmk_;
        Eigen::MatrixXd cov_;

        ExtState ext_;

        constexpr static size_t LMK_SIZE = 3;

        constexpr static TYPE uv_var = TYPE(1.);
        constexpr static TYPE lmk_var = TYPE(0.01);

        Eigen::VectorXd Rll_;
    };
}

#endif //VINSEKF_SCHUR_VINS_H
