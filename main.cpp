#include "vio_frontend_simulator.h"
#include "eskf/schur_vins.h"
#include <iostream>

#if 0
// 假设的EKF后端类（用于演示）
class VIOBackendEKF {
public:
    void init(const State& initial_state) {
        std::cout << "初始化EKF后端..." << std::endl;
        // 实际实现中应该初始化状态和协方差矩阵
    }

    void processImuData(const ImuData& imu_data) {
        // 实际实现中应该进行IMU预测更新
    }

    void processCameraData(const CameraData& camera_data, const std::vector<Eigen::Vector3d>& feature_positions) {
        // 实际实现中应该进行视觉测量更新
    }

    State getCurrentState() const {
        // 返回当前估计状态
        return State();
    }
};
#endif

int main() {
    // 创建模拟器
    VIOFrontendSimulator simulator;

    // 配置模拟器参数
    simulator.setImuNoise(0.01, 0.01, 0.001, 0.001);  // IMU噪声参数

    // 配置轨迹：半径5m，速度1m/s，持续20s
    simulator.setTrajectoryParams(5.0, 1.0, 20.0);

    // 配置环形特征点：3个同心圆环，半径分别为8m、10m、12m，共200个特征点
    std::vector<double> ring_radii = {8.0, 10.0, 12.0};
    simulator.setCircularFeaturesParams(200, ring_radii, Eigen::Vector3d(0, 0, 1.5));

    // 生成模拟数据
    std::vector<ImuData> imu_data;
    std::vector<CameraData> camera_data;
    std::vector<State> ground_truth;
    simulator.generateData(imu_data, camera_data, ground_truth);

    // 获取特征点真实位置
    const auto& feature_positions = simulator.getFeaturePositions();

//    std::cout << "生成数据完成: " << std::endl;
//    std::cout << "IMU数据数量: " << imu_data.size() << std::endl;
//    std::cout << "相机数据数量: " << camera_data.size() << std::endl;
//    std::cout << "特征点数量: " << simulator.getFeaturePositions().size() << std::endl;


    std::cout << "feature_positions.size() = " << feature_positions.size() << std::endl;

    std::cout << "q:\n" << ground_truth[0].q << std::endl;
    std::cout << "p:\n" << ground_truth[0].p << std::endl;
    std::cout << "v:\n" << ground_truth[0].v << std::endl;

    // 初始化EKF后端
//    slam::Map map;
    static slam::SchurVINS ekf;

    // 处理数据
    size_t cam_idx = 0;
    size_t imu_idx = 0;
    size_t gt_idx = 0;
    for (const auto& cam_data : camera_data) {
        // 处理两个相机帧之间的所有IMU数据
//        std::cout << "1 = " << imu_data[imu_idx].timestamp << ", 2 = " << cam_data.timestamp << std::endl;
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp < cam_data.timestamp) {
            ekf.processIMU(imu_data[imu_idx]);
            imu_idx++;
        }
//        std::cout << "1 = " << imu_data[imu_idx].timestamp << ", 2 = " << cam_data.timestamp << std::endl;

        while (gt_idx < ground_truth.size() && static_cast<slam::Tus>(ground_truth[gt_idx].timestamp * 1e6) < cam_data.timestamp) {
            gt_idx++;
        }

        std::cout << "POS: GT = " << ground_truth[gt_idx].p.transpose() << ", EST = " << ekf.state_.position.transpose() << std::endl;
        std::cout << "QUAT: GT = " << ground_truth[gt_idx].q << ", EST = " << ekf.state_.orientation << std::endl;

        for (auto &it : cam_data.measurements) {
            const auto id = it.first;
            if (auto i = feature_positions.find(id); i != feature_positions.end()) {
                Eigen::Vector3d uv_gt = ground_truth[gt_idx].q.inverse() * (i->second - ground_truth[gt_idx].p);
                uv_gt /= uv_gt.z();

                Eigen::Vector3d uv_ekf = ekf.state_.orientation.inverse() * (i->second - ekf.state_.position);
                uv_ekf /= uv_ekf.z();

//                std::cout << "uv: gt = " << uv_gt(0) << ", " << uv_gt(1);
//                std::cout << ", ekf = " << uv_ekf(0) << ", " << uv_ekf(1);
//                std::cout << ", meas = " << it.second(0) << ", " << it.second(1) << std::endl;
            }
        }

        // 处理相机数据
        ekf.processFrame(cam_data, feature_positions);

//        std::cout << "free_sfw_idx_ = " << ekf.latest_free_sfw_idx_ << std::endl;

        // 这里可以添加代码，将EKF的估计结果与ground_truth进行比较
        // State estimated_state = ekf.getCurrentState();
        // 计算误差并输出...

        ++cam_idx;
    }

    // 处理剩余的IMU数据
    while (imu_idx < imu_data.size()) {
        ekf.processIMU(imu_data[imu_idx]);
        imu_idx++;
    }

    std::cout << "数据处理完成" << std::endl;
    return 0;
}