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

    std::cout << "生成数据完成: " << std::endl;
    std::cout << "IMU数据数量: " << imu_data.size() << std::endl;
    std::cout << "相机数据数量: " << camera_data.size() << std::endl;
    std::cout << "特征点数量: " << simulator.getFeaturePositions().size() << std::endl;

    // 获取特征点真实位置
    const auto& feature_positions = simulator.getFeaturePositions();

    // 初始化EKF后端
    slam::Map map;
    slam::SchurVINS ekf(map);

    // 处理数据
    size_t imu_idx = 0;
    for (const auto& cam_data : camera_data) {
        // 处理两个相机帧之间的所有IMU数据
        while (imu_idx < imu_data.size() && imu_data[imu_idx].timestamp <= cam_data.timestamp) {
            ekf.processIMU(imu_data[imu_idx]);
            imu_idx++;
        }

        // 处理相机数据
        ekf.processFrame(cam_data, feature_positions);

        std::cout << "free_sfw_idx_ = " << ekf.latest_free_sfw_idx_ << std::endl;

        // 这里可以添加代码，将EKF的估计结果与ground_truth进行比较
        // State estimated_state = ekf.getCurrentState();
        // 计算误差并输出...
    }

    // 处理剩余的IMU数据
    while (imu_idx < imu_data.size()) {
        ekf.processIMU(imu_data[imu_idx]);
        imu_idx++;
    }

    std::cout << "数据处理完成" << std::endl;
    return 0;
}