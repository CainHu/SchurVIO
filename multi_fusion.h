#ifndef MULTI_SENSOR_EKF_H
#define MULTI_SENSOR_EKF_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>

// 四元数工具类
class QuaternionUtils {
public:
    // 四元数乘法 q1 * q2
    static Eigen::Quaterniond multiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        return q1 * q2;
    }

    // 四元数逆
    static Eigen::Quaterniond inverse(const Eigen::Quaterniond& q) {
        return q.inverse();
    }

    // 旋转向量转四元数
    static Eigen::Quaterniond rotVecToQuat(const Eigen::Vector3d& rot_vec) {
        double theta = rot_vec.norm();
        if (theta < 1e-8) {
            return Eigen::Quaterniond::Identity();
        }
        Eigen::Vector3d axis = rot_vec.normalized();
        return Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
    }

    // 四元数转旋转向量
    static Eigen::Vector3d quatToRotVec(const Eigen::Quaterniond& q) {
        Eigen::AngleAxisd aa(q);
        return aa.angle() * aa.axis();
    }

    // 四元数转旋转矩阵
    static Eigen::Matrix3d quatToRotMat(const Eigen::Quaterniond& q) {
        return q.toRotationMatrix();
    }

    // 用四元数旋转向量
    static Eigen::Vector3d rotateVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
        return q * v;
    }
};

// 传感器数据结构
struct IMUData {
    double timestamp;
    Eigen::Vector3d accel;    // 加速度 (m/s^2)
    Eigen::Vector3d gyro;     // 角速度 (rad/s)
};

struct VIOData {
    int id;                   // VIO系统ID
    double timestamp;
    Eigen::Vector3d position; // VIO坐标系下的位置 (m)
    Eigen::Vector3d velocity; // VIO坐标系下的速度 (m/s)
    Eigen::Quaterniond orientation; // VIO坐标系下的姿态
    Eigen::Matrix<double, 9, 9> covariance; // 位置、速度、姿态的协方差
};

struct GPSData {
    double timestamp;
    Eigen::Vector3d position; // GPS坐标系下的位置 (m)
    Eigen::Vector3d velocity; // GPS坐标系下的速度 (m/s)
    Eigen::Matrix<double, 6, 6> covariance; // 位置、速度的协方差
};

// EKF状态
struct EKFState {
    double timestamp;

    // 系统状态
    Eigen::Vector3d position;    // 全局坐标系下的位置 (m)
    Eigen::Vector3d velocity;    // 全局坐标系下的速度 (m/s)
    Eigen::Quaterniond orientation; // 全局坐标系到机体坐标系的姿态

    // IMU零偏
    Eigen::Vector3d accel_bias;  // 加速度计零偏
    Eigen::Vector3d gyro_bias;   // 陀螺仪零偏

    // VIO外参 (VIO坐标系到全局坐标系)
    std::vector<Eigen::Quaterniond> vio_rotations;  // 旋转外参
    std::vector<Eigen::Vector3d> vio_translations;  // 平移外参

    // GPS外参 (GPS坐标系到全局坐标系)
    Eigen::Quaterniond gps_rotation;  // 旋转外参
    Eigen::Vector3d gps_translation;  // 平移外参
};

// 多传感器融合EKF类
template<int NumVIOs>
class MultiSensorEKF {
public:
    MultiSensorEKF() {
        initialize();
    }

    // 初始化EKF
    void initialize() {
        // 初始化状态
        state_.timestamp = 0.0;
        state_.position.setZero();
        state_.velocity.setZero();
        state_.orientation.setIdentity();
        state_.accel_bias.setZero();
        state_.gyro_bias.setZero();

        // 初始化VIO外参
        state_.vio_rotations.resize(NumVIOs, Eigen::Quaterniond::Identity());
        state_.vio_translations.resize(NumVIOs, Eigen::Vector3d::Zero());

        // 初始化GPS外参
        state_.gps_rotation.setIdentity();
        state_.gps_translation.setZero();

        // 初始化协方差矩阵
        P_.setZero(stateSize(), stateSize());

        // 状态方差初始化
        double pos_var = 0.1;      // 位置方差
        double vel_var = 0.1;      // 速度方差
        double att_var = 0.01;     // 姿态方差 (rad^2)
        double ab_var = 0.001;     // 加速度计零偏方差
        double gb_var = 0.0001;    // 陀螺仪零偏方差
        double vio_rot_var = 0.01; // VIO旋转外参方差
        double vio_trans_var = 0.1;// VIO平移外参方差
        double gps_rot_var = 0.01; // GPS旋转外参方差
        double gps_trans_var = 0.1;// GPS平移外参方差

        // 系统状态方差
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * pos_var;          // 位置
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * vel_var;          // 速度
        P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * att_var;          // 姿态(旋转向量表示)

        // IMU零偏方差
        P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * ab_var;           // 加速度计零偏
        P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * gb_var;         // 陀螺仪零偏

        // VIO外参方差
        int idx = 15;
        for (int i = 0; i < NumVIOs; ++i) {
            P_.block<3, 3>(idx, idx) = Eigen::Matrix3d::Identity() * vio_rot_var;    // 旋转
            idx += 3;
            P_.block<3, 3>(idx, idx) = Eigen::Matrix3d::Identity() * vio_trans_var; // 平移
            idx += 3;
        }

        // GPS外参方差
        P_.block<3, 3>(idx, idx) = Eigen::Matrix3d::Identity() * gps_rot_var;   // 旋转
        idx += 3;
        P_.block<3, 3>(idx, idx) = Eigen::Matrix3d::Identity() * gps_trans_var; // 平移

        // IMU噪声参数
        accel_noise_std_ = 0.1;    // 加速度噪声标准差
        gyro_noise_std_ = 0.01;    // 角速度噪声标准差
        accel_bias_noise_std_ = 0.001; // 加速度计零偏噪声标准差
        gyro_bias_noise_std_ = 0.0001; // 陀螺仪零偏噪声标准差

        // 重力向量
        gravity_ = Eigen::Vector3d(0, 0, -9.81);
    }

    // 处理IMU数据(预测步骤)
    void processIMU(const IMUData& imu_data) {
        if (state_.timestamp == 0.0) {
            state_.timestamp = imu_data.timestamp;
            return;
        }

        // 计算时间差
        double dt = imu_data.timestamp - state_.timestamp;
        if (dt <= 0) {
            throw std::invalid_argument("IMU data timestamp is not increasing");
        }

        // 预测状态
        predict(imu_data, dt);

        // 更新时间戳
        state_.timestamp = imu_data.timestamp;
    }

    // 处理VIO数据(更新步骤)
    void processVIO(const VIOData& vio_data) {
        if (vio_data.id < 0 || vio_data.id >= NumVIOs) {
            throw std::invalid_argument("Invalid VIO ID");
        }

        // 如果还没有IMU数据，不进行更新
        if (state_.timestamp == 0.0) {
            return;
        }

        // 时间同步：预测到VIO数据的时间戳
        double dt = vio_data.timestamp - state_.timestamp;
        if (dt < 0) {
            // 如果VIO数据比当前状态旧，警告但仍处理
            std::cerr << "Warning: VIO data is older than current state" << std::endl;
        } else {
            // 预测到VIO数据的时间
            IMUData dummy_imu;
            dummy_imu.timestamp = vio_data.timestamp;
            dummy_imu.accel = Eigen::Vector3d::Zero();
            dummy_imu.gyro = Eigen::Vector3d::Zero();
            predict(dummy_imu, dt);
        }

        // 执行EKF更新
        updateVIO(vio_data);

        // 更新时间戳
        state_.timestamp = vio_data.timestamp;
    }

    // 处理GPS数据(更新步骤)
    void processGPS(const GPSData& gps_data) {
        // 如果还没有IMU数据，不进行更新
        if (state_.timestamp == 0.0) {
            return;
        }

        // 时间同步：预测到GPS数据的时间戳
        double dt = gps_data.timestamp - state_.timestamp;
        if (dt < 0) {
            // 如果GPS数据比当前状态旧，警告但仍处理
            std::cerr << "Warning: GPS data is older than current state" << std::endl;
        } else {
            // 预测到GPS数据的时间
            IMUData dummy_imu;
            dummy_imu.timestamp = gps_data.timestamp;
            dummy_imu.accel = Eigen::Vector3d::Zero();
            dummy_imu.gyro = Eigen::Vector3d::Zero();
            predict(dummy_imu, dt);
        }

        // 执行EKF更新
        updateGPS(gps_data);

        // 更新时间戳
        state_.timestamp = gps_data.timestamp;
    }

    // 获取当前状态
    const EKFState& getState() const {
        return state_;
    }

    // 获取当前协方差矩阵
    const Eigen::MatrixXd& getCovariance() const {
        return P_;
    }

private:
    // 状态预测
    void predict(const IMUData& imu_data, double dt) {
        // 提取当前状态
        Eigen::Vector3d p = state_.position;
        Eigen::Vector3d v = state_.velocity;
        Eigen::Quaterniond q = state_.orientation;
        Eigen::Vector3d ba = state_.accel_bias;
        Eigen::Vector3d bg = state_.gyro_bias;

        // 处理IMU数据（去除零偏）
        Eigen::Vector3d accel = imu_data.accel - ba;
        Eigen::Vector3d gyro = imu_data.gyro - bg;

        // 姿态更新（使用中值积分）
        Eigen::Quaterniond dq = QuaternionUtils::rotVecToQuat(gyro * dt);
        Eigen::Quaterniond q_new = QuaternionUtils::multiply(q, dq);
        q_new.normalize();

        // 速度更新
        Eigen::Vector3d a_world = QuaternionUtils::rotateVector(q, accel) + gravity_;
        Eigen::Vector3d v_new = v + a_world * dt;

        // 位置更新
        Eigen::Vector3d p_new = p + v * dt + 0.5 * a_world * dt * dt;

        // 更新状态
        state_.position = p_new;
        state_.velocity = v_new;
        state_.orientation = q_new;

        // 计算状态转移矩阵F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(stateSize(), stateSize());

        // 位置对速度的导数
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

        // 速度对姿态的导数
        Eigen::Matrix3d R = QuaternionUtils::quatToRotMat(q);
        Eigen::Matrix3d skew_accel = skewSymmetric(accel);
        F.block<3, 3>(3, 6) = -R * skew_accel * dt;

        // 速度对加速度计零偏的导数
        F.block<3, 3>(3, 9) = -R * dt;

        // 姿态对陀螺仪零偏的导数
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        // 计算噪声协方差矩阵Q
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(stateSize(), stateSize());
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        // 加速度噪声的影响
        Eigen::Matrix3d Q_accel = Eigen::Matrix3d::Identity() * (accel_noise_std_ * accel_noise_std_);
        Q.block<3, 3>(3, 3) += R * Q_accel * R.transpose() * dt2;
        Q.block<3, 3>(0, 0) += R * Q_accel * R.transpose() * dt4 / 4.0;
        Q.block<3, 3>(0, 3) += R * Q_accel * R.transpose() * dt3 / 2.0;
        Q.block<3, 3>(3, 0) += Q.block<3, 3>(0, 3).transpose();

        // 角速度噪声的影响
        Eigen::Matrix3d Q_gyro = Eigen::Matrix3d::Identity() * (gyro_noise_std_ * gyro_noise_std_);
        Q.block<3, 3>(6, 6) += Q_gyro * dt2;

        // 加速度计零偏噪声的影响
        Eigen::Matrix3d Q_ab = Eigen::Matrix3d::Identity() * (accel_bias_noise_std_ * accel_bias_noise_std_);
        Q.block<3, 3>(9, 9) += Q_ab * dt2;

        // 陀螺仪零偏噪声的影响
        Eigen::Matrix3d Q_gb = Eigen::Matrix3d::Identity() * (gyro_bias_noise_std_ * gyro_bias_noise_std_);
        Q.block<3, 3>(12, 12) += Q_gb * dt2;

        // 更新协方差矩阵
        P_ = F * P_ * F.transpose() + Q;
    }

    // 使用VIO数据进行更新
    void updateVIO(const VIOData& vio_data) {
        int vio_id = vio_data.id;

        // 计算观测模型H和观测值z
        Eigen::VectorXd z(9);  // 位置(3) + 速度(3) + 姿态(3，旋转向量)
        Eigen::MatrixXd H(9, stateSize());
        H.setZero();

        // 当前状态
        Eigen::Vector3d p = state_.position;
        Eigen::Vector3d v = state_.velocity;
        Eigen::Quaterniond q = state_.orientation;
        Eigen::Quaterniond q_vio = state_.vio_rotations[vio_id];
        Eigen::Vector3d t_vio = state_.vio_translations[vio_id];

        // VIO数据转换到全局坐标系
        Eigen::Vector3d vio_pos_global = t_vio + QuaternionUtils::rotateVector(q_vio, vio_data.position);
        Eigen::Vector3d vio_vel_global = QuaternionUtils::rotateVector(q_vio, vio_data.velocity);
        Eigen::Quaterniond vio_att_global = QuaternionUtils::multiply(q_vio, vio_data.orientation);

        // 观测值
        z.head<3>() = vio_pos_global;                     // 位置
        z.segment<3>(3) = vio_vel_global;                 // 速度
        z.tail<3>() = QuaternionUtils::quatToRotVec(vio_att_global);  // 姿态(旋转向量)

        // 计算残差
        Eigen::VectorXd x_pred(9);
        x_pred.head<3>() = p;
        x_pred.segment<3>(3) = v;
        x_pred.tail<3>() = QuaternionUtils::quatToRotVec(q);
        Eigen::VectorXd r = z - x_pred;

        // 处理姿态残差的周期性
        for (int i = 0; i < 3; ++i) {
            while (r[i] > M_PI) r[i] -= 2 * M_PI;
            while (r[i] < -M_PI) r[i] += 2 * M_PI;
        }

        // 观测矩阵H
        // 位置相关
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // 对自身位置的导数

        // 速度相关
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // 对自身速度的导数

        // 姿态相关
        H.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();  // 对自身姿态的导数

        // VIO外参相关
        int vio_param_idx = 15 + vio_id * 6;

        // VIO旋转外参对位置的影响
        Eigen::Matrix3d skew_p = skewSymmetric(QuaternionUtils::rotateVector(q_vio, vio_data.position));
        H.block<3, 3>(0, vio_param_idx) = -skew_p;

        // VIO平移外参对位置的影响
        H.block<3, 3>(0, vio_param_idx + 3) = Eigen::Matrix3d::Identity();

        // VIO旋转外参对速度的影响
        Eigen::Matrix3d skew_v = skewSymmetric(QuaternionUtils::rotateVector(q_vio, vio_data.velocity));
        H.block<3, 3>(3, vio_param_idx) = -skew_v;

        // VIO旋转外参对姿态的影响
        H.block<3, 3>(6, vio_param_idx) = Eigen::Matrix3d::Identity();

        // 观测噪声协方差R (使用VIO提供的协方差)
        Eigen::MatrixXd R = vio_data.covariance;

        // 计算卡尔曼增益K = P * H^T * (H * P * H^T + R)^-1
        Eigen::MatrixXd HPH = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * HPH.inverse();

        // 更新状态
        updateStateFromError(K * r);

        // 更新协方差 P = (I - K * H) * P
        P_ = (Eigen::MatrixXd::Identity(stateSize(), stateSize()) - K * H) * P_;
    }

    // 使用GPS数据进行更新
    void updateGPS(const GPSData& gps_data) {
        // 计算观测模型H和观测值z
        Eigen::VectorXd z(6);  // 位置(3) + 速度(3)
        Eigen::MatrixXd H(6, stateSize());
        H.setZero();

        // 当前状态
        Eigen::Vector3d p = state_.position;
        Eigen::Vector3d v = state_.velocity;
        Eigen::Quaterniond q_gps = state_.gps_rotation;
        Eigen::Vector3d t_gps = state_.gps_translation;

        // GPS数据转换到全局坐标系
        Eigen::Vector3d gps_pos_global = t_gps + QuaternionUtils::rotateVector(q_gps, gps_data.position);
        Eigen::Vector3d gps_vel_global = QuaternionUtils::rotateVector(q_gps, gps_data.velocity);

        // 观测值
        z.head<3>() = gps_pos_global;  // 位置
        z.tail<3>() = gps_vel_global;  // 速度

        // 计算残差
        Eigen::VectorXd x_pred(6);
        x_pred.head<3>() = p;
        x_pred.tail<3>() = v;
        Eigen::VectorXd r = z - x_pred;

        // 观测矩阵H
        // 位置相关
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // 对自身位置的导数

        // 速度相关
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // 对自身速度的导数

        // GPS外参索引
        int gps_param_idx = 15 + NumVIOs * 6;

        // GPS旋转外参对位置的影响
        Eigen::Matrix3d skew_p = skewSymmetric(QuaternionUtils::rotateVector(q_gps, gps_data.position));
        H.block<3, 3>(0, gps_param_idx) = -skew_p;

        // GPS平移外参对位置的影响
        H.block<3, 3>(0, gps_param_idx + 3) = Eigen::Matrix3d::Identity();

        // GPS旋转外参对速度的影响
        Eigen::Matrix3d skew_v = skewSymmetric(QuaternionUtils::rotateVector(q_gps, gps_data.velocity));
        H.block<3, 3>(3, gps_param_idx) = -skew_v;

        // 观测噪声协方差R (使用GPS提供的协方差)
        Eigen::MatrixXd R = gps_data.covariance;

        // 计算卡尔曼增益K = P * H^T * (H * P * H^T + R)^-1
        Eigen::MatrixXd HPH = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * HPH.inverse();

        // 更新状态
        updateStateFromError(K * r);

        // 更新协方差 P = (I - K * H) * P
        P_ = (Eigen::MatrixXd::Identity(stateSize(), stateSize()) - K * H) * P_;
    }

    // 从误差向量更新状态
    void updateStateFromError(const Eigen::VectorXd& dx) {
        // 位置更新
        state_.position += dx.head<3>();

        // 速度更新
        state_.velocity += dx.segment<3>(3);

        // 姿态更新（旋转向量）
        Eigen::Vector3d d_rot = dx.segment<3>(6);
        Eigen::Quaterniond dq = QuaternionUtils::rotVecToQuat(d_rot);
        state_.orientation = QuaternionUtils::multiply(state_.orientation, dq);
        state_.orientation.normalize();

        // IMU零偏更新
        state_.accel_bias += dx.segment<3>(9);
        state_.gyro_bias += dx.segment<3>(12);

        // VIO外参更新
        int idx = 15;
        for (int i = 0; i < NumVIOs; ++i) {
            // 旋转外参更新
            Eigen::Vector3d d_vio_rot = dx.segment<3>(idx);
            Eigen::Quaterniond dq_vio = QuaternionUtils::rotVecToQuat(d_vio_rot);
            state_.vio_rotations[i] = QuaternionUtils::multiply(state_.vio_rotations[i], dq_vio);
            state_.vio_rotations[i].normalize();
            idx += 3;

            // 平移外参更新
            state_.vio_translations[i] += dx.segment<3>(idx);
            idx += 3;
        }

        // GPS外参更新
        // 旋转外参更新
        Eigen::Vector3d d_gps_rot = dx.segment<3>(idx);
        Eigen::Quaterniond dq_gps = QuaternionUtils::rotVecToQuat(d_gps_rot);
        state_.gps_rotation = QuaternionUtils::multiply(state_.gps_rotation, dq_gps);
        state_.gps_rotation.normalize();
        idx += 3;

        // 平移外参更新
        state_.gps_translation += dx.segment<3>(idx);
    }

    // 计算状态向量大小
    int stateSize() const {
        // 位置(3) + 速度(3) + 姿态(3) + 加速度计零偏(3) + 陀螺仪零偏(3) 
        // + 每个VIO外参(6) * NumVIOs + GPS外参(6)
        return 3 + 3 + 3 + 3 + 3 + NumVIOs * 6 + 6;
    }

    // 计算反对称矩阵
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d m;
        m << 0, -v.z(), v.y(),
                v.z(), 0, -v.x(),
                -v.y(), v.x(), 0;
        return m;
    }

    EKFState state_;                // EKF状态
    Eigen::MatrixXd P_;             // 协方差矩阵

    // IMU噪声参数
    double accel_noise_std_;        // 加速度噪声标准差
    double gyro_noise_std_;         // 角速度噪声标准差
    double accel_bias_noise_std_;   // 加速度计零偏噪声标准差
    double gyro_bias_noise_std_;    // 陀螺仪零偏噪声标准差

    Eigen::Vector3d gravity_;       // 重力向量
};

#endif // MULTI_SENSOR_EKF_H
    