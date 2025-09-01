#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// 工具函数：四元数归一化
Quaterniond normalizeQuat(const Quaterniond& q) {
    double norm = q.norm();
    if (norm < 1e-8) return Quaterniond::Identity();
    return q / norm;
}

// 工具函数：四元数转旋转矩阵
Matrix3d quat2Rot(const Quaterniond& q) {
    return normalizeQuat(q).toRotationMatrix();
}

// 工具函数：叉乘矩阵（v^）
Matrix3d skew(const Vector3d& v) {
    Matrix3d m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

// 工具函数：IMU预积分（预测IMU状态增量）
struct IMUIntegral {
    double dt;                // 积分时间步
    Vector3d dP;              // 位置增量（IMU系）
    Vector3d dV;              // 速度增量（IMU系）
    Quaterniond dQ;           // 姿态增量（IMU系）

    // 输入IMU原始数据、当前偏置，计算预积分结果
    void compute(const Vector3d& omega, const Vector3d& acc,
                 const Vector3d& bg, const Vector3d& ba, double dt_) {
        dt = dt_;
        Vector3d omega_true = omega - bg;  // 去偏角速度
        Vector3d acc_true = acc - ba;      // 去偏加速度

        // 一阶预积分（简化版，高精度场景可用中值积分）
        dQ = Quaterniond(AngleAxisd(omega_true.norm() * dt, omega_true.normalized()));
        Matrix3d R_prev = Matrix3d::Identity();  // 积分前姿态（相对）
        dV = R_prev * acc_true * dt;
        dP = 0.5 * R_prev * acc_true * dt * dt;
    }
};

// VIO数据结构（每个相机的输出）
struct VIOData {
    int cam_id;               // 相机ID（0,1,...N-1）
    double timestamp;         // 时间戳
    Vector3d P_CW;            // 相机在世界系的位置
    Vector3d V_CW;            // 相机在世界系的速度
    Quaterniond Q_CW;         // 世界系→相机系的姿态
    Matrix<double, 9, 9> R;   // 测量噪声矩阵（3P+3V+3Q）
};

class MultiVIOIMUEKF {
public:
    // 构造函数（输入相机数量、IMU噪声参数）
    MultiVIOIMUEKF(int num_cams,
                   const Vector3d& sigma_omega,  // 陀螺仪噪声标准差
                   const Vector3d& sigma_acc,    // 加速度计噪声标准差
                   const Vector3d& sigma_bg,     // 陀螺仪偏置游走标准差
                   const Vector3d& sigma_ba)     // 加速度计偏置游走标准差
            : num_cams_(num_cams) {
        // 1. 初始化状态向量X（16 + 7*num_cams）
        int state_dim = 16 + 7 * num_cams_;
        X_.resize(state_dim);
        X_.setZero();
        // 初始姿态设为单位四元数
        X_.segment<4>(6) = Quaterniond::Identity().coeffs();  // Q_IW: X[6-9]
        // 初始外参设为单位（后续由VIO观测优化）
        for (int i = 0; i < num_cams_; ++i) {
            int idx = 16 + 7 * i;
            X_.segment<4>(idx) = Quaterniond::Identity().coeffs();  // Q_ICi
            X_.segment<3>(idx + 4).setZero();                       // T_ICi
        }

        // 2. 初始化协方差矩阵P（状态维度×状态维度）
        P_.resize(state_dim, state_dim);
        P_.setZero();
        // 初始状态噪声（根据实际情况调整）
        P_.block<3,3>(0,0) = 1e-4 * Matrix3d::Identity();  // P_IW
        P_.block<3,3>(3,3) = 1e-2 * Matrix3d::Identity();  // V_IW
        P_.block<4,4>(6,6) = 1e-6 * Matrix4d::Identity();  // Q_IW
        P_.block<3,3>(10,10) = 1e-3 * Matrix3d::Identity(); // ba
        P_.block<3,3>(13,13) = 1e-3 * Matrix3d::Identity(); // bg
        // 外参初始噪声（较大，允许优化）
        for (int i = 0; i < num_cams_; ++i) {
            int idx = 16 + 7 * i;
            P_.block<4,4>(idx, idx) = 1e-2 * Matrix4d::Identity();  // Q_ICi
            P_.block<3,3>(idx+4, idx+4) = 1e-1 * Matrix3d::Identity(); // T_ICi
        }

        // 3. 初始化过程噪声矩阵Q（预测阶段用）
        Q_.resize(state_dim, state_dim);
        Q_.setZero();
        double dt = 1e-3;  // 假设IMU频率1000Hz（实际由IMU数据时间步决定）
        double sigma_omega2 = sigma_omega.squaredNorm() * dt;
        double sigma_acc2 = sigma_acc.squaredNorm() * dt;
        double sigma_bg2 = sigma_bg.squaredNorm() * dt;
        double sigma_ba2 = sigma_ba.squaredNorm() * dt;
        // IMU姿态噪声（映射到四元数）
        Q_.block<4,4>(6,6) = sigma_omega2 * Matrix4d::Identity();
        // IMU速度噪声
        Q_.block<3,3>(3,3) = sigma_acc2 * Matrix3d::Identity();
        // IMU位置噪声
        Q_.block<3,3>(0,0) = 0.5 * sigma_acc2 * dt * dt * Matrix3d::Identity();
        // 偏置游走噪声
        Q_.block<3,3>(10,10) = sigma_ba2 * Matrix3d::Identity();
        Q_.block<3,3>(13,13) = sigma_bg2 * Matrix3d::Identity();
        // 外参固定：过程噪声设为极小（抑制无观测时的漂移）
        for (int i = 0; i < num_cams_; ++i) {
            int idx = 16 + 7 * i;
            Q_.block<4,4>(idx, idx) = 1e-12 * Matrix4d::Identity();  // Q_ICi
            Q_.block<3,3>(idx+4, idx+4) = 1e-12 * Matrix3d::Identity(); // T_ICi
        }

        // 4. 重力向量（ENU系：z轴向上）
        g_ = Vector3d(0, 0, 9.81);
        last_imu_time_ = -1;
    }

    // 预测阶段：用IMU数据更新状态和协方差
    void predict(const Vector3d& omega, const Vector3d& acc, double timestamp) {
        if (last_imu_time_ < 0) {  // 首次IMU数据，初始化时间
            last_imu_time_ = timestamp;
            return;
        }
        double dt = timestamp - last_imu_time_;
        last_imu_time_ = timestamp;
        if (dt < 1e-6) return;  // 时间步过小，跳过

        // 1. 提取当前状态
        Vector3d P_IW = X_.segment<3>(0);
        Vector3d V_IW = X_.segment<3>(3);
        Quaterniond Q_IW = normalizeQuat(Quaterniond(X_.segment<4>(6)));
        Vector3d ba = X_.segment<3>(10);
        Vector3d bg = X_.segment<3>(13);

        // 2. IMU预积分（计算状态增量）
        IMUIntegral integral;
        integral.compute(omega, acc, bg, ba, dt);

        // 3. 更新IMU状态（世界系）
        Matrix3d R_IW = quat2Rot(Q_IW);
        // 位置更新：P = P + V*dt + 0.5*R*acc_true*dt²
        X_.segment<3>(0) = P_IW + V_IW * dt + 0.5 * R_IW * (acc - ba) * dt * dt;
        // 速度更新：V = V + R*acc_true*dt + g*dt
        X_.segment<3>(3) = V_IW + R_IW * (acc - ba) * dt + g_ * dt;
        // 姿态更新：Q = Q * dQ（右乘增量，对应IMU系相对运动）
        Quaterniond Q_new = Q_IW * integral.dQ;
        X_.segment<4>(6) = normalizeQuat(Q_new).coeffs();
        // 偏置更新：假设偏置缓慢变化，暂不更新（由过程噪声体现游走）

        // 4. 状态转移矩阵F（简化版，忽略高阶小项）
        int state_dim = X_.size();
        MatrixXd F = MatrixXd::Identity(state_dim, state_dim);
        // 姿态对陀螺仪偏置的导数（简化为 -R_IW * dt）
        F.block<3,3>(6,13) = -R_IW * dt;
        // 速度对加速度计偏置的导数（-R_IW * dt）
        F.block<3,3>(3,10) = -R_IW * dt;
        // 位置对加速度计偏置的导数（-0.5*R_IW*dt²）
        F.block<3,3>(0,10) = -0.5 * R_IW * dt * dt;

        // 5. 协方差预测：P = F * P * F^T + Q
        P_ = F * P_ * F.transpose() + Q_;
        // 确保协方差对称（数值稳定性）
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // 更新阶段：用VIO数据更新状态（含外参）
    void update(const VIOData& vio_data) {
        int cam_id = vio_data.cam_id;
        if (cam_id < 0 || cam_id >= num_cams_) {
            cout << "Invalid camera ID: " << cam_id << endl;
            return;
        }

        // 1. 提取当前状态
        Vector3d P_IW = X_.segment<3>(0);
        Vector3d V_IW = X_.segment<3>(3);
        Quaterniond Q_IW = normalizeQuat(Quaterniond(X_.segment<4>(6)));
        Matrix3d R_IW = quat2Rot(Q_IW);
        // 提取相机外参
        int cam_state_idx = 16 + 7 * cam_id;
        Quaterniond Q_IC = normalizeQuat(Quaterniond(X_.segment<4>(cam_state_idx)));
        Vector3d T_IC = X_.segment<3>(cam_state_idx + 4);
        Matrix3d R_IC = quat2Rot(Q_IC);

        // 2. VIO观测转换为IMU状态预测（核心：坐标系转换）
        // 相机位置 → IMU位置：P_CW = P_IW + R_IW * T_IC → 预测IMU位置：P_IW_pred = P_CW - R_IW * T_IC
        Vector3d P_IW_pred = vio_data.P_CW - R_IW * T_IC;
        // 相机速度 → IMU速度：V_CW = V_IW + skew(R_IW * omega_IW) * T_IC → 简化为 V_IW_pred = V_CW（忽略牵连速度，高精度场景需补充）
        Vector3d V_IW_pred = vio_data.V_CW;
        // 相机姿态 → IMU姿态：Q_CW = Q_IC * Q_IW → 预测IMU姿态：Q_IW_pred = Q_IC.inverse() * Q_CW
        Quaterniond Q_IW_pred = Q_IC.inverse() * vio_data.Q_CW;
        Q_IW_pred = normalizeQuat(Q_IW_pred);

        // 3. 计算测量残差r（9维：3P+3V+3Q）
        Matrix<double, 9, 1> r;
        r.segment<3>(0) = P_IW - P_IW_pred;                          // 位置残差
        r.segment<3>(3) = V_IW - V_IW_pred;                          // 速度残差
        r.segment<3>(6) = 2 * (Q_IW_pred.inverse() * Q_IW).vec();    // 姿态残差（四元数差映射到向量）

        // 4. 构建观测矩阵H（9×state_dim）
        int state_dim = X_.size();
        MatrixXd H = MatrixXd::Zero(9, state_dim);
        // H对IMU位置的导数（P_IW直接影响残差）
        H.block<3,3>(0,0) = Matrix3d::Identity();
        // H对IMU速度的导数（V_IW直接影响残差）
        H.block<3,3>(3,3) = Matrix3d::Identity();
        // H对IMU姿态的导数（Q_IW影响位置和姿态残差）
        H.block<3,3>(0,6) = -skew(R_IW * T_IC);  // 位置残差对Q_IW的导数（映射到四元数向量）
        H.block<3,3>(6,6) = Matrix3d::Identity(); // 姿态残差对Q_IW的导数
        // H对相机外参的导数（Q_IC和T_IC影响预测值）
        H.block<3,3>(0, cam_state_idx + 4) = -R_IW;  // 位置残差对T_IC的导数
        H.block<3,3>(6, cam_state_idx) = -Matrix3d::Identity();  // 姿态残差对Q_IC的导数

        // 5. 卡尔曼增益计算：K = P * H^T * (H*P*H^T + R)^-1
        MatrixXd S = H * P_ * H.transpose() + vio_data.R;
        MatrixXd K = P_ * H.transpose() * S.inverse();

        // 6. 状态更新：X = X + K*r
        X_ += K * r;
        // 四元数归一化（避免数值漂移）
        X_.segment<4>(6) = normalizeQuat(Quaterniond(X_.segment<4>(6))).coeffs();
        for (int i = 0; i < num_cams_; ++i) {
            int idx = 16 + 7 * i;
            X_.segment<4>(idx) = normalizeQuat(Quaterniond(X_.segment<4>(idx))).coeffs();
        }

        // 7. 协方差更新：P = (I - K*H) * P
        P_ = (MatrixXd::Identity(state_dim, state_dim) - K * H) * P_;
        // 确保协方差对称
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // 获取当前融合后的IMU状态
    void getIMUState(Vector3d& P_IW, Vector3d& V_IW, Quaterniond& Q_IW) const {
        P_IW = X_.segment<3>(0);
        V_IW = X_.segment<3>(3);
        Q_IW = normalizeQuat(Quaterniond(X_.segment<4>(6)));
    }

    // 获取相机外参
    bool getCameraExtrinsic(int cam_id, Quaterniond& Q_IC, Vector3d& T_IC) const {
        if (cam_id < 0 || cam_id >= num_cams_) return false;
        int idx = 16 + 7 * cam_id;
        Q_IC = normalizeQuat(Quaterniond(X_.segment<4>(idx)));
        T_IC = X_.segment<3>(idx + 4);
        return true;
    }

private:
    int num_cams_;              // 相机数量
    VectorXd X_;                // 状态向量
    MatrixXd P_;                // 协方差矩阵
    MatrixXd Q_;                // 过程噪声矩阵
    Vector3d g_;                // 重力向量（ENU系）
    double last_imu_time_;      // 上一帧IMU时间戳
};