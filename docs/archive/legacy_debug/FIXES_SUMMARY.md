# SchurVINS 代码修正总结

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 坐标系说明

本项目使用 **NED (North-East-Down) 坐标系**：
- **N (North)**: x轴指向北
- **E (East)**: y轴指向东
- **D (Down)**: z轴指向下（重力方向）
- **重力**: g = [0, 0, 9.81]^T （指向下）

## 机体坐标系定义

标准的机体坐标系（Body Frame）定义：
- **x_body**: Forward（前）
- **y_body**: Right（右）
- **z_body**: Down（下）

## 主要修正内容

### 1. 仿真器坐标系修正

#### 文件：`vio_frontend_simulator.cpp` 和 `vio_frontend_simulator1.cpp`

**问题**：原代码中旋转矩阵构建不符合标准的NED机体坐标系定义。

**修正前**：
```cpp
Eigen::Vector3d forward_dir(-sin(angle), cos(angle), 0.0);
Eigen::Vector3d down_dir(0.0, 0.0, 1.0);
Eigen::Vector3d right_dir = down_dir.cross(forward_dir).normalized();

Eigen::Matrix3d R;
R.col(0) = forward_dir;
R.col(1) = down_dir;      // 错误：y轴不应该是down
R.col(2) = -right_dir;
```

**修正后**：
```cpp
// NED坐标系：机体坐标系为 x-前(Forward), y-右(Right), z-下(Down)
Eigen::Vector3d forward_dir(-sin(angle), cos(angle), 0.0);
forward_dir.normalize();
Eigen::Vector3d down_dir(0.0, 0.0, 1.0);
Eigen::Vector3d right_dir = forward_dir.cross(down_dir);  // 右 = 前 × 下
right_dir.normalize();

// 构建旋转矩阵 R_bn (body to NED)
Eigen::Matrix3d R_bn;
R_bn.col(0) = forward_dir;  // 机体x轴（前）在NED中的表示
R_bn.col(1) = right_dir;    // 机体y轴（右）在NED中的表示
R_bn.col(2) = down_dir;     // 机体z轴（下）在NED中的表示
current.q = Eigen::Quaterniond(R_bn);
```

**理由**：
- 旋转矩阵 R_bn 的每一列代表机体坐标系的各个轴在NED坐标系中的表示
- 机体系应该是 FRD (Forward-Right-Down)

### 2. 观测模型投影修正

#### 文件：`eskf\schur_vins.cpp`

**问题**：外参平移的处理顺序不正确。

**修正前**：
```cpp
const auto d_ij_w = lmk->position - frm->p();
const auto d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
const auto d_cj_c = Ric.transpose() * d_cj_i;
```

**修正后**：
```cpp
const auto d_ij_w = lmk->position - frm->p();
const auto d_ij_b = Rwi.transpose() * d_ij_w;  // 世界系到机体系
const auto d_cj_c = Ric.transpose() * (d_ij_b - ext_.t_ic);  // 机体系到相机系
```

**理由**：
- 先将路标点从世界系转到机体系
- 再减去外参平移（机体系表示）
- 最后通过旋转转到相机系
- 正确的公式：P_c = R_cb * (R_wb^T * (P_w - P_wb) - t_bc)

**对应的Jacobian修正**：
```cpp
Mat2_6 J_ext;
J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_ij_b - ext_.t_ic);
```

### 3. 观测噪声处理修正（USE_QR方法）

#### 文件：`eskf\schur_vins.cpp` (第488-509行)

**问题**：观测噪声方差错误地除以了时间间隔dt。

**修正前**：
```cpp
TYPE r = uv_var / dt;
```

**修正后**：
```cpp
// 观测噪声方差：QR分解不改变噪声方差
TYPE r = uv_var;
```

**理论依据**：
- QR分解：J_lmk = Q * [R; 0]
- 左乘 Q^T：Q^T * [J_pose, J_lmk] * dx = Q^T * e
- 噪声变换：Q^T * n ~ N(0, Q^T * R * Q) = N(0, σ² * I)
- 因为Q是正交矩阵，所以 **噪声方差保持不变**

同样修正了 landmark 更新部分（第525-543行）。

### 4. 观测噪声处理修正（Schur补方法）

#### 文件：`eskf\schur_vins.cpp` (第688行和752行)

**问题**：特征值分解后的噪声方差计算错误。

**修正前**：
```cpp
const auto R = uv_var / es.eigenvalues()(zero_end) / dt;
```

**修正后**：
```cpp
// 特征值分解后的观测噪声方差
// 由于 H = J^T * (1/σ²) * J，特征值λ = (1/σ²) * λ_obs
// 所以 σ_i² = σ² / λ_i
const auto R = uv_var / es.eigenvalues()(zero_end);
```

**理论依据**：
- Hessian矩阵：H = J^T * R^{-1} * J = (1/σ²) * J^T * J
- 特征值分解：H = V * Λ * V^T
- 在特征空间中，观测方程变为：λ_i * v_i^T * dx = v_i^T * g + n_i
- 其中 n_i ~ N(0, σ²/λ_i)

### 5. 协方差对称性保证

#### 文件：`eskf\schur_vins.cpp` (第210行)

**问题**：非DEBUG模式下，协方差预测后缺少对称化处理。

**修正**：添加了对称化处理
```cpp
// 确保对称性
cov = 0.5 * (cov + cov.transpose());
```

**理由**：
- 由于浮点运算误差，协方差矩阵可能失去对称性
- ESKF要求协方差矩阵必须是对称正定的
- 强制对称化确保数值稳定性

## NED坐标系下的公式验证

### IMU运动学方程

在NED坐标系下，状态方程为：

```
dp/dt = v
dv/dt = R_nb * (a_m - b_a) + g
dq/dt = 0.5 * q ⊗ ω_b
```

其中：
- p: 位置（NED系）
- v: 速度（NED系）
- q: 四元数（NED到Body）
- a_m: 加速度计测量值（Body系）
- ω_b: 角速度（Body系，ω_b = ω_m - b_g）
- g = [0, 0, 9.81]^T: 重力（NED系，向下）
- b_a, b_g: 加速度计和陀螺仪偏置

**代码验证**（vio_frontend_simulator.cpp 第131行）：
```cpp
Eigen::Vector3d acc_ideal = curr.q.inverse() * ((curr.v - prev.v) / dt - g);
```

展开：
- (curr.v - prev.v) / dt = dv/dt（速度变化率，NED系）
- dv/dt - g = a_true（真实加速度，NED系）
- curr.q.inverse() * a_true = R_nb * a_true（转到Body系）
- 加速度计测量 = R_nb * (dv/dt - g) ✓ 正确

### 观测模型

路标点投影到相机的观测模型：

```
P_c = R_cb * (R_wb^T * (P_w - P_wb) - t_bc)
z = [P_c.x / P_c.z, P_c.y / P_c.z]^T
```

其中：
- P_w: 路标在世界系（NED）的位置
- P_wb: 机体在世界系的位置
- R_wb: 世界到机体的旋转
- R_cb: 机体到相机的旋转
- t_bc: 机体到相机的平移（机体系表示）

**代码验证**（修正后）：
```cpp
const auto d_ij_w = lmk->position - frm->p();          // P_w - P_wb
const auto d_ij_b = Rwi.transpose() * d_ij_w;          // R_wb^T * (P_w - P_wb)
const auto d_cj_c = Ric.transpose() * (d_ij_b - ext_.t_ic);  // R_cb * (... - t_bc)
```
✓ 与理论公式一致

### Jacobian推导

**对路标位置的Jacobian**：
```
∂z/∂P_w = J_proj * R_cw
```
其中 R_cw = R_cb * R_wb^T = (R_wb * R_bc)^T

**代码验证**：
```cpp
Mat2_3 J_lmk = J * (frm->q() * ext_.q_ic).inverse().toRotationMatrix();
```
✓ 正确

**对机体位置的Jacobian**：
```
∂z/∂P_wb = -J_proj * R_cw
```

**代码验证**：
```cpp
J_pose.rightCols<3>().noalias() = -J_lmk;
```
✓ 正确

**对机体姿态的Jacobian**：
```
∂z/∂δθ = J_proj * R_cb * [R_wb^T * (P_w - P_wb)]×
       = J_lmk * [d_ij_w]×
```

**代码验证**：
```cpp
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);
```
✓ 正确

## 仍需注意的问题

### 1. 协方差初始化
当前的初始协方差可能过小（乘以0.1），建议根据实际情况调整：
```cpp
// common.h
constexpr static double STB_Q_INIT = 3e-2 * 1e-1;  // 可能需要增大
constexpr static double STB_P_INIT = 1e-0 * 1e-1;  // 可能需要增大
```

### 2. 外参校准
当前外参设为单位变换：
```cpp
struct ExtState {
    Quat q_ic = Quat::Identity();
    Vec3 t_ic = Vec3::Zero();
};
```
如果相机和IMU不重合，需要设置正确的外参。

### 3. 重力估计
如果启用重力在线估计（ESTIMATE_GRAVITY = true），需要确保有足够的激励运动，否则重力方向可能漂移。

### 4. 观测噪声参数
当前设置：
```cpp
constexpr static TYPE uv_var = TYPE(400);  // 20^2，像素方差
```
这个值对应标准差20像素，可能偏大。建议根据实际相机噪声调整（通常1-5像素）。

### 5. 高度修正
仿真中设置：
```cpp
current.p.z() = 1.0;  // Down保持1m
```
在NED系中，z正向是向下，所以z=1表示在地面下1米。如果要在空中飞行，应该设为负值：
```cpp
current.p.z() = -1.0;  // 在地面上方1m
```

## 编译和测试

修改完成后，建议：
1. 重新编译项目
2. 运行仿真，观察姿态、位置、速度的估计精度
3. 检查协方差矩阵是否合理（不应该过大或过小）
4. 对比两个仿真器的结果是否一致

## 参考文献

- MSCKF: "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"
- SchurVINS: 基于Schur补的VIO算法
- 标准坐标系定义：NED (North-East-Down) 和 FRD (Forward-Right-Down)
