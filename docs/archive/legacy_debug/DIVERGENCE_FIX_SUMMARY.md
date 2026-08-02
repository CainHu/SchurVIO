# SchurVINS 发散问题修复总结

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 发散现象
```
YPR: GT = 2.81243 -2.7726 3.02544, EST = 1.8774 -2.40684 0.102834
POS: GT = -3.05893 -3.95512 1, EST = -673017 1.62013e+06 -7.01975e+06
VEL: GT = 0.791024 -0.611785 0, EST = 124061 53020.4 -47478.5
```

位置误差达到**百万级别**，这是典型的数值发散。

## 根本原因分析

### 1. 观测噪声方差错误（最严重）✓ 已修复

**问题**：
```cpp
constexpr static TYPE uv_var = TYPE(400);  // σ² = 400，σ = 20像素
```

**分析**：
- 仿真器输出的是**归一化平面坐标**，不是像素坐标
- 代码第209行：`Eigen::Vector2d un_pt {(uv.x() - cx) / fx, (uv.y() - cy) / fy}`
- 像素噪声：σ_pixel = 1.0 pixel
- 归一化噪声：σ_norm = σ_pixel / focal ≈ 1.0 / 320 ≈ 0.003
- 方差：σ_norm² ≈ 0.00001

**影响**：
- uv_var = 400 意味着滤波器认为观测标准差是 √400 = 20（归一化坐标）
- 这相当于 20 * 320 = 6400 像素的误差！
- 滤波器完全不信任观测，只依赖IMU预测
- IMU累积误差导致快速发散

**修复**：
```cpp
// eskf/schur_vins.h
constexpr static TYPE uv_var = TYPE(0.0001);  // σ_norm² ≈ 0.0001
```

### 2. 初始协方差过小 ✓ 已修复

**问题**：
```cpp
constexpr static double STB_Q_INIT = 3e-2 * 1e-1;  // 实际值 = 3e-3
constexpr static double STB_P_INIT = 1e-0 * 1e-1;  // 实际值 = 1e-1
```

**影响**：
- 初始协方差太小，滤波器过于自信初始状态
- 即使观测给出修正，滤波器也不接受大的更新
- 导致估计锁死在错误状态附近

**修复**：
```cpp
// common.h - 移除所有的 * 1e-1
constexpr static double STB_Q_INIT = 3e-2;
constexpr static double STB_P_INIT = 1e-0;
constexpr static double STB_V_INIT = 1e-1;
constexpr static double STB_BG_INIT = 5e-2;
constexpr static double STB_BA_INIT = 5e-1;
constexpr static double STB_G_INIT = 1e-2;

constexpr static double STB_Q_PROC = 4e-2;
constexpr static double STB_P_PROC = 3e-3;
constexpr static double STB_V_PROC = 2e-1;
constexpr static double STB_BG_PROC = 1e-2;
constexpr static double STB_BA_PROC = 1e-1;
constexpr static double STB_G_PROC = 1e-3;
```

### 3. 观测Jacobian错误 ✓ 已修复

**问题**：
```cpp
// 错误：使用世界系下的向量
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);
```

**正确推导**：

观测模型：
```
P_c = R_cb * R_wb^T * (P_w - P_wb) - R_cb * t_bc
```

对姿态误差δθ求导：
```
δR_wb = R_wb * exp([δθ×]) ≈ R_wb * (I - [δθ×])
δR_wb^T ≈ (I + [δθ×]) * R_wb^T

∂P_c/∂δθ = R_cb * [δθ×] * R_wb^T * (P_w - P_wb)
         = R_cb * [δθ×] * d_ij_b
         = R_cw * [d_ij_b]×
```

**关键**：反对称矩阵应该用**机体系下的向量** d_ij_b，而不是世界系的 d_ij_w！

**修复**：
```cpp
// eskf/schur_vins.cpp (两处)
Mat2_6 J_pose;
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_b);  // 使用 d_ij_b
J_pose.rightCols<3>().noalias() = -J_lmk;
```

## 修复验证

### 理论验证

**正确的Jacobian**：
```
d_ij_w = P_w - P_wb  (世界系)
d_ij_b = R_wb^T * d_ij_w  (机体系)
d_ij_c = R_cb * (d_ij_b - t_bc)  (相机系)

∂d_ij_c/∂δθ = R_cb * [d_ij_b]×
∂d_ij_c/∂P_wb = -R_cb * R_wb^T = -R_cw

J_proj = [1/z, 0, -x/z²]
         [0, 1/z, -y/z²]

H_θ = J_proj * ∂d_ij_c/∂δθ = J_proj * R_cb * [d_ij_b]×
H_p = J_proj * ∂d_ij_c/∂P_wb = -J_proj * R_cw
```

### 数值稳定性改进

1. **观测噪声适配归一化坐标**：σ_norm² = 0.0001（合理范围）
2. **初始协方差增大10倍**：允许更大的初始不确定性
3. **Jacobian符号正确**：使用正确的坐标系向量

## 预期效果

修复后应该观察到：
1. **位置误差**：从百万级别降到米级别
2. **速度误差**：从数万级别降到 m/s 级别
3. **姿态误差**：保持在度级别
4. **估计收敛**：观测更新后误差逐渐减小
5. **协方差合理**：反映真实不确定性

## 其他建议

### 1. 添加数值稳定性检查

在更新后检查状态合理性：
```cpp
// 在 updateState 后添加
if (state_.position.norm() > 1000.0) {
    std::cerr << "Warning: position diverging!" << std::endl;
}
if (state_.velocity.norm() > 100.0) {
    std::cerr << "Warning: velocity diverging!" << std::endl;
}
```

### 2. 调整观测噪声

如果仍然不稳定，可以微调：
```cpp
// 更保守：增大观测噪声
constexpr static TYPE uv_var = TYPE(0.0004);  // σ = 0.02

// 更激进：减小观测噪声（需要准确的相机标定）
constexpr static TYPE uv_var = TYPE(0.00005);  // σ = 0.007
```

### 3. 检查特征点深度

添加调试输出验证特征点深度都为正：
```cpp
if (d_cj_c.z() <= 0) {
    std::cerr << "Warning: negative depth " << d_cj_c.z() << std::endl;
    continue;  // 跳过负深度点
}
```

### 4. 监控协方差矩阵

```cpp
std::cout << "Cov trace: " << cov_.trace() << std::endl;
if (cov_.trace() > 1e6) {
    std::cerr << "Warning: covariance exploding!" << std::endl;
}
```

## 修改文件列表

1. ✅ `eskf/schur_vins.h` - 观测噪声方差
2. ✅ `common.h` - 初始协方差和过程噪声
3. ✅ `eskf/schur_vins.cpp` - Jacobian修正（两处）

## 总结

**最严重的问题**是观测噪声方差设置错误（400 vs 0.0001，相差4000倍）。这导致滤波器完全不信任视觉观测，纯依赖IMU积分，快速累积漂移。

配合初始协方差过小和Jacobian错误，系统完全失去了可观测性，导致百万级别的位置误差。

修复这三个问题后，系统应该能够正常收敛。如果仍有小的漂移，可以微调噪声参数。

## 下一步

1. **重新编译**项目
2. **运行仿真**
3. **观察误差曲线**：应该看到误差在合理范围内波动
4. **如果仍有问题**：添加调试输出，检查中间变量
