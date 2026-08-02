# 发散问题诊断和修复

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 已发现的问题

### 问题1：观测噪声方差过大 ✓ 已修复
**现象**：`uv_var = 400` (标准差20像素)

**问题**：
- 观测数据是归一化平面坐标，不是像素坐标
- 像素噪声 σ_pixel = 1.0
- 归一化：σ_norm = σ_pixel / focal ≈ 1/320 ≈ 0.003
- 方差：σ_norm² ≈ 0.00001

**修复**：
```cpp
// eskf/schur_vins.h
constexpr static TYPE uv_var = TYPE(0.0001);  // 从400改为0.0001
```

### 问题2：初始协方差过小 ✓ 已修复
**问题**：所有初始协方差都乘以了 0.1，导致滤波器过于自信

**修复**：
```cpp
// common.h
constexpr static double STB_Q_INIT = 3e-2;   // 移除 * 1e-1
constexpr static double STB_P_INIT = 1e-0;   // 移除 * 1e-1
// ... 所有噪声参数都移除了 * 1e-1
```

### 问题3：ESKF姿态更新（已验证是正确的）
当前实现：
```cpp
state_.orientation = (vec2quat(dx_q) * state_.orientation).normalized();
```

这是**正确的**！因为：
- ESKF误差四元数：q_true = δq ⊗ q_nominal（左乘）
- dx_q 是误差四元数的向量表示
- 左乘是正确的

## 需要进一步检查的问题

### 问题4：观测模型Jacobian的符号

**当前代码**（第416行）：
```cpp
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);
```

**推导验证**：

观测模型：
```
P_c = R_cb * R_wb^T * (P_w - P_wb) - R_cb * t_bc
z = [P_c.x/P_c.z, P_c.y/P_c.z]^T
```

对姿态误差 δθ 的导数：
```
R_wb_true = R_wb * exp([δθ×])
         ≈ R_wb * (I - [δθ×])
R_wb_true^T ≈ (I + [δθ×]) * R_wb^T
```

所以：
```
∂P_c/∂δθ = R_cb * [δθ×] * R_wb^T * (P_w - P_wb)
         = R_cb * R_wb^T * [R_wb^T * (P_w - P_wb)]×
         = R_cw * [d_ij_b]×
```

但这里有个问题！**d_ij_w 是世界系下的向量**，而我们需要的是**机体系下的向量**！

正确的应该是：
```cpp
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_b);  // 使用 d_ij_b 而不是 d_ij_w
```

### 问题5：序贯更新中的残差计算

在序贯更新中（第507行）：
```cpp
auto e = e_red(j) - hT.dot(dx_p);
```

这个残差更新可能有问题。标准的序贯卡尔曼更新应该是：
```
innovation = z - H * x_prior
```

在我们的情况下：
- e_red(j) 是原始残差（z - H * x_0）
- 但经过多次序贯更新后，x已经不是x_0了
- 应该用**更新后的状态**来计算新的innovation

让我检查这个逻辑...

## 立即需要修复的问题

### 修复1：Jacobian使用正确的向量
