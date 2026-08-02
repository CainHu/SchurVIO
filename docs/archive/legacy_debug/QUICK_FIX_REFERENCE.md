# 快速修复参考

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 三个关键修复（必须全部应用）

### 1. 观测噪声方差 - eskf/schur_vins.h:104
```cpp
// 修改前
constexpr static TYPE uv_var = TYPE(400);

// 修改后
constexpr static TYPE uv_var = TYPE(0.0001);
```
**原因**：仿真器输出归一化坐标（σ≈0.003），不是像素坐标（σ=1-20）

---

### 2. 初始协方差 - common.h:76-88
```cpp
// 修改前（所有参数都乘以0.1）
constexpr static double STB_Q_INIT = 3e-2 * 1e-1;
constexpr static double STB_P_INIT = 1e-0 * 1e-1;
// ...

// 修改后（移除 * 1e-1）
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
**原因**：协方差太小，滤波器过于自信

---

### 3. Jacobian修正 - eskf/schur_vins.cpp:415-417 (两处)
```cpp
// 修改前
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);

// 修改后
J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_b);
```
**原因**：对姿态的Jacobian应该使用机体系向量，不是世界系向量

---

## 编译和测试

```bash
cd build
cmake ..
make
./SchurVIO
```

## 预期结果

**修复前**：
```
POS: GT = -3.05893 -3.95512 1, EST = -673017 1.62013e+06 -7.01975e+06
```

**修复后（预期）**：
```
POS: GT = -3.05893 -3.95512 1, EST = -3.1 -4.0 1.0
```

误差应该在**米级别**，不是百万级别！

---

## 如果仍有问题

### 检查特征点深度
```cpp
// 在 eskf/schur_vins.cpp:404 后添加
if (d_cj_c.z() <= 0) {
    std::cerr << "Negative depth: " << d_cj_c.z() << std::endl;
    continue;
}
```

### 监控协方差
```cpp
// 在 predict 后添加
std::cout << "Cov trace: " << cov_.trace() << std::endl;
```

### 调试输出
```cpp
// 在 updateState 后添加
std::cout << "dx norm: " << dx.norm() << std::endl;
```

---

## 参数微调（可选）

如果仍然不够稳定，可以微调观测噪声：

```cpp
// 更保守（观测噪声更大）
constexpr static TYPE uv_var = TYPE(0.0004);

// 更激进（观测噪声更小，需要精确标定）
constexpr static TYPE uv_var = TYPE(0.00005);
```

---

## 已创建的文档

1. `FIXES_SUMMARY.md` - 所有NED坐标系修正
2. `CAMERA_ORIENTATION_FIX.md` - 相机朝向修正
3. `DIVERGENCE_FIX.md` - 发散问题诊断（部分）
4. `DIVERGENCE_FIX_SUMMARY.md` - 发散问题完整分析

---

**核心要点**：观测噪声错了4000倍是主因，必须修复！
