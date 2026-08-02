# 当前状态和调试策略

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 已完成的修复（7个）

1. ✅ 状态增广协方差错误（致命）
2. ✅ 观测噪声方差错误（致命）
3. ✅ Jacobian向量错误
4. ✅ 初始协方差过小
5. ✅ 观测模型投影顺序
6. ✅ 相机朝向
7. ✅ 协方差对称化

## 已添加的调试输出

### 1. 状态更新调试（eskf/schur_vins.cpp:912）
```cpp
std::cout << "=== State Update ===" << std::endl;
std::cout << "dx norm: " << dx.norm() << std::endl;
std::cout << "dx[Q]: " << dx.segment<3>(I::Q).transpose() << std::endl;
std::cout << "dx[P]: " << dx.segment<3>(I::P).transpose() << std::endl;
std::cout << "dx[V]: " << dx.segment<3>(I::V).transpose() << std::endl;
```

### 2. 深度检查（eskf/schur_vins.cpp:403）
```cpp
if (d_cj_c.z() <= 0.1) {
    std::cerr << "Warning: landmark has small/negative depth: " << d_cj_c.z() << std::endl;
    continue;  // 跳过小/负深度的点
}
```

### 3. 数值稳定性检查（eskf/schur_vins.cpp:520）
```cpp
if (var < 1e-10) {
    std::cerr << "Warning: innovation variance too small: " << var << std::endl;
    continue;
}
```

### 4. 修正列置换（eskf/schur_vins.cpp:496-502）
```cpp
// QR分解后正确恢复列顺序
const auto &perm = qr.colsPermutation();
MatXX H_red = MatXX::Zero(J_STATE.cols(), J_STATE.cols());
for (int i = 0; i < J_STATE.cols(); ++i) {
    H_red.col(perm.indices()(i)) = R_red.col(i);
}
```

## 需要验证的问题

### 可能的问题1：状态更新幅度过大

如果看到输出：
```
dx norm: 10.5  # 非常大的更新
dx[Q]: [2.1, -1.8, 3.5]  # 姿态修正过大（应该 < 0.1 rad）
```

**可能原因**：
- 观测噪声仍然不合理
- 协方差矩阵不正确
- Jacobian计算错误

### 可能的问题2：负深度或极小深度

如果看到很多：
```
Warning: landmark has small/negative depth: -2.5
Warning: landmark has small/negative depth: 0.05
```

**可能原因**：
- 相机朝向还是不对
- 外参设置错误
- 特征点位置生成错误

### 可能的问题3：观测残差异常

如果残差很大：
```
Landmark 123: depth=5.2, err=(0.8, 1.2)  # err应该 < 0.05
```

**可能原因**：
- 观测模型错误
- 坐标系转换错误
- 相机内参不匹配

## 调试步骤

### 第1步：编译运行
```bash
cd build
cmake ..
make
./SchurVIO > output.log 2>&1
```

### 第2步：检查输出

**关键指标**：
1. **第一次更新的dx norm**：应该 < 1.0
2. **深度警告数量**：应该很少或没有
3. **innovation variance**：应该 > 1e-6

**正常输出示例**：
```
=== State Update ===
dx norm: 0.05
dx[Q]: [0.001, -0.002, 0.003]  # 小的姿态修正
dx[P]: [0.01, -0.02, 0.005]    # 厘米级位置修正
dx[V]: [0.001, 0.002, -0.001]  # 小的速度修正

YPR: GT = 0.790 1.571 2.346, EST = 0.791 1.570 2.347  # 接近
POS: GT = 5.000 0.050 1.000, EST = 5.002 0.048 1.001  # 接近
```

**异常输出示例**：
```
=== State Update ===
dx norm: 5.2  # 太大！
dx[Q]: [1.5, -2.1, 0.8]  # 巨大的姿态跳变
dx[P]: [2.5, -3.1, 1.2]  # 米级的位置跳变

YPR: GT = 0.790 1.571 2.346, EST = 2.150 0.823 -1.456  # 完全错误
```

### 第3步：根据输出诊断

#### 情况A：dx norm很大（> 1.0）

**可能问题**：观测噪声太小，滤波器过度信任错误的观测

**解决方案**：
```cpp
// eskf/schur_vins.h
constexpr static TYPE uv_var = TYPE(0.001);  // 增大到0.001
```

#### 情况B：大量负深度警告

**可能问题**：相机朝向或外参错误

**检查**：
```cpp
// 在 generateCameraData 中添加
std::cout << "Camera pos: " << state.p.transpose() << std::endl;
std::cout << "Camera ori: " << state.q.coeffs().transpose() << std::endl;
std::cout << "Feature pos: " << P_w.transpose() << std::endl;
std::cout << "Depth in camera: " << P_c.z() << std::endl;
```

#### 情况C：姿态估计正确，但位置发散

**可能问题**：位置与其他状态的协方差不正确

**检查**：
```cpp
// 在 pushFrame 后添加
std::cout << "Cov(p,p): " << cov_.block<3,3>(I::P, I::P).diagonal().transpose() << std::endl;
std::cout << "Cov(p,v): " << cov_.block<3,3>(I::P, I::V).diagonal().transpose() << std::endl;
```

## 如果问题仍未解决

### 最后的排查点

1. **外参是否正确**
   - 当前：q_ic = Identity, t_ic = Zero
   - 如果相机和IMU不重合，需要设置正确的外参

2. **观测数据格式**
   - 检查 `obs->un_pt` 是否真的是归一化坐标
   - 验证：un_pt.x() 和 un_pt.y() 应该在 [-1, 1] 范围内

3. **特征点三角化**
   - 当前直接使用真值：`lmk->position = lmk_map.at(id)`
   - 在实际中应该通过三角化初始化

4. **时间同步**
   - IMU和相机时间戳是否匹配
   - 预测步骤的dt是否正确

## 参数微调建议

如果系统基本工作但精度不够，可以微调：

```cpp
// 观测噪声（在 eskf/schur_vins.h）
constexpr static TYPE uv_var = TYPE(0.0001);  // 当前值
// 如果发散 → 增大到 0.001
// 如果收敛太慢 → 减小到 0.00005

// 初始协方差（在 common.h）
constexpr static double STB_Q_INIT = 3e-2;  // 当前值
// 如果初始误差大 → 增大到 1e-1
// 如果过于保守 → 减小到 1e-2

// 过程噪声（在 common.h）
constexpr static double STB_Q_PROC = 4e-2;  // 当前值
// 如果漂移严重 → 增大到 1e-1
// 如果震荡 → 减小到 1e-2
```

## 总结

当前已完成所有已知的关键修复，并添加了调试输出。下一步：

1. **编译运行**
2. **查看调试输出**
3. **根据输出诊断问题**
4. **反馈结果以便进一步分析**

如果输出日志太长，可以只发送前50-100行，包含第一次状态更新的部分。
