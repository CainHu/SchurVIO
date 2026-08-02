# 最终修复清单

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## ⚠️ 致命Bug（必须修复）

### 1. 状态增广协方差错误 ✓ 已修复
**文件**：`eskf/schur_vins.cpp` 第248-278行

**症状**：第一次视觉更新后姿态立即发散
```
第0帧：YPR EST = 0.786 1.571 2.355  ✓ 正确
第1帧：YPR EST = 1.580 1.563 -3.141 ❌ 完全错误
```

**原因**：增广状态时复制了错误的协方差块

**修复**：正确复制 q 和 p 对应的协方差（INSState的前6维），包括与其他状态的交叉协方差

---

### 2. 观测噪声方差错误 ✓ 已修复
**文件**：`eskf/schur_vins.h` 第104行

**原值**：`uv_var = 400`（标准差20）
**修正**：`uv_var = 0.0001`（标准差0.01）

**原因**：仿真器输出归一化坐标，不是像素坐标

---

### 3. Jacobian使用错误的向量 ✓ 已修复
**文件**：`eskf/schur_vins.cpp` 第415行（两处）

**原值**：`J_lmk * hat(d_ij_w)`（世界系向量）
**修正**：`J_lmk * hat(d_ij_b)`（机体系向量）

**原因**：ESKF姿态误差定义在机体系

---

## 🔧 重要修复

### 4. 初始协方差过小 ✓ 已修复
**文件**：`common.h` 第76-88行

**修正**：移除所有噪声参数的 `* 1e-1`

---

### 5. 观测模型投影 ✓ 已修复
**文件**：`eskf/schur_vins.cpp` 第402-403行

**修正**：正确的外参处理顺序
```cpp
const auto d_ij_b = Rwi.transpose() * d_ij_w;  // 世界→机体
const auto d_cj_c = Ric.transpose() * (d_ij_b - ext_.t_ic);  // 机体→相机
```

---

### 6. 相机朝向 ✓ 已修复
**文件**：`vio_frontend_simulator.cpp` 和 `vio_frontend_simulator1.cpp`

**修正**：机体z轴指向径向外侧（相机朝外看特征点）

---

### 7. 协方差对称化 ✓ 已修复
**文件**：`eskf/schur_vins.cpp` 第210行

**添加**：`cov = 0.5 * (cov + cov.transpose())`

---

## 📋 修复优先级

### 🔴 P0 - 必须立即修复（否则完全无法工作）
1. ✅ 状态增广协方差错误
2. ✅ 观测噪声方差错误

### 🟡 P1 - 重要修复（影响精度和稳定性）
3. ✅ Jacobian向量错误
4. ✅ 初始协方差过小
5. ✅ 观测模型投影

### 🟢 P2 - 优化（改善性能）
6. ✅ 相机朝向
7. ✅ 协方差对称化

---

## 🎯 预期效果

### 修复前
```
第1帧：YPR: GT = 0.790 1.571 2.346, EST = 1.580 1.563 -3.141
第1帧：POS: GT = 5.000 0.050 1.000, EST = 5.000 0.040 1.000
姿态误差：~90度（完全错误）
位置误差：~1cm（IMU预测还算准确）

第20帧：位置发散到数米甚至数十米
```

### 修复后（预期）
```
第1帧：YPR: GT = 0.790 1.571 2.346, EST = 0.790 1.571 2.346
第1帧：POS: GT = 5.000 0.050 1.000, EST = 5.000 0.050 1.000
姿态误差：< 1度
位置误差：< 1cm

第20帧：
姿态误差：< 5度
位置误差：< 10cm
速度误差：< 0.1 m/s
```

---

## 📝 测试步骤

1. **重新编译**
```bash
cd build
cmake ..
make
```

2. **运行仿真**
```bash
./SchurVIO
```

3. **观察关键指标**
- 第1帧更新后，姿态应该接近真值（不是跳变到1.58）
- 位置误差应该保持在厘米级
- 速度误差应该保持在m/s级
- 偏置估计应该缓慢收敛到零附近

4. **如果仍有问题**
   - 添加调试输出（见下节）
   - 检查特征点深度是否为正
   - 检查协方差矩阵的迹

---

## 🐛 调试建议

### 添加调试输出

在 `updateState` 后：
```cpp
std::cout << "dx norm: " << dx.norm() << std::endl;
if (dx.norm() > 1.0) {
    std::cerr << "Warning: large update!" << std::endl;
    std::cout << "dx = " << dx.transpose() << std::endl;
}
```

在 `updateVisual` 中：
```cpp
if (d_cj_c.z() <= 0) {
    std::cerr << "Negative depth: " << d_cj_c.z() << " for landmark " << id << std::endl;
    continue;  // 跳过
}
```

监控协方差：
```cpp
std::cout << "Cov trace: " << cov_.trace() << std::endl;
```

---

## 📚 相关文档

1. `STATE_AUGMENTATION_BUG_FIX.md` - 状态增广bug详解
2. `DIVERGENCE_FIX_SUMMARY.md` - 发散问题完整分析
3. `FIXES_SUMMARY.md` - NED坐标系修正
4. `CAMERA_ORIENTATION_FIX.md` - 相机朝向修正
5. `QUICK_FIX_REFERENCE.md` - 快速参考

---

## ✅ 所有修改的文件

1. ✅ `eskf/schur_vins.h` - 观测噪声
2. ✅ `eskf/schur_vins.cpp` - 状态增广、Jacobian、投影、对称化
3. ✅ `common.h` - 初始协方差
4. ✅ `vio_frontend_simulator.cpp` - 相机朝向
5. ✅ `vio_frontend_simulator1.cpp` - 相机朝向

---

## 🎓 技术要点

### ESKF状态增广的正确方法

增广状态 x_aug = [q, p] 是从INS状态 x_ins = [q, p, v, bg, ba, g] 中提取的。

**协方差增广**必须保持：
```
Cov(x_aug, x_ins) = Cov([q,p], [q,p,v,bg,ba,g])
                  = [Cov(q,q)  Cov(q,p)  Cov(q,v)  ...]
                    [Cov(p,q)  Cov(p,p)  Cov(p,v)  ...]
```

**关键**：不仅要复制对角块 Cov(q,q) 和 Cov(p,p)，还要复制交叉协方差 Cov(q,v)、Cov(p,bg) 等。

### MSCKF/SchurVINS的核心

1. **状态向量**：INS状态 + 多个相机位姿（滑窗）
2. **观测方程**：特征点重投影误差
3. **QR分解**：消除特征点状态，降低维度
4. **Schur补**：边缘化特征点，只更新位姿

---

**核心要点**：状态增广的协方差复制是整个算法的基础，错了就全盘皆输！
