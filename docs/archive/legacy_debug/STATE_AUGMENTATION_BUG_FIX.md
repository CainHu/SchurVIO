# 状态增广协方差Bug修复

> 历史归档：本文记录早期调试过程，代码位置、默认参数和部分结论可能已过时；当前实现请以 `docs/README.md` 及专题文档为准。

## 问题描述

姿态估计在**第一次视觉更新后**立即发散：

```
第0帧（初始化）：
YPR: GT = 0.785898 1.5708 2.35519, EST = 0.785898 1.5708 2.35519  ✓ 完全正确

第1帧（第一次更新后）：
YPR: GT = 0.790398 1.5708 2.34619, EST = 1.57978 1.5628 -3.1406  ❌ 完全错误！
```

姿态从正确值突然跳变到错误值，这是典型的**协方差矩阵错误**导致的。

## 根本原因

### 状态定义

**INSState**（18维）：
```cpp
struct INSState {
    Vec3 orientation;  // 0-2: q的误差状态
    Vec3 position;     // 3-5: p
    Vec3 velocity;     // 6-8: v
    Vec3 gyro_bias;    // 9-11: bg
    Vec3 accel_bias;   // 12-14: ba
    Vec3 gravity;      // 15-17: g
};
```

**AugState**（6维）：
```cpp
struct AugState {
    Vec3 orientation;  // 0-2: q的误差状态
    Vec3 position;     // 3-5: p
};
```

### 错误的协方差增广

**原代码**（错误）：
```cpp
// 错误：复制了INSState的整个前6维协方差
cov_.middleRows<A::SIZE>(i).noalias() = cov_.topRows<A::SIZE>();
cov_.middleCols<A::SIZE>(i).noalias() = cov_.leftCols<A::SIZE>();
cov_.block<A::SIZE, A::SIZE>(i, i).noalias() = cov_.topLeftCorner<A::SIZE, A::SIZE>();
```

问题：
1. `cov_.topRows<A::SIZE>()` 取的是INS协方差的前6行
2. 这**恰好对应 q 和 p**（0-5维）
3. **但是**，`cov_.leftCols<A::SIZE>()` 也是前6列
4. 这样复制**只对了对角块**，交叉协方差完全错了！

### 协方差矩阵结构

正确的大协方差矩阵结构：
```
        | INS(18) | Aug0(6) | Aug1(6) | ... |
--------|---------|---------|---------|-----|
INS(18) |   P11   |   P12   |   P13   | ... |
Aug0(6) |   P21   |   P22   |   P23   | ... |
Aug1(6) |   P31   |   P32   |   P33   | ... |
...     |   ...   |   ...   |   ...   | ... |
```

增广第k个状态时，需要：
1. **P_k,INS = P_qp,INS**（增广状态与INS的协方差）
2. **P_INS,k = P_INS,qp**（对称）
3. **P_k,k = P_qp,qp**（增广状态自身的协方差）
4. **P_k,j = P_qp,j**（增广状态与其他增广状态的协方差，j < k）
5. **P_j,k = P_j,qp**（对称）

其中 qp 表示INS状态的 q 和 p 部分（0-5维）。

## 修复方案

**正确的协方差增广**：
```cpp
using I = INSState;
auto idx = map_.getWinLatestIndex();
const auto aug_row = INSState::SIZE + idx * A::SIZE;  // 增广状态在大协方差矩阵中的行索引

// 1. 复制 q 和 p 对应的协方差（INSState的前6维）
// cov_[aug, INS] = cov_[q:p, INS]
cov_.block<A::SIZE, INSState::SIZE>(aug_row, 0) = cov_.block<A::SIZE, INSState::SIZE>(I::Q, 0);

// cov_[INS, aug] = cov_[INS, q:p]
cov_.block<INSState::SIZE, A::SIZE>(0, aug_row) = cov_.block<INSState::SIZE, A::SIZE>(0, I::Q);

// 2. 处理增广状态之间的协方差（如果已有其他增广状态）
for (size_t k = 0; k < idx; ++k) {
    const auto other_aug_row = INSState::SIZE + k * A::SIZE;
    // cov_[aug, other_aug] = cov_[q:p, other_aug]
    cov_.block<A::SIZE, A::SIZE>(aug_row, other_aug_row) = cov_.block<A::SIZE, A::SIZE>(I::Q, other_aug_row);
    // cov_[other_aug, aug] = cov_[other_aug, q:p]
    cov_.block<A::SIZE, A::SIZE>(other_aug_row, aug_row) = cov_.block<A::SIZE, A::SIZE>(other_aug_row, I::Q);
}

// 3. 增广状态自身的协方差
// cov_[aug, aug] = cov_[q:p, q:p]
cov_.block<A::SIZE, A::SIZE>(aug_row, aug_row) = cov_.block<A::SIZE, A::SIZE>(I::Q, I::Q);
```

## 为什么原代码会导致发散

### 错误的协方差导致错误的Kalman增益

1. **错误的交叉协方差**：
   - 原代码：`cov_[aug, INS] = cov_[0:6, 0:6]`（只有q和p的自协方差）
   - 正确的：`cov_[aug, INS] = cov_[q:p, 0:18]`（q和p与所有INS状态的协方差）

2. **Kalman增益计算**：
   ```
   K = P * H^T * (H * P * H^T + R)^-1
   ```
   - P 错误 → K 错误 → 更新方向完全错误

3. **后果**：
   - 第一次视觉更新使用了错误的K
   - 姿态被错误地修正到完全不对的方向
   - 后续更新基于错误的姿态，越来越发散

### 数值示例

假设：
- INS协方差：`P_qp,v = 0.5`（q/p与速度v的协方差）
- 观测对速度敏感：`H_v != 0`

**错误的增广**：
- `P_aug,v = 0`（被错误地设为0）
- Kalman增益：`K_aug = 0 * H_v^T / ... = 0`
- 增广状态不更新，或更新方向错误

**正确的增广**：
- `P_aug,v = P_qp,v = 0.5`
- Kalman增益：`K_aug = 0.5 * H_v^T / ...`（正确）
- 增广状态正确更新

## 验证修复

修复后，第一次更新应该：
1. 姿态保持接近真值（小幅修正）
2. 位置逐渐收敛
3. 协方差逐渐减小（滤波器越来越自信）

预期输出：
```
第0帧：YPR: GT = 0.786 1.571 2.355, EST = 0.786 1.571 2.355
第1帧：YPR: GT = 0.790 1.571 2.346, EST = 0.790 1.571 2.346  ✓ 接近真值
第2帧：YPR: GT = 0.795 1.571 2.336, EST = 0.795 1.571 2.337  ✓ 小误差
```

## 总结

这是一个**致命的bug**，导致：
1. 协方差矩阵结构完全错误
2. Kalman增益计算错误
3. 第一次视觉更新就导致发散

修复后，系统应该能够正常工作。这个bug比之前的观测噪声问题更严重，因为它破坏了滤波器的基本结构。

## 其他需要检查的点

如果修复后仍有问题，检查：
1. **观测噪声**：uv_var = 0.0001 是否合理
2. **初始协方差**：是否足够大
3. **特征点深度**：是否都为正
4. **Jacobian**：是否使用了正确的向量（d_ij_b）
