# 视觉更新优化总览

本目录记录 SchurVIO 视觉后验更新的性能优化过程。

## 整体结果

从最初的 203.5 s 到现在的 11.9 s，**总体 17 倍**，精度全程逐位不变。

| 阶段 | 路径 | 耗时 | 说明 |
|---|---|---|---|
| 起点 | QR | 203.5 s | |
| 方案2（观测缓冲区） | QR | — | 非关键帧也做后验更新 |
| 方案4（Zero-copy） | QR | — | 非关键帧观测不落地成中间结构 |
| QR 大分解优化 | QR | **61.8 s** | `colPivHouseholderQr` → `householderQr` |
| 切换到 Schur 路径 | Schur | **18.3 s** | 原样就比优化后的 QR 快 3.4× |
| Schur 路径优化 | Schur | **11.9 s** | `Hll` 块对角 + 序贯循环对称性 + 不变量提升 |
| `Hpp` 分解改 LDLT | Schur | **9.7 s** | 分解本身 2.92 s → 0.39 s |

> 方案2/方案4 单独看对总耗时无可测量影响（非关键帧 refine 仅占 0.05 s），
> 它们解决的是**功能问题**（非关键帧此前被直接丢弃、不做后验更新），不是性能问题。

## 文档

| 文档 | 内容 |
|---|---|
| [OPT_QR_PATH.md](OPT_QR_PATH.md) | QR 路径 203.5 s → 61.8 s |
| [OPT_SCHUR_PATH.md](OPT_SCHUR_PATH.md) | Schur 路径 18.3 s → 11.9 s |
| [OPT_LDLT.md](OPT_LDLT.md) | `Hpp` 分解改用 LDLT，11.9 s → 9.7 s |
| [QR_VS_SCHUR.md](QR_VS_SCHUR.md) | 理论：为什么 QR 比 Schur 慢 5 倍；两条失败的优化尝试 |
| [ANALYSIS_REPORT.md](ANALYSIS_REPORT.md) | **精度分析**：视觉后验的修正作用、噪声敏感度、协方差问题（配套 `out/report.html` 交互图表） |
| [TRIANGULATION.md](TRIANGULATION.md) | 多视图三角化、失败门限、初始 landmark 协方差和真值离线评估 |
| [SIMULATION_SCENARIOS.md](SIMULATION_SCENARIOS.md) | Circle-out / Circle-in / Helix-3D / Stop-go 场景与统一噪声模型 |
| [ABLATION_STUDY.md](ABLATION_STUDY.md) | 四场景严格消融：三角化、landmark 修正、IMU 白噪声离散化与偏置随机游走 |
| [HPP_NULLSPACE.md](HPP_NULLSPACE.md) | 理论：为什么 `Hpp` 恒有 ~31 维零空间；跳过策略是否正确 |
| [OBSERVABILITY_CONSTRAINT.md](OBSERVABILITY_CONSTRAINT.md) | FEJ 可观性约束：四维 VIO gauge、Schur 实现、硬投影反例与四场景 A/B |
| [LANDMARK_UPDATE_STRATEGIES.md](LANDMARK_UPDATE_STRATEGIES.md) | 固定点、重三角化、Schur 回代与旧独立 EKF 的数学边界和严格对比 |
| [HLL_STRUCTURE.md](HLL_STRUCTURE.md) | 理论：`Hll` 的零特征值 = 深度方向；`Hll` 换 LDLT |

各文档都记录了**失败的尝试和被数据推翻的判断**，不只记成功的部分。

## 路径切换

`eskf/schur_vins.cpp` 顶部：

```cpp
//#define USE_QR
#define USE_SCHUR      // 当前默认
```

两条路径精度基本一致（`POS` 相差约 4e-5），Schur 路径快 5 倍以上。

## 编译期开关

| 开关 | 位置 | 默认 | 说明 |
|---|---|---|---|
| `INSState::ESTIMATE_GRAVITY` | `common.h` | `false` | 仿真重力已知时固定；真实设备可重新开启 |
| `ExtState::ESTIMATE_EXTRINSIC` | `common.h` | `false` | 是否估计相机-IMU 外参 |
| `USE_LDLT_FOR_HPP` | `eskf/schur_vins.h` | `true` | `Hpp` 分解：`true`=LDLT，`false`=特征分解 |
| `USE_LDLT_FOR_HLL` | `eskf/schur_vins.h` | `true` | `Hll` 分解，同上（性能上两者无差别） |

`ESTIMATE_EXTRINSIC` 关闭时，外参雅可比 `J_ext` / `J_EXT` 的代码通过 `if constexpr`
屏蔽——不参与运行，但始终参与语法和类型检查，不会腐烂。

## 两条通用教训

**1. 先测量，再优化。** 两次凭直觉的判断都被数据推翻：

- QR 路径：预估「增广矩阵 + 去零行」有 5–10 % 收益 → 实测**完全无差别**
- Schur 路径：预估 `Hpl` 稀疏化能降一个数量级 → 实测**反而慢 42 %**
  （估计 mean K ≈ 5，实测 14.8，估错 3 倍）

**2. 这台机器的单次计时不可信。** 同一个二进制可在 115 s ~ 204 s 间浮动。
所有结论都必须基于**交错 A/B**：

```bash
for i in 1 2 3; do
  echo -n "base: "; /tmp/base.exe 2>&1 | grep "^t_cost"
  echo -n "opt:  "; /tmp/opt.exe  2>&1 | grep "^t_cost"
done
```

期间还发生过一次事故：`git checkout` 在编译**之前**执行，导致「基线」二进制
实际走的是另一条路径。发现方式是 `big QR = 45.48 s` 不该出现在 Schur 路径里。

## 内置计时

程序结束时打印分阶段耗时，用于定位瓶颈：

```
--- breakdown ---        (QR 路径)
per-lmk small QR = ...
big QR (J_STATE) = ...
seq state update = ...
lmk pos update   = ...

--- schur path ---       (Schur 路径)
build H          = ...
schur complement = ...
eig+seq state    = ...
eig+upd lmk      = ...
  of which: Hpp eig = ...
```
