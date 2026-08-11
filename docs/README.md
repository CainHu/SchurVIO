# 视觉更新优化总览

本目录记录 SchurVIO 的源码结构、数学推导、视觉后验策略、精度消融与历史问题归档。

## 历史求解器优化结果

下表记录早期同一数据流中的求解器交错 A/B：Schur 优化先从 203.5 s 降到 11.9 s
（约 17 倍），Hpp LDLT 再降到 9.7 s（累计约 21 倍），精度全程基本不变。后续加入真实
三角化、多场景、一次性调度和 Hybrid 持久点后，工作量已经变化，不能把这些绝对秒数
当成当前 100 秒报告的耗时基线。

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
| [SOURCE_LAYOUT.md](SOURCE_LAYOUT.md) | 当前 `schur_vins_*.cpp` 拆分、调用流程、数学文档映射和维护约定 |
| [MATHEMATICAL_PIPELINE.md](MATHEMATICAL_PIPELINE.md) | 从 IMU 传播、clone 增广、三角化到 Schur/Joseph 后验的数学总流程 |
| [ESKF_STATE_PROPAGATION_AND_AUGMENTATION.md](ESKF_STATE_PROPAGATION_AND_AUGMENTATION.md) | **ESKF 专题**：坐标系、左乘误差、IMU 转移、联合 cross covariance、clone 增广、Joseph 与 reset 边界 |
| [INITIALIZATION_AND_TIME_SYNCHRONIZATION.md](INITIALIZATION_AND_TIME_SYNCHRONIZATION.md) | **启动专题**：初始均值/协方差、仿真 oracle Q/P/V 边界、首条 IMU 与相机时刻同步、真实设备初始化缺口 |
| [CLONE_REMOVAL_AND_SLOT_REUSE.md](CLONE_REMOVAL_AND_SLOT_REUSE.md) | **删帧专题**：协方差主子块、信息形式 Schur、固定物理槽正确性、完整行列覆盖和紧凑化边界 |
| [VISUAL_RESIDUAL_NOISE_MODEL.md](VISUAL_RESIDUAL_NOISE_MODEL.md) | **视觉统计专题**：归一化方差、Huber/硬门限、一次性 MSCKF、持久点与历史 `uv_var` 的不同语义 |
| [EVALUATION_METRICS_AND_GAUGE_ALIGNMENT.md](EVALUATION_METRICS_AND_GAUGE_ALIGNMENT.md) | **评估专题**：左乘误差、ATE/RPE、四自由度 gauge 对齐、NEES/NIS 的真实归一化与协方差谱检查 |
| [REIMPLEMENTATION_GUIDE_137BFEA_TO_HEAD.md](REIMPLEMENTATION_GUIDE_137BFEA_TO_HEAD.md) | **重实现路线**：从 `137bfea` 按问题、数学依赖、流程图和验收逐阶段重建当前算法 |
| [OPT_QR_PATH.md](OPT_QR_PATH.md) | QR 路径 203.5 s → 61.8 s |
| [OPT_SCHUR_PATH.md](OPT_SCHUR_PATH.md) | Schur 路径 18.3 s → 11.9 s |
| [OPT_LDLT.md](OPT_LDLT.md) | `Hpp` 分解改用 LDLT，11.9 s → 9.7 s |
| [QR_VS_SCHUR.md](QR_VS_SCHUR.md) | 理论：为什么 QR 比 Schur 慢 5 倍；两条失败的优化尝试 |
| [VISUAL_UPDATE_SCHEDULING.md](VISUAL_UPDATE_SCHEDULING.md) | Legacy / SchurVINS / MSCKF / VINS-Mono 调度、一次性观测生命周期与宏切换 |
| [KEYFRAME_REDUNDANCY_POLICY.md](KEYFRAME_REDUNDANCY_POLICY.md) | **实验性冗余删帧专题**：旋转补偿视差、未成熟轨迹保护、删除评分、状态框图与消融字段 |
| [HYBRID_MSCKF.md](HYBRID_MSCKF.md) | **默认混合 MSCKF 专题**：状态扩维、持久点四层筛选、Schur 延迟初始化、联合 EKF、低视差延迟与无深度旋转约束 |
| [DEPTH_FREE_ROTATION_CONSTRAINT.md](DEPTH_FREE_ROTATION_CONSTRAINT.md) | **纯旋转专题**：深度自然约消、小平移偏差、球面切平面残差、FEJ 雅可比、噪声与 EKF 接入 |
| [RDVIO_SCHEDULING.md](RDVIO_SCHEDULING.md) | RD-VIO 的 RR/NN/RN/NR、延迟三角化、无深度旋转因子、R 子窗压缩与严格结果 |
| [LANDMARK_PARAMETERIZATION.md](LANDMARK_PARAMETERIZATION.md) | World/锚定 XYZ、3-DOF 逆深度与 log-depth 的完整雅可比和数值消融 |
| [ANALYSIS_REPORT.md](ANALYSIS_REPORT.md) | **精度分析**：视觉后验的修正作用、噪声敏感度、协方差问题（配套 `out/report.html` 交互图表） |
| [TRIANGULATION.md](TRIANGULATION.md) | 多视图三角化、失败门限、初始 landmark 协方差和真值离线评估 |
| [SIMULATION_SCENARIOS.md](SIMULATION_SCENARIOS.md) | Circle-out / Circle-in / Helix-3D / Stop-go / Rotation-translation 场景与统一噪声模型 |
| [SIM_ROTATION_TRANSLATION.md](SIM_ROTATION_TRANSLATION.md) | **调度压力场景**：R/N 解析分段、RR/RN/NN/NR 覆盖、持续姿态和三维环带特征 |
| [ABLATION_STUDY.md](ABLATION_STUDY.md) | 四场景严格消融：三角化、landmark 修正、IMU 白噪声离散化与偏置随机游走 |
| [HPP_NULLSPACE.md](HPP_NULLSPACE.md) | 理论：当前零空间如何随活跃 clone 数变化、历史 31 的来源、LDLT 主元与空槽压缩机会 |
| [OBSERVABILITY_CONSTRAINT.md](OBSERVABILITY_CONSTRAINT.md) | FEJ 可观性约束：四维 VIO gauge、Schur 实现、硬投影反例与四场景 A/B |
| [LANDMARK_UPDATE_STRATEGIES.md](LANDMARK_UPDATE_STRATEGIES.md) | 固定点、重三角化、Schur 回代与旧独立 EKF 的数学边界和严格对比 |
| [HLL_STRUCTURE.md](HLL_STRUCTURE.md) | 理论：`Hll` 的弱深度方向；Schur 伪逆为何保留特征分解、历史独立点 LDLT 的边界 |
| [CONSISTENT_SUBSPACE_AND_LANDMARK_COVARIANCE.md](CONSISTENT_SUBSPACE_AND_LANDMARK_COVARIANCE.md) | `Hll/Hpp/gp` 同域投影、Landmark 协方差与影子地图实验 |
| [SHADOW_LANDMARKS.md](SHADOW_LANDMARKS.md) | 独立影子地图与影子候选的概念边界、历史实验和回归结果；混合主流程以 `HYBRID_MSCKF.md` 为准 |
| [archive/legacy_debug/README.md](archive/legacy_debug/README.md) | 早期坐标系、发散和状态增广调试文档；仅用于历史追溯 |

各文档都记录了**失败的尝试和被数据推翻的判断**，不只记成功的部分。

推荐按以下顺序阅读数学部分：

```mermaid
flowchart LR
    A["初始化、时间同步"] --> B["坐标与 ESKF 传播"]
    B --> C["clone 增广、删帧与槽位"]
    C --> D["数学总流程"]
    D --> E["三角化"]
    E --> F["Landmark 参数化"]
    F --> G["Schur 与 QR 等价性"]
    G --> H["Hll/Hpp 零空间"]
    H --> I["FEJ 与一致有效子空间"]
    I --> J["无深度纯旋转约束"]
    J --> K["视觉调度与 RD-VIO"]
    K --> L["评估定义、消融与报告"]
```

## 路径切换

`eskf/schur_vins_visual.cpp` 顶部：

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
| `USE_LDLT_FOR_HLL` | `eskf/schur_vins.h` | `true` | 仅控制旧 Independent EKF 消融；默认 Schur 的 3×3 `Hll` 伪逆固定用特征分解判秩 |
| `SCHUR_VIO_VISUAL_SCHEDULER` | `CMakeLists.txt` / `eskf/visual_update_scheduler.h` | `MSCKF` | 视觉调度：Legacy、SchurVINS、MSCKF 或 VINS-Mono 风格 |
| `SCHUR_VIO_FRAME_POLICY` | `CMakeLists.txt` / `eskf/frame_selection_policy.h` | `AUTO` | 帧选择：默认 MSCKF 下解析为 `KEYFRAME_ONLY`；可显式选择实验性 `KEYFRAME_REDUNDANCY` |
| `SCHUR_VIO_FRAME_WINDOW_SIZE` | `CMakeLists.txt` | `0` | `0` 使用策略默认预算，非零时固定 clone 数用于公平消融 |

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

## 修改归档

- [2026-08-02：视觉后验调度、Landmark 参数化与代码拆分归档](CHANGE_ARCHIVE_2026-08-02.md)
