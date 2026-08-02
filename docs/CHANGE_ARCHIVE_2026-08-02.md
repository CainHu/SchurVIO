# 视觉后验调度与参数化修改归档（2026-08-02）

## 归档结论

本次归档把近期的 Schur 视觉后验修改收敛为一套可编译切换、可重复测试、可生成报告的实现。生产默认保持：

- `USE_SCHUR` 路径；
- `MSCKF` 一次性轨迹调度；
- `WORLD_XYZ` 三自由度 Landmark 参数化；
- `Hpp/Hll` 默认使用 LDLT，并保留特征分解开关；
- FEJ 可观性约束开启，硬投影实验关闭；
- 重力和相机-IMU 外参默认不加入状态，`J_ext` 代码继续保留并由编译期分支屏蔽。

RD-VIO 调度和三种锚定 Landmark 参数化均作为实验选项保留，不替换上述默认配置。

## 代码拆分

原先集中在 `SchurVINS::updateVisual()` 内的新增逻辑已经按职责拆分：

| 模块 | 职责 |
|---|---|
| `eskf/visual_update_scheduler.h` | 五种视觉观测生命周期与滑窗规模的编译期开关 |
| `eskf/rdvio_scheduler.{h,cpp}` | R/N 判定、RR/NN/RN/NR 转移、关键帧提升和纯旋转子窗压缩计划 |
| `eskf/rdvio_constraints.{h,cpp}` | 无深度纯旋转约束和零平移约束对 `Hpp/gp` 的累加 |
| `eskf/landmark_parameterization.{h,cpp}` | 四种 3-DOF 参数化、锚帧位姿雅可比、局部正规方程回到世界 XYZ |
| `eskf/schur_vins.cpp` | 视觉更新流程编排、Schur 消元、状态后验和统计日志 |

拆分后，参数化和 RD-VIO 的数学实现不再与轨迹筛选、Schur 消元及地图后处理混在同一代码段中。主流程仍采用编译期调度选择，因此默认构建不会为未选择的调度引入运行时分派。

## 本次归档功能

### 观测调度

- 保留历史 `LEGACY`、SchurVINS 风格和 VINS-Mono 风格对照；
- 新增并默认采用 `MSCKF` 一次性轨迹消费，阻止同一像素观测被重复当作独立量测；
- 新增 RD-VIO 风格 R/N 分层调度、R 帧延迟三角化、旋转约束和 R 子窗压缩；
- 滑窗支持按时间序号删除任意 clone，并正确维护物理协方差块索引。

### Landmark 参数化

- `WORLD_XYZ`；
- `ANCHORED_XYZ`；
- `ANCHORED_INV_DEPTH`，即三自由度 `(alpha, beta, rho)`；
- `ANCHORED_LOG_DEPTH`，即三自由度 `(alpha, beta, log(z))`。

地图中的持久位置始终是世界系 XYZ。参数化只改变单次 Schur 线性化坐标，并完整加入锚帧位姿雅可比。实验表明三自由度逆深度显著改善 `Hll` 条件数，但短时轨迹精度没有确定提升，因此默认仍为世界 XYZ。

### 仿真与报告

- 报告支持按 tag 生成独立 HTML，避免不同调度实验互相覆盖；
- 调度和参数化摘要进入报告配置表；
- 新增 `rotation_translation` 场景，用于暴露纯旋转到平移切换时的退化；
- 轨迹三维图不再在窗口尺寸变化时整页重载，降低拖动/滚轮操作卡死风险；
- 提供调度及参数化批量回归脚本。

## 归档实验结果

### 参数化快速消融

条件：Circle-out、5 s、150 个点、MSCKF-Schur。

| 参数化 | 对齐 ATE / m | 1 s RPE / m | `Hll` 平均有效条件数 |
|---|---:|---:|---:|
| World XYZ | 0.241925 | 0.200313 | 3693.3 |
| Anchored XYZ | 0.242036 | 0.200478 | 3693.1 |
| Anchored inverse-depth 3D | 0.242427 | 0.200753 | **47.9** |
| Anchored log-depth 3D | 0.242573 | 0.200859 | 1017.8 |

### 调度快速消融

8 s、250 个点时，RD-VIO 实验模式在 Circle-out、Circle-in 和 Helix-3D 上的对齐 ATE 低于本次 MSCKF 基线，但在 `rotation_translation` 上出现约 3.12 m 的对齐 ATE 和明显偏大的 NEES。该结果说明当前移植的 R/N 判定与约束还不能替代完整 RD-VIO 前端、预积分拼接和分层优化器。因此 RD-VIO 继续标记为实验模式，生产默认不变。

### 重构后等价性冒烟测试

条件：Circle-out、3 s、100 个点、默认 MSCKF + WORLD_XYZ。

```text
RMSE position = 0.0475 m
RMSE velocity = 0.0625 m/s
RMSE attitude = 0.0080 rad
observations new/reused/blocked = 417/0/0
tracks consumed/dropped = 76/34
```

该结果与拆分前的同配置记录一致；一次性观测不变量继续成立。

## 复现实验

```powershell
# 五种调度快速回归，脚本结束后恢复 MSCKF
powershell -ExecutionPolicy Bypass -File tools/run_scheduler_analysis.ps1 -Quick

# 四种三自由度参数化快速回归，脚本结束后恢复 WORLD_XYZ
powershell -ExecutionPolicy Bypass -File tools/run_landmark_parameterization_analysis.ps1 -Quick

# 生成默认四场景自包含报告 out/report.html
cmake --build cmake-build-release --target VinsAnalysis VinsReport
powershell -ExecutionPolicy Bypass -File tools/run_multi_scenario_analysis.ps1 `
  -Duration 30 -Features 600 -BuildDirectory cmake-build-release
```

详细数学原理和消融边界分别见：

- [视觉更新调度](VISUAL_UPDATE_SCHEDULING.md)
- [RD-VIO 调度](RDVIO_SCHEDULING.md)
- [Landmark 参数化](LANDMARK_PARAMETERIZATION.md)
- [仿真场景](SIMULATION_SCENARIOS.md)

## 版本库边界

本归档提交源码、CMake 配置、实验脚本和 Markdown 结论。`out/` 中的 CSV/HTML 属于可再生实验产物，不进入提交；IDE 配置、构建目录和 `tmp/` 下的论文/临时仓库同样不归档到 Git。归档分支为 `codex/covariance-stability`，本文所在提交即本轮归档点。
