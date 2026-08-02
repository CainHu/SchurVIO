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
| `eskf/schur_vins.cpp` | 外层相机/IMU 数据流、误差状态注入和名义状态设置 |
| `eskf/schur_vins_imu.cpp` | IMU 名义状态积分、误差转移和协方差传播 |
| `eskf/schur_vins_triangulation.cpp` | 多视图三角化、非线性精化和初始点协方差 |
| `eskf/schur_vins_visual.cpp` | clone 管理、轨迹调度、Schur 消元、状态后验和统计日志 |
| `eskf/schur_vins_shadow.cpp` | 不反馈导航状态的影子 Landmark 地图后处理 |

拆分后，IMU、三角化、视觉后验和影子地图不再堆叠在一个约 2400 行的源文件中。
主流程仍采用编译期调度选择，因此默认构建不会为未选择的调度引入运行时分派；拆分也不改变
矩阵装配顺序和浮点累加顺序。完整源码对应关系见 [SOURCE_LAYOUT.md](SOURCE_LAYOUT.md)。

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

## 后续文档数学增强

在代码归档之后，文档又补充了一轮可独立审阅的数学说明：

- 新增 [数学总流程](MATHEMATICAL_PIPELINE.md)，串联 IMU 传播、clone 增广、重投影、
  Schur 消元、有效子空间、序贯 Joseph 更新和地图后处理；
- [Landmark 参数化](LANDMARK_PARAMETERIZATION.md) 补齐 World/Anchored XYZ、三自由度逆深度、
  log-depth 的正反变换、锚帧雅可比、Schur 坐标不变性和协方差回变换；
- [RD-VIO 调度](RDVIO_SCHEDULING.md) 补齐论文判据与工程分位数判据的差异、四 Case 状态机、
  无深度旋转因子、零平移风险与 R 子窗压缩流程；
- 三角化、QR/Schur、`Hll/Hpp` 零空间、FEJ、Landmark 修正、仿真生成和报告指标均新增
  对应公式及 Mermaid 流程图。

该小节所述的数学增强阶段当时只修改 Markdown；后续源码拆分、帧策略和长时回归修复
已在本文后续章节单独记录。

## 版本库边界

本归档提交源码、CMake 配置、实验脚本和 Markdown 结论。`out/` 中的 CSV/HTML 属于可再生实验产物，不进入提交；IDE 配置、构建目录和 `tmp/` 下的论文/临时仓库同样不归档到 Git。归档分支为 `codex/covariance-stability`，本文所在提交即本轮归档点。

## 100 秒长时发散回归修复

后续长时回归暴露出调度与几何门限不匹配：一次性 MSCKF 只保留 10 个 clone，却继承了持久窗口
实验使用的 8 度三角化门限。在 20 Hz 相机频率下，大多数轨迹到达窗口边界时仍未积累足够视差，
随后被永久丢弃。

- MSCKF 默认保留 20 个 clone；
- MSCKF/RD-VIO 默认使用 2 度门限，历史重复窗口模式仍使用 8 度；
- `VinsAnalysis` 输出 clone 数、视差门限与轨迹丢弃率，并在丢弃率超过 80% 时告警；
- 多场景脚本不再为一次性调度扫描无效的 `uv_var`；
- 三角化门限扫描覆盖 1、2、3、5、8 度。

同一组 100 s / 600 点实验中，四个场景位置 RMSE 从 238.9374、70.0672、1.8628、
409.8012 m 降至 0.6409、2.8183、0.6934、0.4838 m；平均 NIS 仍接近 1，且所有场景
保持 `reused=0`、`blocked=0`。

## MSCKF 帧策略解耦与默认策略

视觉后验生命周期和帧保留策略现已成为两个独立的编译期选择：

- `SCHUR_VIO_VISUAL_SCHEDULER` 控制一次性或重复窗口后验语义；
- `SCHUR_VIO_FRAME_POLICY` 控制图像增广、关键帧/R-N 分类和 clone 删除；
- `SCHUR_VIO_FRAME_WINDOW_SIZE` 可固定公共 clone 预算，用于公平消融。

`tools/run_frame_policy_analysis.ps1` 固定 MSCKF、WORLD_XYZ、20 个 clone 和 2 度门限。
100 s / 600 点结果中，关键帧策略在 Circle-out、Circle-in、Helix-3D、Stop-go 和
Rotation-translation 上分别为 0.3443、0.4179、0.2013、0.6688、8.4281 m；FIFO 分别为
0.6409、2.8183、0.6934、0.4838、8.9361 m。因此 MSCKF 的 `AUTO` 默认解析为关键帧策略，
同时保持 `reused=0`、`blocked=0`。

VINS-Mono 的三角化重试不再依赖观测数量，而记录最新观测帧 ID。固定长度滚动窗口替换旧帧后，
即使观测总数不变也会重新尝试三角化。修复后的 100 s VINS-Mono 四个标准场景结果为
0.3211、0.1234、0.0348、0.1661 m，不再出现 Stop-go 零更新；但其 NIS 仍约 0.001，
因为当前 ESKF 对照实现仍会重复使用活动窗口残差。

## 源码拆分与注释增强

- 将原 `schur_vins.cpp` 拆为外层流程、IMU、三角化、视觉后验和影子地图五个编译单元；
- 补充 clone 增广、IMU 传播、多视图三角化、`Hll` 伪逆、Schur 消元、FEJ、
  序贯伪量测和 Joseph 协方差更新的中文公式注释；
- 删除视觉后验中由 `#if 1` 屏蔽的旧直接逆更新死代码，保留可回归的 QR/Schur 主路径；
- 新增 [SOURCE_LAYOUT.md](SOURCE_LAYOUT.md) 与 [SHADOW_LANDMARKS.md](SHADOW_LANDMARKS.md)；
- 将根目录早期调试记录移入 `docs/archive/legacy_debug/`，并明确其历史属性；
- `out/*.csv`、`out/*.html` 与 `out/*.log` 统一视为可再生实验产物，不进入提交。

## 默认 MSCKF 混合后端

在保持 MSCKF 一次性像素生命周期不变的前提下，新增受控的混合后端：

- 默认路径复用 RD-VIO 的 R/N 运动分类和无深度旋转残差，但不采用四 Case 调度、R 子窗压缩
  或零平移先验；
- 低视差且仍可见的轨迹延迟消费，先使用最新纯旋转观测，等待后续平移基线；
- 已丢失的低视差轨迹保存首尾 bearing、相机位姿快照和时间摘要；历史快照只用于影子候选
  筛选，不直接构造导航残差；
- 影子候选通过 3D NIS、几何评分和稳定次数检查后，只能由当前有效 clone 的 Schur 正规方程
  完成延迟初始化；
- 最多 20 个持久 Landmark 追加到联合状态，完整维护 `P_xl/P_ll`，并通过当前关键帧直接 EKF
  更新；IMU 传播和 clone 增广同步维护交叉协方差；
- 持久点晋升采用 4×3 图像网格限额，默认每格最多 2 个；
- 默认旋转信息尺度为 0.02，持久点量测方差倍率为 64，均由严格长时扫描确定。

源码新增 `eskf/schur_vins_persistent.cpp` 和 `eskf/schur_vins_track_archive.cpp`，分析输出新增延迟、
轨迹摘要、候选池、持久点和无深度旋转约束统计；`summary.csv`、调度/帧策略 CSV 与 HTML 表格
同步支持这些字段。

100 s / 600 点回归中，Circle-out、Circle-in、Helix-3D、Stop-go、Rotation/translation 的位置
RMSE 分别从 0.3443、0.4179、0.2013、0.6688、8.4281 m 变为 0.3262、0.3124、0.0867、
0.0604、7.3984 m；所有场景保持 `reused=0`、`blocked=0`。
