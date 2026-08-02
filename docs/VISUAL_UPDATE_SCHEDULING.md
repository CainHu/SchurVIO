# 视觉后验调度与观测生命周期

## 结论

默认采用 `MSCKF` 调度：每张图像增广相机位姿，但一条特征轨迹只在结束、触及滑窗边界或达到最大长度时进入一次 Schur 后验；更新后整条轨迹立即消费并释放。同一个像素观测不会被当作新的独立量测重复送入 ESKF。

项目同时保留 `LEGACY`、`SCHURVINS` 和 `VINS_MONO` 三个对照模式，通过编译期宏切换。后三者中的 `VINS_MONO` 是“VINS-Mono 风格的帧保留/删除调度”，不是完整的 VINS-Mono 非线性优化器。

## 为什么一次性消费是长期默认方案

设先验为 \(p(x)\)，一批视觉观测为 \(z\)，正常的一次后验是

\[
p(x\mid z) \propto p(z\mid x)p(x).
\]

若下一相机时刻没有新增该特征的观测，却再次把同一批 \(z\) 当成独立量测更新，相当于

\[
\tilde p(x\mid z) \propto p(z\mid x)^2p(x),
\]

继续重复则似然被提升到更高次幂。均值不一定立刻明显错误，但协方差会过度收缩，NIS/NEES 和后续增益会失真。若希望像滑窗优化那样反复重线性化历史残差，就必须保留并一致地边缘化旧信息因子，而不能把每次重求解都当作一次新的 EKF 量测。

MSCKF 模式仍使用当前项目的 Schur 消元。对一条轨迹的线性化方程

\[
r = H_x\delta x + H_f\delta f + n
\]

构造联合正规方程后消去特征增量：

\[
H_{xx}^{s}=H_{xx}-H_{xf}H_{ff}^{\dagger}H_{fx},\qquad
g_x^{s}=g_x-H_{xf}H_{ff}^{\dagger}g_f.
\]

这和经典 MSCKF 的左零空间投影在保留相同有效秩时等价；区别只是本项目继续复用已经优化好的 Schur 路径。

## 五种可切换策略

| 策略 | 增广 | 何时更新 | 轨迹生命周期 | 窗口处理 | 定位 |
|---|---|---|---|---|---|
| `LEGACY` | 仅关键帧 | 每个相机时刻都解当前关键帧批次 | 持久，历史观测会重复进入后验 | 满 30 帧后删最老帧 | 原实现回归基线 |
| `SCHURVINS` | 每张图像 | 每张图像解活跃轨迹 | 持久并可重复线性化 | 更新时最多 4 帧参与，随后保留最新帧和较新的关键帧，共 3 帧 | 论文/官方代码风格对照 |
| `MSCKF` | 每张图像 | 轨迹丢失、触及待删 clone 或长度达到 10 | 更新一次后整条删除；同 ID 后续重新建轨 | 更新时最多 11 帧参与，随后保留 10 帧 | 默认、Bayes 生命周期最清楚 |
| `VINS_MONO` | 每张图像 | 每张图像解当前批次 | 本 ESKF 对照模式中仍会重复 | 次新帧是关键帧则删最老帧，否则删次新非关键帧 | 仅帧调度 A/B |
| `RDVIO` | 每张图像 | N 轨迹用 Schur；未三角化 R 轨迹用无深度旋转因子 | 与 MSCKF 一样一次性消费 | RR/NN/RN/NR 分层保留，长 R 子窗按 3:1 压缩 | 实验模式，详见 `RDVIO_SCHEDULING.md` |

### SchurVINS 模式的边界

官方 SchurVINS 每张图像执行状态增广和 Schur 更新，并使用很小的状态窗口；论文中的地图点协方差更新不保存完整 \(P_{xl}\)。本实现复现其调度思想并保留现有 Schur/landmark 实验接口，但不会声称已经逐行复刻官方前端、关键帧选择和地图管理。

### VINS-Mono 模式的边界

VINS-Mono 在固定窗口内反复优化同一组残差是合理的，因为它是在重求解同一个代价函数；窗口满后还会把被删除状态和相关量测压缩成边缘化先验。本项目目前没有等价的非线性边缘化因子，因此这里只实现其帧删除规则：

- 次新帧为关键帧：边缘化最老帧；
- 次新帧为非关键帧：删除次新帧的视觉状态，保留最新帧。

该模式用于隔离“窗口调度”带来的影响，不应作为生产默认值。

### ROVIO/RVIO 与 RD-VIO 适配的边界

ROVIO/RVIO 的核心差异不是“什么时候调用同一个 `updateVisual`”：它们涉及直接光度残差、robocentric 状态或不同前端，仍不能只靠帧删除顺序复刻。`RDVIO` 模式则已实现可一致迁移的 R/N 分类、四 Case、延迟三角化、无深度旋转量测和子窗压缩；没有前端支持的 IMU-PARSAC 与完整 BA 明确留在实现边界之外，详见 `RDVIO_SCHEDULING.md`。

## 宏与构建方式

公共数值宏定义在 `eskf/visual_update_scheduler.h`：

```cpp
#define SCHUR_VIO_SCHEDULER_LEGACY 0
#define SCHUR_VIO_SCHEDULER_SCHURVINS 1
#define SCHUR_VIO_SCHEDULER_MSCKF 2
#define SCHUR_VIO_SCHEDULER_VINS_MONO 3
#define SCHUR_VIO_SCHEDULER_RDVIO 4
```

默认值是 `SCHUR_VIO_SCHEDULER_MSCKF`。推荐通过 CMake 选择，它最终仍生成编译期宏，不产生运行时分支：

```powershell
cmake -S . -B cmake-build-release -DSCHUR_VIO_VISUAL_SCHEDULER=MSCKF
cmake --build cmake-build-release --target VinsAnalysis -j 4
```

可选字符串为 `LEGACY`、`SCHURVINS`、`MSCKF`、`VINS_MONO`、`RDVIO`。

## 审计与回归

每个 `Observation` 保存 `visual_update_count`。分析程序输出：

- `new_observations`：第一次进入后验的有效观测数；
- `reused_observations`：再次进入后验的观测数；
- `duplicate_observations_blocked`：MSCKF 防御性检查拦截的重复观测数；
- `tracks_consumed / tracks_dropped`：一次性结束的轨迹数和未成功进入后验的轨迹数；
- `max_window`：更新时参与的最大 clone 数。

MSCKF 的两个硬性回归条件为

\[
N_{reused}=0,\qquad N_{blocked}=0.
\]

运行四策略对比：

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_scheduler_analysis.ps1 -Quick
```

完整四场景对比去掉 `-Quick`。结果写入 `out/scheduler_summary.csv`；脚本最后会把构建目录恢复为默认 `MSCKF`。

### 当前快速回归结果

在 Circle-out、5 s、150 个特征点、最小三角化视差 3° 的 Release 冒烟实验中：

| 策略 | 新观测 | 重复观测 | 重复率 | NIS 均值 | 后验耗时 | 更新时最大窗口 |
|---|---:|---:|---:|---:|---:|---:|
| Legacy | 449 | 22691 | 98.1% | 0.00033 | 0.121 s | 25（短实验尚未到 30） |
| SchurVINS | 69 | 179 | 72.2% | 0.754 | 0.018 s | 4 |
| VINS-Mono 风格 | 1382 | 13274 | 90.6% | 0.00027 | 0.101 s | 11 |
| MSCKF | 1147 | 0 | 0% | 0.994 | 0.027 s | 11 |

这个短实验只用于验证调度和信息生命周期，不能按位置 RMSE 给算法排名。Legacy/VINS-Mono 风格的重复后验会人为增强信息，而且为保留历史行为仍使用 `uv_var/dt` 信息密度；MSCKF/SchurVINS 使用归一化图像观测方差。不同噪声语义下的单个 RMSE 不是严格同变量消融。

默认 MSCKF 在相同 3° 视差门槛下还通过了 Circle-out、Circle-in、Helix-3D、Stop-go 四个 8 s 场景：四者均为 `reused=0`、`blocked=0`，NIS 均值分别约为 1.02、0.97、0.78、0.89。Circle-in 仍有大量低视差轨迹三角化失败，8 s 位置 RMSE 约 1.38 m；这应通过窗口长度、视差触发和延迟初始化继续优化，不能靠重复使用历史量测掩盖。

## 代码结构

- `eskf/visual_update_scheduler.h`：策略宏、名称和窗口参数；
- `SchurVINS::updateVisual`：轨迹触发条件、一次性消费和帧删除计划；
- `SlidingWindow::active_idx`：把时间顺序与固定协方差物理 slot 解耦；
- `Map::removeLandmark`：安全删除轨迹在 frame/feature/observation 两侧的所有引用；
- `SchurVINS::updateState`：使用 `Frame::ordering` 更新对应的物理协方差块，支持删除任意时间位置的 clone。

## 参考实现

- [SchurVINS 官方仓库](https://github.com/bytedance/SchurVINS)
- [OpenVINS 官方仓库（MSCKF 轨迹消费与 clone 边缘化）](https://github.com/rpng/open_vins)
- [VINS-Mono 官方仓库](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
