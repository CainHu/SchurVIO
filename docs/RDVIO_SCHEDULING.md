# RD-VIO R/N 分层调度的 Schur-ESKF 适配

## 实现边界

这里实现的是 RD-VIO 中能够一致迁移到当前 Schur-ESKF 的部分，不声称复刻原论文的完整非线性优化器：

- 已实现：R/N 判定、RR/NN/RN/NR 分层调度、R 帧延迟三角化、无深度旋转约束、长 R 子窗压缩、可选零平移约束；
- 未伪装实现：图像前端的 IMU-PARSAC、空间分桶、track longevity 权重、完整 keyframe/subframe BA、预积分因子拼接；
- 论文采用固定 bearing 的 1-DOF 逆深度，本项目此次只研究独立的 3-DOF 参数化，见 `LANDMARK_PARAMETERIZATION.md`。

默认生产配置仍是 `MSCKF`；RD-VIO 是可切换实验模式：

```powershell
cmake -S . -B cmake-build-release -DSCHUR_VIO_VISUAL_SCHEDULER=RDVIO
```

## R/N 判定

对相邻帧公共 bearing，先用 IMU 传播姿态补偿旋转：

\[
\hat b_j=R_{wc,j}^{T}R_{wc,i}b_i,
\qquad
\theta_k=\cos^{-1}(b_j^T\hat b_j).
\]

与论文官方实现一致，排序后取 70% 分位角而不是最大角，降低单个误匹配的影响：

\[
Q_{0.7}(\theta)<\theta_R \Rightarrow R\text{-frame}.
\]

当前仿真含约 1 pixel 噪声，阈值为 `0.60°`；官方真机配置更严格，不能跨传感器机械照搬。公共轨迹少于 20 时保守判为 N。

## 四种 Case

`R` 表示旋转主导帧，`N` 表示有足够平移视差的普通帧：

| Case | 调度 |
|---|---|
| RR | 当前 R 作为 subframe 追加 |
| NN | 追加 N subframe；达到 `subframe_size=3` 后创建新 keyframe |
| RN | 将最后一个 R subframe 提升为 keyframe，当前 N 也成为 keyframe |
| NR | 将最后一个 N subframe 提升为 keyframe，当前 R 挂到其后 |

当前 ESKF 是扁平 clone 窗口，`is_key_frame=false` 表示论文中的 subframe。它没有 BA 中的嵌套对象，但保留了相同的状态保留/提升时序。

## R 帧为什么延迟三角化

纯旋转时相机中心几乎不移动，两条 bearing 只改变方向而没有可靠基线。三角化矩阵的深度方向趋于退化，强行初始化会得到任意远深度和虚假小协方差。因此 R 帧对未初始化轨迹只保留 bearing，等 N 帧提供平移后再三角化。

已经三角化的老点仍可使用普通重投影，因为它们的深度来自更早的有效基线。

## 无深度旋转约束

对最终无法三角化且即将一次性消费的 R 轨迹，使用切平面残差

\[
r_R=B_j^T\left(b_j-R_{wc,j}^TR_{wc,i}b_i\right),
\]

其中 `B_j` 的两列张成 `b_j` 的正交切平面。该残差不含 landmark 深度，只约束两帧姿态。两个 bearing 都含噪，因此正则方程权重取 `1/2`。

每条轨迹只选择最近的一次 R 转换，随后整条轨迹删除；这保持 `reused_observations=0`，避免把 rotation factor 用过的样本又送入后续结构化 Schur 更新。

## 零平移约束与风险

论文在 R 子窗中可加入

\[
r_p=-(p_j-p_i),\qquad R_p=\sigma_p^2I.
\]

本实现默认 `σ_p=0.03 m`。它能抑制纯旋转阶段 IMU 双积分漂移，但其正确性完全依赖 R/N 分类。一旦把缓慢平移误判为 R，该约束就会把真实位移压回零。`RotationTranslation` 场景专门暴露这个风险；后续调参应同时扫描角阈值与 `σ_p`，不能只看普通圆周轨迹。

## R 子窗压缩

连续 R subframe 达到 9 帧后，每 3 帧只保留最后一帧。论文优化器会同时拼接 IMU 预积分；当前 ESKF 的 clone 间没有显式预积分因子，所以做法是：先消费触及待删 clone 的轨迹，再按时间逆序释放 clone 和协方差 slot。

## 8 秒严格对比

共同配置：250 点、归一化图像噪声来自 1 pixel、最小三角化视差 3°、观测严格一次性消费。

| 场景 | MSCKF 对齐 ATE / m | RD-VIO 对齐 ATE / m | MSCKF 1 s RPE / m | RD-VIO 1 s RPE / m |
|---|---:|---:|---:|---:|
| Circle-out | 0.2454 | **0.2114** | 0.1584 | **0.1152** |
| Circle-in | 0.9907 | **0.6955** | 0.5093 | **0.3126** |
| Helix-3D | 0.2784 | **0.0909** | 0.1498 | **0.0646** |
| Stop-go | **0.2093** | 0.2613 | **0.1385** | 0.2876 |
| Rotation/translation | 未纳入本次同批 MSCKF 报告 | 3.1152 | — | 2.9848 |

这组结果不能简单得出“RD-VIO 总是更好”：普通前三个场景改善，但 Stop-go 明显恶化，专门构造的 R/N 场景还暴露了慢平移误判和长时间弱位置可观导致的不一致（NEES 很大）。因此 RD-VIO 保持实验宏，默认仍用 MSCKF。

## 报告与回归

- `out/report_msckf_schur.html`：MSCKF-Schur 四场景报告；
- `out/report_rdvio_schur.html`：RD-VIO-Schur 四场景报告；
- `out/scheduler_summary.csv`：调度、Case 计数、压缩、旋转因子和零平移因子统计；
- `rotation_translation`：交替 4 s 纯旋转、6 s 平移的周期轨迹，用于覆盖 RR/NN/RN/NR。

完整调度对比：

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_scheduler_analysis.ps1
```
