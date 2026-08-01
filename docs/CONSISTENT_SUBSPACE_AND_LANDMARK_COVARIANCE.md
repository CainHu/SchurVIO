# 一致有效子空间与 Landmark 协方差实验

本文记录两组实现与严格实验：Schur 视觉更新的一致有效子空间硬投影，以及不维护完整
`Pxl` 时可用的 landmark 协方差/地图后处理策略。生产默认仍为 `FEJ + Retriangulate`；
硬投影、独立 EKF 膨胀和影子地图均为显式实验开关。

## 1. 为什么旧硬投影会失败

单个 landmark 消元前的正规方程为

$$
\begin{bmatrix}H_{pp}&H_{pl}\\H_{lp}&H_{ll}\end{bmatrix}
\begin{bmatrix}\delta x_p\\\delta x_l\end{bmatrix}
=\begin{bmatrix}g_p\\g_l\end{bmatrix}.
$$

若 `Hll` 的伪逆、Schur 梯度和 `Hpp` 状态更新分别采用不同分解或阈值，数值上可能出现

$$
g_s\notin\operatorname{Range}(H_s),\qquad
H_s=H_{pp}-H_{pl}H_{ll}^{+}H_{lp}.
$$

此时只投影 `gp`，再对 `Hpp` 运行另一套 LDLT 截断，会使矩阵与梯度不在同一个有效域内。
旧实验的严重退化并不是“不可观方向不能投影”，而是这套数值实现不一致。

## 2. Hll 的统一秩判定

现在每个 `Hll` 只做一次对称特征分解：

$$
H_{ll}=U\Lambda U^T,
\qquad
\tau_l=10^{-8}\lambda_{\max}.
$$

定义保留集合 $\mathcal I_l=\{i\mid\lambda_i>\tau_l\}$，则

$$
H_{ll}^{+}=U_{\mathcal I_l}\Lambda_{\mathcal I_l}^{-1}U_{\mathcal I_l}^T.
$$

Schur 矩阵和 Schur 梯度都通过这一个伪逆计算。因此被删除方向同时从
`Hpl*Hll+*Hlp` 和 `Hpl*Hll+*gl` 消失。日志额外记录被删除的 landmark 梯度能量比例：

$$
\rho_l=\frac{\|(I-U_rU_r^T)g_l\|}{\|g_l\|}.
$$

30 s/600 点四场景实验中，`1e-8` 相对阈值没有删除任何 `Hll` 方向，说明此前常见的弱深度
方向仍是有效信息，不能用 `1e-6` 之类更激进阈值直接截掉。

## 3. 先验白化的四维硬投影

状态同时含弧度、米、速度和偏置，直接在原坐标做欧氏投影会依赖单位。先对先验协方差

$$
P=LL^T,
\qquad \delta x=Lz
$$

做 Cholesky 白化，并构造

$$
H_z=L^TH_sL,
\qquad g_z=L^Tg_s,
\qquad N_z=L^{-1}N.
$$

对 $N_z$ 做列主元 QR，得到正交基 $Q_N$，然后

$$
\Pi_z=I-Q_NQ_N^T,
\qquad \bar H_z=\Pi_zH_z\Pi_z,
\qquad \bar g_z=\Pi_zg_z.
$$

对称分解 $\bar H_z=V\Lambda V^T$ 后，仅保留
$\lambda_i>10^{-6}\lambda_{\max}$ 的方向，并显式令

$$
g_{z,\mathrm{eff}}=V_rV_r^T\bar g_z.
$$

状态顺序更新不再重新对投影结果做 LDLT 秩判定，而是直接使用同一组
$V_r,\Lambda_r$。第 $i$ 个标量伪量测在原状态坐标中的形式为

$$
h_i=L^{-T}v_i,
\qquad z_i=\frac{v_i^T\bar g_z}{\lambda_i},
\qquad R_i=\frac{\texttt{uv\_var}}{\lambda_i\,\Delta t}.
$$

这保证 `Hpp`、`gp`、秩判定和最终 Kalman 更新严格共享一个有效子空间。

## 4. 硬投影严格实验

配置：30 s、600 特征、`uv_var=0.01`、密度型 IMU 噪声、偏置随机游走开启、
三角化最小视差 8°、landmark 使用 `Retriangulate`。

| 场景 | FEJ RMSE / 投影 RMSE (m) | 投影后平均泄漏 | Hpp 平均删除方向 | 删除梯度比例最大值 | FEJ / 投影耗时 (s) |
|---|---:|---:|---:|---:|---:|
| Circle-out | 0.2022 / 0.2026 | 1.42e-17 | 40.03 | 2.06e-10 | 1.95 / 26.51 |
| Circle-in | 0.1107 / 0.1088 | 1.19e-17 | 40.09 | 2.13e-4 | 7.64 / 65.31 |
| Helix-3D | 0.0747 / 0.0745 | 1.48e-17 | 42.02 | 1.04e-11 | 6.31 / 63.34 |
| Stop-go | 0.1208 / 0.1222 | 1.11e-17 | 38.60 | 4.29e-4 | 6.41 / 55.46 |

结论：新投影已消除旧实现的灾难性退化，泄漏从约 `1e-3~1e-2` 降到机器精度，同时轨迹
基本持平。但 198 维白化和特征分解使视觉后验耗时增加约 8～14 倍，所以
`project_observability_constraint_` 继续默认关闭，生产只启用 FEJ。

Hpp 删除方向多于“四维 gauge”是正常的。当前矩阵维数固定为 198，视觉 Hessian 不直接约束
核心 IMU 状态，启动阶段还含未激活 clone；再叠加视差、特征分布和几何退化，完整实验平均约
39～42 个低信息方向。真正需要检查的是被删除梯度比例，而不是要求删除数量固定等于 4 或 31。

## 5. 为什么给独立 landmark 加 Q 不能补出 Pxl

联合量测创新协方差应为

$$
S=H_xP_{xx}H_x^T+H_lP_{ll}H_l^T
 +H_xP_{xl}H_l^T+H_lP_{lx}H_x^T+R.
$$

独立 landmark EKF 丢失了中间两个有符号、各向异性的交叉项。加半正定过程噪声
$Q_l\succeq0$ 只能放大 `Pll`，不能等价恢复 `Pxl`。

实现保留三条实验线：

- `IndependentEkf`：历史算法；
- `IndependentEkfInflated`：每次视觉更新加 $Q_l=q_l\Delta t I$；
- `IndependentEkfAdaptive`：按 landmark 标量 NIS 的指数滑动平均缩放 $Q_l$。

默认实验值为 $q_l=10^{-3}\ \mathrm{m^2/s}$。三维 landmark 的 NEES 理论均值为 3，
95% 卡方阈值为 7.8147；因此仅看点误差不足以判断协方差是否正确。

| 场景 | 旧独立：点误差 / NEES / 95%覆盖率 | 固定膨胀：点误差 / NEES / 覆盖率 | 重三角化：点误差 / NEES / 覆盖率 |
|---|---:|---:|---:|
| Circle-out | 0.375 / 76.84 / 16.9% | 0.384 / 22.81 / 21.8% | 0.329 / 1.37 / 100% |
| Circle-in | 0.0666 / 5.13 / 82.0% | 0.0809 / 2.63 / 91.1% | 0.123 / 0.405 / 100% |
| Helix-3D | 0.0297 / 17.71 / 56.2% | 0.0409 / 1.94 / 98.5% | 0.0523 / 0.378 / 100% |
| Stop-go | 0.0501 / 2.14 / 95.5% | 0.0551 / 1.14 / 99.2% | 0.228 / 0.833 / 99.7% |

固定膨胀能缓解过度自信，但地图误差普遍略增，Circle-out 仍严重不一致。NIS 自适应结果与
旧独立 EKF 基本相同：共享状态误差和重复相关量测并不一定造成大的“独立 landmark 创新”，
所以 NIS 无法可靠检测缺失的 `Pxl`。这正是不能把过程噪声当作联合协方差替代品的实验依据。

## 6. 解耦的影子地图后处理器

`enable_shadow_landmark_postprocessor_` 为每个点维护
`shadow_position/shadow_cov_position`。它只消费最新关键帧观测，绝不写回 ESKF 构造视觉残差
所用的 `Landmark::position`，因此影子协方差即使近似，也不会污染导航后验。

影子量测把 clone 位姿协方差并入等效噪声：

$$
R_{\mathrm{eff}}=\sigma_{uv}^2I+J_xP_{xx}^{(clone)}J_x^T.
$$

当当前归一化创新大于 1 时，再使用有上限的 fading factor 立即放大点协方差，并用 Joseph
形式更新。完整实验结果如下：

| 场景 | 重三角化点误差 (m) | 影子地图点误差 (m) | 影子 NEES | 影子 95%覆盖率 |
|---|---:|---:|---:|---:|
| Circle-out | 0.329 | 0.312 | 4.73 | 82.6% |
| Circle-in | 0.123 | 0.0651 | 1.87 | 100% |
| Helix-3D | 0.0523 | 0.0298 | 7.20 | 52.2% |
| Stop-go | 0.228 | 0.0478 | 5.16 | 81.0% |

影子地图在四个场景都降低了 gauge 对齐点误差，而且完全不改变 ESKF 轨迹；但 Helix 等场景
覆盖率仍不足，说明它依然不是严格联合滤波。建议把它用于“需要更好地图输出、但不让地图
协方差反馈导航”的后处理场景，不用于宣称严格一致的 landmark 概率估计。

## 7. 默认选择与复现

默认配置保持：

```cpp
enforce_observability_constraint_ = true;
project_observability_constraint_ = false;
landmark_update_mode_ = LandmarkUpdateMode::Retriangulate;
enable_shadow_landmark_postprocessor_ = false;
```

复现命令：

```powershell
.\tools\run_observability_analysis.ps1 -Duration 30 -Features 600
.\tools\run_landmark_consistency_analysis.ps1 -Duration 30 -Features 600
```

输出文件：

- `out/observability_summary.csv`：泄漏、秩、删除梯度比例、轨迹和耗时；
- `out/landmark_consistency_summary.csv`：最终点误差、NEES、95%覆盖率和影子地图指标；
- `out/triangulation_<scenario>_<tag>.csv`：逐点最终/影子位置、协方差、NEES 与覆盖结果。

若未来要求 landmark 概率严格一致，下一步应实现活动 landmark 子集的完整 `Pxl` 基准，
而不是继续调大 `Ql`。该改动会改变状态增广、边缘化和存储复杂度，应该作为独立架构实验。
