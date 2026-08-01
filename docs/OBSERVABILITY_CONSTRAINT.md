# Schur 视觉后验的可观性约束

## 结论

当前实现采用 **clone 位姿 FEJ（First-Estimate Jacobian）**，默认开启；landmark 仍在当前估计处重线性化。Schur 后对正规方程做四维硬投影的实验开关予以保留，但默认关闭。

30 秒、600 特征、相同噪声与三角化配置的四场景 A/B 表明：FEJ 显著改善两种圆周弱激励场景，Helix-3D 和 Stop-go 仅有小幅波动，全部测试均无负协方差。

| 场景 | 原始位置 RMSE：FEJ / 基线 (m) | 对齐 ATE：FEJ / 基线 (m) | 1 s RPE：FEJ / 基线 (m) | 判断 |
|---|---:|---:|---:|---|
| Circle-out | 0.223 / 1.203（-81.4%） | 0.187 / 1.049（-82.2%） | 0.0607 / 0.2420（-74.9%） | 显著改善 |
| Circle-in | 0.0969 / 0.6210（-84.4%） | 0.0823 / 0.5190（-84.2%） | 0.0361 / 0.1509（-76.1%） | 显著改善 |
| Helix-3D | 0.0677 / 0.0577（+17.2%） | 0.0461 / 0.0395（+16.6%） | 0.0237 / 0.0231（+2.8%） | 小幅退化，仍为厘米级 |
| Stop-go | 0.1165 / 0.1137（+2.4%） | 0.0700 / 0.0674（+3.8%） | 0.0510 / 0.0507（+0.5%） | 基本持平 |

完整逐行数据写入 `out/observability_summary.csv`，轨迹与逐次后验写入 `out/traj_*_oc_{on,off}.csv` 和 `out/update_*_oc_{on,off}.csv`。

## 四维不可观方向

当前误差姿态采用左乘形式：

\[
R^+ = \operatorname{Exp}(\delta\theta)R.
\]

对第 \(i\) 个 clone，视觉预测为：

\[
z_i = \pi\!\left(R_{ic}^{T}\left[R_{wi}^{T}(p_l-p_i)-t_{ic}\right]\right).
\]

在有重力的 VIO 中，绝对平移三维和绕重力方向的全局偏航一维不可观。令单位重力轴为 \(\hat g\)，并用一个窗口位置 \(p_0\) 作数值锚点，clone 的四维零空间块为：

\[
N_i =
\begin{bmatrix}
0_{3\times3} & \hat g \\
I_{3\times3} & -[p_i-p_0]_{\times}\hat g
\end{bmatrix}.
\]

其中前三列是共同平移，最后一列是共同偏航。减去 \(p_0\) 只是在偏航列中组合了已有平移列，不改变零空间张成的子空间，但会改善 \(N^TN\) 的条件数。

独立程序 `ObservabilityBasisCheck` 按实际 `J_pose/J_lmk` 公式构造多视图正规方程。正确符号组合的 `J*N` 和 Schur 后 `Hpp*N` 均约为 \(10^{-16}\)，相反符号约为 \(10^{-2}\) 到 \(10^0\)。

## 为什么使用 FEJ

如果每次都在后验修正后的 clone 位姿处重新计算雅可比，当前雅可比的零空间会随线性化点漂移，而传播协方差仍携带历史线性化信息。反复更新后，滤波器可能把本来不可观的全局平移或偏航误当作新信息，表现为协方差过度收缩和弱激励轨迹漂移。

实现中：

1. clone 增广完成时调用 `record_to_state_fej()`，冻结该 clone 的首次姿态和位置；
2. 残差、鲁棒权重和可见性检查仍使用当前状态；
3. 开启 FEJ 后，`J_pose/J_lmk` 使用 clone 的 `q_fej/p_fej`；
4. landmark 使用当前 `position`，因为它是长寿命、持续精化的地图点。一次更新内所有观测共享同一 landmark 线性化点，因此联合平移/旋转 gauge 仍成立，同时避免永久冻结旧深度。

这相当于约束短寿命滑窗状态的线性化点，而允许持久 landmark 正常重线性化。

## 为什么没有默认开启 Schur 后硬投影

实验过以下投影：

\[
\Pi = I-N(N^TN)^{-1}N^T,\qquad
H_{pp}\leftarrow \Pi H_{pp}\Pi,\qquad
g_p\leftarrow \Pi g_p.
\]

它必须在 landmark Schur 消元后应用；消元前的完整 gauge 同时包含 pose 和 landmark 分量，只投影 pose 块会破坏联合正规方程。

即便放在 Schur 后，当前实现中硬投影仍会明显恶化结果。原因是 `Hll` 采用带阈值伪逆、`Hpp` 又用相对阈值跳过近零 LDLT 方向，约化梯度与半正定信息矩阵在数值零空间附近不再严格同域。对 `gp` 做精确四维投影会改变序贯伪量测的残差组合，反而放大微小不一致。

10 秒、200 特征的反例中，硬投影开启后：

| 场景 | 硬投影位置 RMSE (m) | 无硬投影基线 (m) |
|---|---:|---:|
| Circle-out | 0.550 | 0.187 |
| Circle-in | 0.091 | 0.068 |
| Helix-3D | 0.780 | 0.022 |
| Stop-go | 0.896 | 0.071 |

因此生产配置只使用 FEJ，不使用硬投影。硬投影代码作为后续研究入口保留；若要重新启用，应先统一 `Hll` 的秩判定、Schur 伪逆与 `Hpp/gp` 的有效子空间，而不是单独投影梯度。

## 一致性与数值方向统计

完整 A/B 的平均 NEES（FEJ / 基线）分别为：Circle-out `0.288 / 0.928`、Circle-in `0.202 / 0.330`、Helix `0.221 / 0.216`、Stop-go `0.326 / 0.360`。当前绝对值整体小于 1，说明协方差仍偏保守；FEJ 改善了圆周场景精度，但不是噪声参数标定的替代品。

平均 Hpp 跳过方向约为 `40.3–42.2`，开启 FEJ 前后差异小于 `0.21`。它大于之前讨论的约 31 个零特征值是正常的，原因包括：

- 视觉 Hessian 对核心 INS 状态行天然为零；
- 窗口启动阶段存在未激活 clone；
- 弱视差、特征分布和 LDLT 基会让额外方向落入 `1e-6*d_max` 阈值；
- 统计值是所有更新的平均，不是某一次满窗口 Hpp 的特征值计数。

四场景的 `neg_cov` 均为 0。FEJ/基线累计 `t_cost` 分别为 `21.07/20.60 s`，约增加 2.3%；单场景计时波动较大，这一数字只说明开销较小，不作为微秒级性能结论。

## 开关与复现实验

运行时成员默认值：

```cpp
bool enforce_observability_constraint_ = true;  // clone FEJ
bool project_observability_constraint_ = false; // 实验性 Schur 硬投影
```

复现四场景 A/B：

```powershell
.\tools\run_observability_analysis.ps1 -Duration 30 -Features 600
```

数值检查：

```powershell
cmake --build cmake-build-release --target ObservabilityBasisCheck
.\cmake-build-release\ObservabilityBasisCheck.exe
```
