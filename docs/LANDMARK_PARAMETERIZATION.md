# 三自由度 Landmark 参数化对比

## 结论

当前推荐仍是默认 `WORLD_XYZ`。三自由度锚定逆深度显著改善了单点信息矩阵 `Hll` 的数值尺度，但在严格保持同一状态、同一观测、同一 Schur 消元和同一秩阈值时，没有观察到可确认的轨迹精度提升。

5 s、150 点、Circle-out、MSCKF-Schur 快速消融如下：

| 参数化 | 对齐 ATE / m | 1 s RPE / m | Hll 平均有效条件数 | Hll 最大有效条件数 | 后验总耗时 / s |
|---|---:|---:|---:|---:|---:|
| World XYZ | 0.241925 | 0.200313 | 3693.3 | 12881 | 0.025 |
| Anchored XYZ | 0.242036 | 0.200478 | 3693.1 | 12882 | 0.027 |
| Anchored `(α,β,ρ)` | 0.242427 | 0.200753 | **47.9** | **83.0** | 0.025 |
| Anchored `(α,β,log z)` | 0.242573 | 0.200859 | 1017.8 | 2671 | 0.023 |

逆深度把平均有效条件数降低约 77 倍，但对齐 ATE 的差异只有约 0.5 mm，属于短实验中的数值路径差异，不能宣称精度提升。它更适合被视为“低视差/远点时的数值保险”，而不是新的信息来源。

## 四种 3-DOF 坐标

地图中始终保存世界系点 `p_w`。宏只改变本次 Schur 线性化的局部增量 `δλ`：

```powershell
cmake -S . -B cmake-build-release `
  -DSCHUR_VIO_LANDMARK_PARAMETERIZATION=ANCHORED_INV_DEPTH
```

可选值：

- `WORLD_XYZ`：`λ = p_w`；
- `ANCHORED_XYZ`：`λ = p_ca`，即锚帧相机系 XYZ；
- `ANCHORED_INV_DEPTH`：`λ = (α,β,ρ)`，其中 `α=x/z, β=y/z, ρ=1/z`；
- `ANCHORED_LOG_DEPTH`：`λ = (α,β,η)`，其中 `η=log z`。

这些都是三自由度参数化，不包含论文中常见的“固定首帧 bearing、只估计一个逆深度”的 1-DOF 模式。

## 雅可比

设锚帧相机坐标为 `p_ca=(x,y,z)`，锚帧相机到世界的旋转为 `R_wca`。局部增量到世界点增量的变换写成

\[
\delta p_w = T_\lambda\,\delta\lambda.
\]

四种模式分别为

\[
T_{world}=I,\qquad T_{anchor\_xyz}=R_{wca},
\]

\[
T_{inv}=R_{wca}
\begin{bmatrix}
z&0&-xz\\
0&z&-yz\\
0&0&-z^2
\end{bmatrix},
\]

\[
T_{log}=R_{wca}
\begin{bmatrix}
z&0&x\\
0&z&y\\
0&0&z
\end{bmatrix}.
\]

若原世界点雅可比为 `J_w`，则

\[
J_\lambda=J_wT_\lambda.
\]

## 为什么必须加入锚帧位姿雅可比

锚定点不是固定在世界系的常量，而是

\[
p_w=p_{wa}+R_{wia}(t_{ic}+R_{ic}p_{ca}).
\]

保持局部点坐标不变时，锚帧位姿扰动仍会移动世界点。因此还必须加入

\[
J_{anchor}=\begin{bmatrix}
-J_w[p_w-p_{wa}]_\times & J_w
\end{bmatrix}.
\]

当观测帧就是锚帧时，它与观测相机自身的位姿雅可比严格相消。这正是“相机与锚定点一起做刚体运动时，锚帧归一化坐标不变”的物理含义。若漏掉该项，就等价于偷偷把锚帧固定到世界系，会破坏四维 VIO gauge 和 FEJ 可观性约束。

## 为什么理论上不会凭空提高精度

若 `T` 可逆，则

\[
H_{ll}'=T^TH_{ll}T,\qquad H_{pl}'=H_{pl}T,
\]

且精确逆满足

\[
H_{pl}'(H_{ll}')^{-1}H_{lp}'
=H_{pl}H_{ll}^{-1}H_{lp}.
\]

所以完整 Schur 补在精确算术下不变。实际差异来自浮点舍入、鲁棒门控、秩阈值、伪逆方向选择及迭代线性化。逆深度能让深度方向的尺度更均衡，但不会产生新观测信息。

## 回归

运行四种参数化：

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_landmark_parameterization_analysis.ps1 -Quick
```

结果写入 `out/parameterization_summary.csv`，并显示在 HTML 报告第 12 节。
