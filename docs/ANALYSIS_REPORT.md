# SchurVIO 视觉后验、噪声参数与多场景分析

配套交互报告：`out/report.html`。当前分析统一使用 `USE_SCHUR`，`Hpp/Hll` 默认 LDLT，
外参雅可比保留但由 `ESTIMATE_EXTRINSIC=false` 屏蔽。

## 1. 生成方法

```powershell
cmake --build cmake-build-release --target VinsAnalysis VinsReport
tools/run_multi_scenario_analysis.ps1 -Duration 30 -Features 600
```

报告同时展示四类仿真：Circle-out、Circle-in、Helix-3D 和 Stop-go。除详细的三维轨迹、
坐标系朝向、特征点和三角化图外，多场景表还包含：

- 位置/速度/姿态 RMSE 与最大位置误差；
- 视觉更新前后误差、后验改善率和修正量；
- NEES/NIS、协方差包络和负协方差计数；
- Huber 降权率、硬残差拒绝率；
- 三角化成功率、误差、视差、条件数与耗时；
- `uv_var` 和过程噪声倍率扫描。

场景定义和传感器噪声见 [SIMULATION_SCENARIOS.md](SIMULATION_SCENARIOS.md)。

## 2. `uv_var` 的含义与默认值

Schur 路径先把 landmark 消元，得到状态信息矩阵 `Hpp`。对每个有效分解方向
`Hpp h_i=d_i h_i`，序贯伪量测使用

```math
R_i=\frac{\texttt{uv_var}}{d_i\,dt}.
```

因此 `uv_var` 不是像素方差，也不是归一化像平面单帧方差。旧默认值 `400` 会让视觉后验
极度保守。归一化图像噪声约为 `(1/184.75)^2=2.93e-5`，但由于滑窗重复观测、线性化误差
和 Schur 序贯模型，不能把该单帧方差直接代入。

### 含真实 IMU/图像噪声的多阶段扫描

15 秒、300 点快速扫描中，加入正深度/重投影硬门限和 3-sigma Huber 后，`1e-4` 在四类
轨迹上都未发散。但扩展到 30 秒、600 点后，滑窗被充分填满并经历更多 landmark 重用，
`1e-4` 在 Circle-out 上出现约 `6.23 m` 的位置 RMSE。短测试据此被判定不足，不能作为
默认参数依据。

30 秒 Circle-out 长时扫描的关键结果为：

| `uv_var` | 位置 RMSE | 最大位置误差 | 判断 |
|---:|---:|---:|---|
| `1e-4` | 1.87 m | 3.42 m | 过度相信视觉伪量测 |
| `1e-3` | 2.28 m | 3.84 m | 仍有较大漂移 |
| `3e-3` | 1.24 m | 2.11 m | 接近稳定区 |
| `1e-2` | **1.20 m** | 2.12 m | 长时 RMSE 最低 |
| `3e-2` | 1.24 m | 2.11 m | 稍偏保守 |
| `1e-1` | 1.31 m | **2.04 m** | 视觉约束偏弱 |

默认最终采用 `uv_var=1e-2`。这不是每个场景的单独最优值，而是长时最坏误差与视觉约束
强度的稳健折中。Circle-out/Circle-in 的绝对位置仍会沿 VIO 全局平移 gauge 漂移，所以
报告同时增加 1 秒相对位移误差；真实数据仍应按重投影残差、数据关联质量和 NIS 标定。

## 3. Schur 视觉鲁棒门控

进入 Hessian 累计前，每条观测依次执行：

1. 相机深度有限且 `z>0.05 m`；
2. 归一化重投影残差有限且小于 `0.1`；
3. 以 `3*triangulation_uv_std` 为阈值计算 Huber 权重；
4. 一个 landmark 至少保留两个有效观测，否则不参与 Schur 消元和更新。

Huber 权重同时作用于 `Hpp/Hll/Hpl/gp/gl`，保证正规方程内部一致。日志记录每次更新的
`obs_used/obs_downweighted/obs_rejected`，报告按场景显示降权率和硬拒绝率。门控稳定了旧扫描
中 Helix-3D 的偶发尖峰，但无法让很小的 `uv_var` 在长时滑窗中可靠，因此量测噪声仍需
保留足够裕量。仅剩一条有效观测的 landmark 不写入 Hessian；第一条先暂存，第二条到来后
再一起累计，避免把未消元点错误当成固定地图约束。

## 4. IMU 噪声离散化与过程噪声

模拟器参数改为连续时间噪声密度。采样周期为 `dt` 时：

```math
\sigma_{white,sample}=\frac{\sigma_c}{\sqrt{dt}},\qquad
\sigma_{bias,step}=\sigma_{rw}\sqrt{dt}.
```

原实现使用 `density/sqrt(rate)`，等价于 `density*sqrt(dt)`，把白噪声标准差低估了
`rate` 倍。现已修复，并恢复偏置随机游走。

统一仿真标称密度和 ESKF 默认过程标准差为：

| 状态方向 | 仿真标称 | ESKF 默认 | 说明 |
|---|---:|---:|---|
| 姿态/gyro | 0.002 | 0.003 | 1.5 倍裕量 |
| 速度/accel | 0.020 | 0.030 | 1.5 倍裕量 |
| gyro bias RW | 0.0001 | 0.0002 | 2 倍裕量 |
| accel bias RW | 0.0005 | 0.0010 | 2 倍裕量 |
| 位置直接 RW | — | 0.0003 | 小量模型裕度 |

在长时 `uv_var=1e-2` 下，Circle-out 的 `proc_scale={0.5,1,2}` 位置 RMSE 分别约为
`{1.64,1.20,1.00} m`，但 Circle-in 从 scale=1 的 `0.62 m` 变为 scale=2 的
`0.69 m`。默认仍保留 `proc_scale=1`，不为单条轨迹重复放大过程噪声。真实 IMU 应以
Allan 方差为主，仿真扫描只验证数量级和稳定区间。

## 5. 为什么关闭重力估计

四个仿真器都使用

```math
g_{true}=[0,0,9.81]^T\ \mathrm{m/s^2},
```

ESKF 初值也是完全相同的向量。在这种已知真值验证中继续估计 `g` 没有收益，反而增加 3 个
状态维度，并引入重力横向分量与横滚/俯仰、加速度计零偏之间的弱可观耦合。因此默认设置

```cpp
INSState::ESTIMATE_GRAVITY = false;
```

关闭后报告中的 `gx/gy/gz` 只用于检查固定值是否被意外修改，`gravity_error` 应严格为 0。
这不代表真实 VIO 一律应关闭重力：若设备初始姿态或重力方向未知，应先做可靠静止初始化，
或开启重力估计并使用有足够三轴激励的数据。

## 6. 视觉后验是否正确修正 ESKF

分析日志在每次视觉更新前后分别保存 `q/p/v`，并与同时间 GT 比较。报告给出：

```math
\Delta e_p=\|p_{prior}-p_{gt}\|-\|p_{post}-p_{gt}\|.
```

`Delta e_p>0` 代表该次视觉更新把位置拉近真值。单次改善率不应机械要求 100%：量测含噪、
状态耦合以及滤波目标是最小化期望协方差而非每次都减小已知真值误差。更重要的是检查：

- 长期 RMSE 是否小于纯 IMU 漂移；
- 先验/后验散点是否大多在对角线下；
- 修正量是否有限、是否出现孤立大尖峰；
- NEES/NIS 是否显示过度自信；
- 四种几何和运动下是否都不发散。

报告中的多场景汇总正是为避免只凭一条“好看”的圆周轨迹判断算法正确。

### 最终 30 秒 / 600 点基线

| 场景 | 绝对位置 RMSE | 1 s 相对位移 RMSE | 最大位置误差 | 后验改善率 | Huber 降权 | 硬拒绝 |
|---|---:|---:|---:|---:|---:|---:|
| Circle-out | 1.203 m | 0.241 m | 2.119 m | 39.0% | 46.85% | 15.89% |
| Circle-in | 0.621 m | 0.151 m | 1.365 m | 72.1% | 26.50% | 4.56% |
| Helix-3D | 0.058 m | 0.023 m | 0.098 m | 64.1% | 4.37% | 0.007% |
| Stop-go | 0.114 m | 0.051 m | 0.207 m | 74.0% | 4.92% | 0.010% |

Circle-out/Circle-in 的绝对误差明显大于局部相对误差，符合平面圆周轨迹下全局平移 gauge
和弱激励累积。Circle-out 的降权/拒绝比例仍偏高，说明短 track、重三角化与滑窗线性化是
当前前端/后端最值得继续改进的部分；报告保留该诊断，不用放宽门限掩盖问题。

## 7. 协方差负值结论

此前负协方差的根因不是 Joseph 视觉更新，而是 IMU 预测只传播 `P_ii`、遗漏
INS 与 clone 的互协方差 `P_ic`。联合状态转移 `F=diag(A,I)` 必须满足

```math
P'_{ii}=AP_{ii}A^T+Q,\qquad P'_{ic}=AP_{ic},\qquad
P'_{ci}=P_{ic}'^T,\qquad P'_{cc}=P_{cc}.
```

三条预测实现路径现均补全该传播；默认使用不显式构造 `A` 的优化分块路径：
`CONFIG_DEBUG=false`、`USE_STABLE_COVARIANCE_PREDICTION=false`。视觉标量更新保留 Joseph
等价形式。多场景汇总中的 `neg_cov` 必须为 0；机器精度量级的小负特征值需按相对阈值判断，
不能与真实不定混为一谈。

## 8. 三角化与真值隔离

未三角化点不再执行 `lmk->position=lmk_map.at(id)`。当前流程为多视图射线初值、带相对
clone 协方差的鲁棒重投影优化、质量门限和初始 landmark 协方差。失败点等新增关键帧观测
后重试，成功点才进入 Schur 后验。

详细公式、失败原因、复杂度和报告字段见 [TRIANGULATION.md](TRIANGULATION.md)。

## 9. 相关文档

- [SIMULATION_SCENARIOS.md](SIMULATION_SCENARIOS.md)：四类仿真和统一噪声模型
- [TRIANGULATION.md](TRIANGULATION.md)：多视图三角化与初始协方差
- [OPT_SCHUR_PATH.md](OPT_SCHUR_PATH.md)：Schur 性能优化
- [OPT_LDLT.md](OPT_LDLT.md)：Hpp/Hll LDLT 开关与验证
- [HPP_NULLSPACE.md](HPP_NULLSPACE.md)：Hpp 结构性零空间
- [HLL_STRUCTURE.md](HLL_STRUCTURE.md)：landmark 深度弱方向
