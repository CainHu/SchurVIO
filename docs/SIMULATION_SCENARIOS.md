# 仿真场景与统一噪声配置

分析工具统一使用 Schur 视觉后验，并提供四个互补场景：

| 场景 | 轨迹 | 相机视线 | 主要用途 |
|---|---|---|---|
| [Circle-out](SIM_CIRCLE_OUT.md) | 5 m 匀速圆周 | 主要朝外 | 常规转弯、短 track、频繁三角化 |
| [Circle-in](SIM_CIRCLE_IN.md) | 10 m 匀速圆周 | 朝圆心 | 长 track、内向几何、重复观测 |
| [Helix-3D](SIM_HELIX_3D.md) | 三维起伏圆周 | 朝中心并滚转 | 完整三轴激励、最大误差检查 |
| [Stop-go](SIM_STOP_GO.md) | 往返与静止交替 | 朝走廊前方 | 弱激励、低视差、鲁棒性 |

## 统一传感器参数

| 参数 | 数值 |
|---|---:|
| IMU / 相机频率 | 200 Hz / 20 Hz |
| 加速度白噪声密度 | 0.02 m/s²/√Hz |
| 角速度白噪声密度 | 0.002 rad/s/√Hz |
| 加速度计偏置随机游走 | 0.0005 |
| 陀螺仪偏置随机游走 | 0.0001 |
| 图像噪声 | 1 pixel |
| 重力 | `(0,0,9.81) m/s²` |

连续时间白噪声密度 `sigma_c` 在采样周期 `dt` 下离散为
`sigma_sample=sigma_c/sqrt(dt)`；偏置随机游走增量标准差为
`sigma_bias=sigma_rw sqrt(dt)`。旧实现把白噪声除以 `sqrt(rate)`，会低估 `rate` 倍的
离散方差，现已统一修正。

特征几何、偏置随机游走、IMU 白噪声和图像噪声使用四个相互独立的固定随机数流。因此改变
特征点数量不会改变 IMU 噪声，参数扫描的每一行也使用完全相同的输入随机序列，便于做
可复现的 A/B 对比。

仿真重力真值与 ESKF 初值完全一致，所以默认 `ESTIMATE_GRAVITY=false`。这不是对真实设备
的通用建议：真实数据需要可靠的静止初始化，若重力方向或初始姿态仍有误差，应恢复重力
估计并设计有充分激励的数据集。

运行全部场景与参数扫描：

```powershell
tools/run_multi_scenario_analysis.ps1 -Duration 30 -Features 600
```

生成结果写入 `out/*.csv` 和 `out/report.html`。
