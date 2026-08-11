# 视觉残差、鲁棒权重与噪声语义

本文解释当前 Schur 路径中三个经常被混用的量：相机单帧归一化观测方差、
历史 **uv_var** 信息密度，以及鲁棒核产生的权重。默认调度已经是一次性
Hybrid MSCKF，因此 **uv_var=1e-2 并不是默认视觉更新实际使用的像素方差**。

## 1. 从像素噪声到归一化像平面

仿真器先生成像素量测

\[
z_{pix}=
\begin{bmatrix}u_{pix}&v_{pix}\end{bmatrix}^{T}
=
\begin{bmatrix}
f_xx/z+c_x\\
f_yy/z+c_y
\end{bmatrix}
+n_{pix}.
\]

进入 VIO 前转换成归一化坐标

\[
z=
\begin{bmatrix}
(u_{pix}-c_x)/f_x\\
(v_{pix}-c_y)/f_y
\end{bmatrix}.
\]

若像素噪声各向同性且标准差为 \(\sigma_{pix}\)，则

\[
R_{norm}=
\begin{bmatrix}
\sigma_{pix}^2/f_x^2&0\\
0&\sigma_{pix}^2/f_y^2
\end{bmatrix}.
\]

当前仿真焦距在两个方向相同，所以用单个

\[
\sigma_{uv}=\text{triangulation\_uv\_std}
\approx\frac{1}{184.75}\approx0.0054
\]

表示，单轴方差约为 \(2.9\times10^{-5}\)。这是真实单帧样本的归一化观测标准差，
同时用于三角化、Huber 门限和默认一次性 MSCKF 的量测方差。

真实相机若 \(f_x\ne f_y\)，更严格的实现应保留二维各向异性协方差；当前标量近似相当于
假设两个方向相同。

## 2. 重投影残差和雅可比

世界点在第 \(j\) 个 clone 的相机坐标为

\[
p_{c,j}=R_{ic}^{T}
\left[R_{wi,j}^{T}(p_f-p_{wi,j})-t_{ic}\right]
=
\begin{bmatrix}x&y&z\end{bmatrix}^{T}.
\]

投影函数与残差定义为

\[
\pi(p_c)=
\begin{bmatrix}x/z&y/z\end{bmatrix}^{T},
\qquad
r_j=z_j-\pi(p_{c,j}).
\]

投影雅可比是

\[
J_\pi=
\begin{bmatrix}
1/z&0&-x/z^2\\
0&1/z&-y/z^2
\end{bmatrix}.
\]

对世界 XYZ 点，

\[
J_l=J_\pi R_{ic}^{T}R_{wi}^{T}.
\]

当前左乘姿态误差下，clone 位姿雅可比为

\[
J_{pose}=
\begin{bmatrix}
J_l[p_f-p_{wi}]_\times&-J_l
\end{bmatrix}.
\]

锚定参数化还会出现锚帧位姿雅可比；外参雅可比保留在源码中，但
**ESTIMATE_EXTRINSIC=false** 时不进入状态正规方程。

## 3. 三层鲁棒保护

普通轨迹进入正规方程前依次经过：

1. 有限值和正深度检查：\(z>0.05\,\mathrm m\)；
2. 硬重投影门限：\(\|r\|\le0.1\)；
3. Huber 降权，门限
   \(\delta=3\sigma_{uv}\)。

Huber 信息权重为

\[
w(r)=
\begin{cases}
1,&\|r\|\le\delta,\\
\delta/\|r\|,&\|r\|>\delta.
\end{cases}
\]

代码把同一个 \(w\) 同时累加到

\[
H_{pp},H_{pl},H_{ll},g_p,g_l,
\]

例如

\[
H_{pp}\mathrel{+}=wJ_p^TJ_p,\qquad
g_p\mathrel{+}=wJ_p^Tr.
\]

这等价于把该观测的协方差放大为

\[
R_{eff}=R/w.
\]

若只给 Hessian 或只给梯度乘权重，Schur 系统会失去同一代价函数来源；当前实现避免了
这种不一致。

硬门限和 Huber 的职责不同：

- 硬门限拒绝明显错误关联、负深度或线性化完全失效的样本；
- Huber 保留中等残差，但降低其信息；
- 量测方差描述门内正常样本的统计不确定度。

不应通过无限放宽硬门限来“减少拒绝率”，也不应靠极大方差掩盖错误数据关联。

## 4. 为什么正规方程里没有显式除以方差

当前普通 Schur 路径先累加无量纲的鲁棒正规方程

\[
H=J^TWJ,\qquad g=J^TWr,
\]

其中 \(W\) 只含 Huber 权重；公共方差 \(\sigma_v^2\) 在把 Schur 信息方向转换成
伪量测时才加入。

若

\[
H_s=BDB^T,
\]

则第 \(i\) 个标量伪量测使用

\[
z_i=\tilde g_i/d_i,\qquad
h_i=b_i,\qquad
R_i=\sigma_v^2/d_i.
\]

这与一开始使用

\[
\frac{1}{\sigma_v^2}J^TWJ
\]

完全等价，前提是同一批观测共享公共方差。若以后引入逐点不同的跟踪协方差，
就必须把各自的 \(R_j^{-1}\) 在累加阶段白化，不能再只在末尾乘一个公共标量。

## 5. 三类调度器的噪声语义

### 5.1 默认 MSCKF / RD-VIO：一次性样本方差

一次性轨迹中每个像素样本最多进入一个后验，因此

\[
\sigma_v^2
=\sigma_{uv}^2
\max(s_{msckf},1),
\]

其中 **msckf_visual_noise_scale** 默认 1。这里不除以相机周期 \(\Delta t\)：
同一幅图像的噪声不会因为相机频率提高而自动变小；除以 \(\Delta t\) 会凭空增加信息。

所以默认 Hybrid MSCKF 下：

- **triangulation_uv_std** 决定普通一次性视觉方差；
- **msckf_visual_noise_scale** 是保守倍率；
- **uv_var** 不参与这条普通轨迹后验。

### 5.2 SchurVINS 活跃轨迹模式

该模式仍使用

\[
\sigma_v^2=\sigma_{uv}^2,
\]

但会重新求解活跃窗口轨迹。它是论文/历史调度对照，观测相关性语义不如一次性模式清晰，
结果不能只凭较低 RMSE 判定更好。

### 5.3 Legacy / VINS-Mono 对照：历史信息密度

为保留旧行为，这两类重复窗口对照使用

\[
\sigma_v^2=\frac{\text{uv\_var}}{\Delta t}.
\]

此时 **uv_var** 是历史连续时间信息密度式调参量，不是像素方差。
早期扫描得到的 \(10^{-2}\) 只适用于这条重复窗口语义，不能搬到一次性 MSCKF 后再宣称
“像素标准差为 0.1”。

~~~mermaid
flowchart TD
    S["选择视觉调度器"] --> O{"一次性消费?"}
    O -- "MSCKF / RD-VIO" --> A["sigma_v^2 = sigma_uv^2 * scale"]
    O -- 否 --> V{"SchurVINS?"}
    V -- 是 --> B["sigma_v^2 = sigma_uv^2"]
    V -- 否 --> C["sigma_v^2 = uv_var / dt"]
    A --> P["Schur 信息方向 -> 标量伪量测"]
    B --> P
    C --> P
~~~

因此多场景脚本在一次性模式下跳过 **uv_var** 扫描是正确行为；否则只会生成输入标签
不同、算法实际方差完全相同的重复行。

## 6. 持久点观测为什么使用 64 倍方差

已晋升持久点会跨多个关键帧重复观测。它们保留完整 \(P_{xL}/P_{LL}\)，因此不属于把旧点
当独立地图量测的错误做法；但实际系统仍存在：

- FEJ 长期线性化误差；
- 前端跟踪的时间相关性；
- 未建模的点运动和外参误差；
- 晋升时条件初始化近似。

当前使用

\[
R_{persistent}
=64\,\sigma_{uv}^2\,I/w.
\]

二维创新协方差和 NIS 为

\[
S=HPH^T+R_{persistent},\qquad
\operatorname{NIS}=r^TS^{-1}r.
\]

门限 9.21 对应 2 自由度卡方分布的约 99% 分位。64 倍是 100 秒多场景扫描得到的保守
工程值，不是普适常数；更换真实前端、相机或持续点模型后应重新标定。

## 7. 无深度旋转约束的噪声

一对 bearing 都含图像噪声，所以理想差分残差的方差首先近似为

\[
R_{rot}\approx2\sigma_{uv}^2I.
\]

默认 Hybrid MSCKF 还用 **depth_free_rotation_information_scale=0.02** 降低其信息，
吸收小平移、R/N 误分类和切平面近似误差。RD-VIO 显式模式使用自己的完整倍率。
详细推导见 [无深度纯旋转约束](DEPTH_FREE_ROTATION_CONSTRAINT.md)。

## 8. NIS、NEES 与报告解释

普通 Schur 路径把有效信息方向变成互不相关的一维伪量测，日志中的平均 NIS 是

\[
\overline{\operatorname{NIS}}
=\frac1r\sum_{i=1}^{r}\frac{e_i^2}{S_i}.
\]

因此理想均值更接近 1，而不是名义状态维数或像素数。它只覆盖实际保留的 \(r\) 个方向；
被 Hpp 阈值删除的零空间不计入自由度。

NEES 则依赖选取的状态误差块和相应协方差。单目 VIO 存在 gauge，未经对齐的绝对位置
误差与局部协方差并不总能直接按满秩卡方分布解释，所以报告同时提供 raw RMSE、
SE(3) 对齐 ATE 和固定时间间隔 RPE。

当前汇总还有两个容易忽略的实现细节：`mean_nis` 对“每次更新的方向均值”等权，而不是
按全部有效方向数加权；`mean_nees` 是左乘世界系 Q/P/V 9 维联合 NEES 除以 9 后的时间
平均。姿态误差坐标、gauge 对齐和协方差谱检查的完整定义见
[EVALUATION_METRICS_AND_GAUGE_ALIGNMENT.md](EVALUATION_METRICS_AND_GAUGE_ALIGNMENT.md)。

还要注意：

- 硬门限会截断创新分布，门后 NIS 不是未经选择的原始高斯样本；
- Huber 会改变等效似然，NIS 主要是工程一致性指标；
- 一次性模式必须同时检查 **reused=0** 和 **blocked=0**，否则再漂亮的 NIS 也可能来自
  重复计数。

## 9. 与过程噪声的耦合

Kalman 增益取决于先验和量测两端：

\[
K=PH^T(HPH^T+R)^{-1}.
\]

增大过程噪声会让 \(P^-\) 增大，通常增强视觉修正；增大量测噪声 \(R\) 会减弱视觉修正。
两者不能互相替代：

- 用大过程噪声补偿过小量测噪声，会让 IMU 间传播过散；
- 用大视觉噪声掩盖错误传播，会让系统接近纯惯导；
- 用 landmark 独立过程噪声无法补出缺失的 \(P_{xL}\)。

正确顺序是先保证坐标、传播、观测生命周期和相关性正确，再调 \(Q/R\)。

## 10. 当前推荐参数与调参顺序

默认仿真配置：

| 参数 | 默认值 | 实际作用范围 |
|---|---:|---|
| **triangulation_uv_std** | 0.0054 | 三角化、Huber、一次性 MSCKF |
| **msckf_visual_noise_scale** | 1 | MSCKF/RD-VIO 普通轨迹 |
| **visual_huber_delta_sigma** | 3 | 普通与持久重投影 |
| **visual_hard_reprojection_limit** | 0.1 | 硬门限 |
| **persistent_measurement_noise_scale** | 64 | 已晋升持久点 |
| **uv_var** | 0.01 | 仅历史重复窗口模式 |
| **proc_noise_scale** | 1 | 全部 IMU 状态过程密度 |

推荐调参顺序：

1. 用真实标定确定像素噪声和焦距，换算 \(\sigma_{uv}\)；
2. 验证一次性量测计数与协方差传播；
3. 固定普通轨迹方差，扫描硬门限和 Huber 降权率；
4. 单独扫描持久点方差倍率和二维 NIS；
5. 根据 IMU Allan 方差设置过程噪声，再做小范围倍率敏感度；
6. 至少运行 Circle-out、Circle-in、Helix-3D、Stop-go 和 100 秒长时回归。

相关文档：

- [ESKF 状态传播与增广](ESKF_STATE_PROPAGATION_AND_AUGMENTATION.md)
- [视觉后验分析](ANALYSIS_REPORT.md)
- [三角化](TRIANGULATION.md)
- [视觉调度与观测生命周期](VISUAL_UPDATE_SCHEDULING.md)
- [混合 MSCKF](HYBRID_MSCKF.md)
