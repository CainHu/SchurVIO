# 从 137bfeada23c123e8 重实现当前 SchurVIO

本文面向这样的场景：检出提交 **137bfeada23c123e8**，不直接复制后续代码，而是按依赖
关系逐步把算法重新实现到当前 Windows 工程的算法端点 **ebab4a04e487ca4a934106151240a99a159de6d0**。
本轮后续提交只补文档和注释，不改变该算法端点。

范围严格限定为 `E:\GithubProject\SchurVIO` 的 Windows 提交链；后续迁移到 WSL 后产生的
修改不在本文重建范围内。

目标不是逐行复刻提交，而是理解每个点子为了解决什么问题、它依赖哪些数学不变量、怎样
证明实现没有悄悄引入重复信息或不一致协方差。文中尽量不给大段代码；真正动手时，再按
每阶段列出的文件职责实现。

## 1. 起点和终点分别是什么

起点 **137bfea** 已经具备：

- IMU 名义状态与 ESKF 协方差传播；
- 固定 30 槽位的位姿 clone 窗口；
- QR 与 Schur 两条视觉后验路径；
- Hpp 默认 LDLT 并可切换特征分解；旧 Independent EKF 的 Hll 后处理也可切换分解；
- 序贯 Joseph 等价协方差更新；
- 初版 HTML 分析报告；
- 尚未替换的真值 landmark 初始化；
- 持续滑窗式视觉批次和独立 landmark EKF 后处理。

终点相对起点新增的核心能力是：

1. 真值与在线算法隔离的多视图三角化；
2. 确定性多场景仿真、真实噪声离散化和严格消融；
3. FEJ 可观性约束与一致有效子空间；
4. 四种 3-DOF landmark 参数化；
5. Legacy、SchurVINS、MSCKF、VINS-Mono、RD-VIO 调度；
6. 默认一次性 MSCKF 与 KeyframeOnly 20-clone 窗口；
7. 低视差延迟、无深度旋转约束和轨迹摘要；
8. 最多 20 个持久 landmark 的完整联合协方差；
9. 实验性冗余感知关键帧删除；
10. 拆分后的源码、报告和数学专题。

算法依赖关系不是提交历史的简单直线：

~~~mermaid
flowchart TD
    B["137bfea 基线"] --> T["真实三角化"]
    T --> N["真实噪声 + 多场景"]
    N --> A["严格消融基础设施"]
    A --> F["FEJ 可观性"]
    F --> L["Landmark 后续修正"]
    L --> S["统一有效子空间"]
    S --> Q["观测生命周期与多调度"]
    Q --> P["参数化 + 帧策略"]
    P --> R["100 秒饥饿修复"]
    R --> H["Hybrid MSCKF"]
    H --> D["低视差旋转因子/轨迹归档"]
    H --> J["持久点联合状态"]
    D --> K["冗余感知删帧实验"]
    J --> K
~~~

## 2. 开始前先冻结四类不变量

### 2.1 坐标不变量

全工程统一使用

\[
R_{wi}\colon i\rightarrow w,\qquad
R_{ic}\colon c\rightarrow i,
\]

\[
p_c=R_{ic}^{T}\left[R_{wi}^{T}(p_f-p_{wi})-t_{ic}\right].
\]

世界重力为 \([0,0,+9.81]^T\)。先写一个单点投影检查：同一真值位姿和世界点经仿真器与
后端投影应得到相同归一化坐标。这个测试不通过时，不要开始调噪声。

### 2.2 误差状态不变量

默认排列是

\[
\delta x_I=
[\delta\theta,\delta p,\delta v,\delta b_g,\delta b_a],
\]

姿态采用

\[
R^{true}=\operatorname{Exp}(\delta\theta)R
\]

的左乘误差。clone 为 \([\delta\theta_C,\delta p_C]\)。状态布局和所有雅可比必须共享
同一误差定义。

### 2.3 联合协方差不变量

IMU 传播不能只更新 \(P_{II}\)：

\[
P_{II}'=AP_{II}A^T+Q,\qquad
P_{Ir}'=AP_{Ir},\qquad
P_{rr}'=P_{rr}.
\]

clone 增广必须复制与全部旧状态的相关性：

\[
P_{C\chi}=J_CP_{I\chi},\qquad
P_{CC}=J_CP_{II}J_C^T.
\]

后续加入持久点后，\(r\) 还包含 landmark，不能只传播 clone cross block。

### 2.4 观测生命周期不变量

默认一次性 MSCKF 要求

\[
\text{同一个像素样本进入导航后验的次数}\le1.
\]

轨迹可以在一个批次中含多帧样本；“一次性”是指整条轨迹在 lost、触窗边界或达到长度
上限时结算一次，不是每条轨迹只使用一个像素。

从第一阶段起就保留三个计数器：

- new：第一次进入后验的像素数；
- reused：第二次进入线性化的像素数；
- blocked：一次性模式中被防御性拦截的重复像素数。

最终默认要求 reused=0 且 blocked=0。

## 3. 第 0 阶段：冻结 137bfea 基线

对应起点：**137bfeada23c123e8**

先保存以下基线产物：

- Release 构建和一条短仿真轨迹；
- 轨迹 CSV、更新 CSV、landmark CSV；
- 总耗时及 QR/Schur 分阶段耗时；
- 协方差对称误差、最小对角、负特征值计数；
- GT/EST 末帧对比。

基线的视觉点位置仍可能来自真值，因此它只能验证“后验求解在理想点位置下能否工作”，
不能作为完整 VIO 精度基准。

建议从独立分支开始：

~~~powershell
git switch --detach 137bfeada23c123e8
git switch -c codex/rebuild-from-137bfea
~~~

每一阶段完成后单独提交。不要把三角化、噪声重标定和调度改动放在一个提交里，否则发散时
无法判断是几何、统计还是生命周期错误。

## 4. 第 1 阶段：用真实多视图三角化替换真值

对应提交：**b7e2e7ed24b018c4fe2637ccacef88ffcf7e4748**

### 4.1 要解决的问题

起点把

\[
p_f\leftarrow p_f^{GT}
\]

写入在线地图。这会绕过 VIO 最脆弱的深度初始化，无法验证 Hll、Schur 消元、低视差和
错误点门控。

### 4.2 射线最小二乘初值

第 \(j\) 帧相机中心和世界射线为

\[
c_j=p_{wi,j}+R_{wi,j}t_{ic},
\qquad
d_j=R_{wi,j}R_{ic}\bar b_j,
\]

其中 \(\bar b_j\) 是归一化单位 bearing。理想世界点满足

\[
(I-d_jd_j^T)(p_f-c_j)=0.
\]

累加得到

\[
A=\sum_j(I-d_jd_j^T),\qquad
b=\sum_j(I-d_jd_j^T)c_j,
\qquad
p_f=A^\dagger b.
\]

先检查最大视差、\(A\) 的条件数和有限值；不要对病态 \(A\) 直接求逆。

### 4.3 鲁棒重投影精化

以射线解为初值，对

\[
\min_{p_f}\sum_j\rho\left(
\|z_j-\pi(R_{wc,j}^T(p_f-c_j))\|^2
\right)
\]

做少量 Gauss–Newton。每轮要求：

- 所有有效观测正深度；
- Huber 权重同时进入 Hessian 和梯度；
- 候选步使重投影代价下降；
- 步长、条件数和最大不确定度有上界。

### 4.4 初始协方差

只用像素信息的近似是

\[
P_f\approx
\left(\sum_jJ_{f,j}^TR_j^{-1}J_{f,j}\right)^\dagger.
\]

当前实现还把锚 clone 的位姿协方差传播到点位置，形成保守近似。完整严格模型会涉及所有
clone 相关性，代价更高；第一版先保证协方差随视差下降、弱深度方向变大。

### 4.5 失败不是异常

至少区分：

- 观测不足；
- 视差不足；
- 条件数过大；
- 负深度；
- 重投影误差过大；
- 位置不确定度过大。

失败轨迹应等待新观测后重试，不能写入一个“看似有效”的默认世界点。

### 4.6 真值隔离

GT 只允许进入离线日志：

\[
e_f=p_f^{est}-p_f^{GT},\qquad
\operatorname{NEES}_f=e_f^TP_f^\dagger e_f.
\]

生产初始化函数不能读取真值 map。保留显式的 Oracle/GT 分析模式，但默认必须是真实
三角化。

验收重点：三角化成功率、失败原因分布、重投影 RMSE、位置误差和耗时，而不是只看轨迹
末帧。

必读：[多视图三角化](TRIANGULATION.md)。

## 5. 第 2 阶段：建立真实噪声和多场景验证

对应提交：**7b9861f859d8cb9a0ff3761bd8ae53d648f028a3**

### 5.1 修正仿真噪声离散化

连续白噪声密度 \(\sigma_c\) 在采样周期 \(\Delta t\) 下应生成

\[
\sigma_{sample}=\frac{\sigma_c}{\sqrt{\Delta t}},
\]

bias random walk 每步为

\[
\Delta b=\sigma_{rw}\sqrt{\Delta t}\,\epsilon.
\]

旧式 \(\sigma_c\sqrt{\Delta t}\) 会把白噪声方差低估约采样率平方量级。特征几何、IMU
白噪声、bias random walk 和图像噪声必须使用独立固定随机流，使改变特征数不会改变
IMU 输入。

### 5.2 增加互补场景

至少同时保留：

- Circle-out：短 track、外向视野；
- Circle-in：长 track、内向几何；
- Helix-3D：三轴激励；
- Stop-go：静止、弱视差和起停；
- 后续 Rotation-translation：纯旋转与平移交替。

只在 Circle-in 上工作不能证明算法鲁棒，因为它天然提供长轨迹和较稳定视差。

### 5.3 固定已知重力

仿真真值与 ESKF 初始值都是

\[
g=[0,0,9.81]^T.
\]

因此默认关闭重力估计，去掉与姿态和加速度计 bias 的弱可观耦合。真实设备不能机械照搬：
应先做静止初始化，或在重力未知时恢复估计。

### 5.4 区分 Q 和 R

ESKF 当前使用状态空间过程密度近似

\[
Q_d=\operatorname{diag}(q_x)\Delta t.
\]

视觉普通样本的归一化方差来自像素噪声和焦距。不要用增大 Q 去掩盖错误 R，也不要用大 R
掩盖遗漏的 cross covariance。

验收重点：四场景都有限、neg_cov=0、噪声随机流可复现，报告能同时显示轨迹、先验/后验、
NIS/NEES、三角化和耗时。

必读：[仿真场景](SIMULATION_SCENARIOS.md)、
[视觉残差与噪声](VISUAL_RESIDUAL_NOISE_MODEL.md)。

## 6. 第 3 阶段：先建严格消融，再继续改算法

对应提交：**2f8ea8ec9616b8fdbacc7102f3f637fc3be2450e**

这一阶段主要是实验基础设施。每组实验只改一个变量，并复用完全相同输入。至少比较：

- 真值初始化与真实三角化；
- 新旧 IMU 白噪声离散化；
- 开/关 bias random walk；
- 开/关 landmark 后续修正；
- 不同过程噪声倍率；
- 后续的 FEJ、硬投影、调度、参数化和持久点。

指标不能只有 raw 位置 RMSE：

\[
\operatorname{ATE}_{SE(3)},\quad
\operatorname{RPE}_{1s},\quad
\operatorname{RMSE}_{p,v,R,b_g,b_a},\quad
\operatorname{NIS},\quad
\operatorname{NEES}.
\]

加入“视觉后验是否把误差拉近 GT”的逐次统计：

\[
\Delta e_p=
\|p_{prior}-p_{GT}\|-\|p_{post}-p_{GT}\|.
\]

单次改善率不要求 100%；它用于发现系统性反向修正或孤立大尖峰。

必读：[严格消融](ABLATION_STUDY.md)、
[视觉后验分析](ANALYSIS_REPORT.md)。

## 7. 第 4 阶段：加入 FEJ 可观性约束

对应提交：**b725fa29e2756caa35fa47d01c244baf09b1ccb3**

### 7.1 为什么需要

理想单目 VIO 在完整惯性先验下仍有四维 gauge：

\[
\mathcal N=
\{\text{全局平移 3 维},\text{绕重力方向偏航 1 维}\}.
\]

反复在更新后的名义状态重线性化，会让不同时间的雅可比零空间不一致，滤波器可能虚构
对 gauge 的信息并过度收缩协方差。

### 7.2 FEJ 做法

clone 创建时冻结

\[
R_{wi,j}^{FEJ},\qquad p_{wi,j}^{FEJ}.
\]

残差仍在当前名义状态计算，位姿雅可比在 FEJ 位姿计算。这样既保留当前残差，又稳定
不可观子空间。

对任一 clone，全局平移基在位置块为 \(I_3\)。以参考位置 \(p_0\) 和单位重力轴
\(\hat g\) 构造偏航方向：

\[
n_{\theta,j}=\hat g,\qquad
n_{p,j}=-[p_j^{FEJ}-p_0]_\times\hat g.
\]

### 7.3 不要急着默认硬投影

理论投影为

\[
\Pi=I-N(N^TN)^{-1}N^T,\qquad
H\leftarrow\Pi^TH\Pi,\quad g\leftarrow\Pi^Tg.
\]

但只投影位姿块、只投影梯度，或让 Hll/Hpp 使用不同秩阈值，都会破坏 Schur 系统同域性。
当前默认只开 FEJ，硬投影保留实验开关。

验收重点：零空间泄漏、轨迹、NEES/NIS 和协方差，而不只是投影后
\(\|HN\|\) 是否变小。

必读：[可观性约束](OBSERVABILITY_CONSTRAINT.md)。

## 8. 第 5 阶段：重做 landmark 后续修正

对应提交：**f7e9d548913ca79423263bb9ac450ce09507ff9e**

### 8.1 为什么旧 Independent EKF 看似更好

旧方法反复用滑窗中的相同观测更新点协方差，却不保存点与导航状态的 \(P_{xL}\)：

\[
P_{joint}\approx
\begin{bmatrix}
P_{xx}&0\\
0&P_{LL}
\end{bmatrix}.
\]

地图点误差可能较低，但协方差会过度收缩；它也可能把同一批像素通过“状态更新”和“独立
点更新”重复解释。给 \(P_{LL}\) 加过程噪声只能缓解收缩，不能凭空恢复缺失的 \(P_{xL}\)。

### 8.2 保留的对照策略

- Fixed：初始化后不再改点；
- Retriangulate：出现新关键帧观测时，用全部当前轨迹重新三角化；
- Schur back-substitution：

\[
\delta l=H_{ll}^{\dagger}
\left(g_l-H_{lp}\delta x\right);
\]

- Independent EKF 及固定/自适应膨胀：仅作消融。

重三角化和回代都要求重投影代价不升，并使用有限步长或线搜索。一次性 MSCKF 下临时点
完成后即删除，所以默认不会走旧的长期独立点更新。

必读：[Landmark 后续修正](LANDMARK_UPDATE_STRATEGIES.md)。

## 9. 第 6 阶段：统一 Hll、Schur 与 Hpp 的有效子空间

对应提交：**8df873a7bea626a25f16eaf1f86ca0be809927a5**

### 9.1 Hll 伪逆必须和梯度同域

对单点

\[
H_{ll}=V\Lambda V^T,
\]

保留

\[
\lambda_i>\tau_l\lambda_{max}
\]

的方向，构造

\[
H_{ll}^{\dagger}
=V\operatorname{diag}
\left(\mathbf1_i/\lambda_i\right)V^T.
\]

Hll 和 \(g_l\) 必须使用同一组方向；否则会出现“信息矩阵已删除某方向，梯度仍在该方向
推动状态”的非物理系统。

### 9.2 Schur 消元

\[
H_s=H_{pp}-H_{pl}H_{ll}^{\dagger}H_{lp},
\qquad
g_s=g_p-H_{pl}H_{ll}^{\dagger}g_l.
\]

### 9.3 先验白化硬投影实验

姿态以 rad、位置以 m 表达，直接欧氏投影受单位尺度影响。用先验平方根 \(P=LL^T\)
白化后再正交化零空间：

\[
\bar H=L^TH_sL,\qquad
\bar g=L^Tg_s,\qquad
\bar N=L^{-1}N.
\]

投影后的 H 和 g 只允许在同一次谱分解保留的方向进入后验，不能再次用另一阈值判秩。
严格实验表明硬投影仍会恶化当前场景，所以默认保持关闭；这个阶段的价值是把失败原因
定位清楚，而不是强行启用理论上漂亮的开关。

### 9.4 影子地图

增加与导航解耦的地图后处理器，用于比较 landmark 策略。它不保存 \(P_{xL}\)，因此绝不
允许反馈导航；否则又回到假独立地图问题。

必读：[一致有效子空间](CONSISTENT_SUBSPACE_AND_LANDMARK_COVARIANCE.md)、
[Hll 结构](HLL_STRUCTURE.md)。

## 10. 第 7 阶段：把“选哪些帧”和“何时消费量测”拆开

对应提交：**8a652f9ed33f340353d34053167d1657bfabb89b**

### 10.1 两个正交决策

帧策略回答：

\[
\text{哪些图像创建 clone，窗口满时删哪一帧？}
\]

视觉调度回答：

\[
\text{一条轨迹何时进入后验，能否再次进入？}
\]

不要用一个枚举同时编码两者。最终工程分别使用
**VisualUpdateScheduler** 和 **FrameSelectionPolicy**。

### 10.2 一次性 MSCKF 状态机

~~~mermaid
stateDiagram-v2
    [*] --> Tracking
    Tracking --> Ready: lost / 触窗边界 / 达长度上限
    Tracking --> Deferred: 低视差且仍可见
    Deferred --> Tracking: 新观测到达
    Ready --> Triangulated: 三角化成功
    Ready --> RotationOnly: 深度失败但旋转约束可用
    Triangulated --> Consumed: Schur 后验一次
    RotationOnly --> Consumed: bearing 后验一次
    Ready --> Dropped: 所有几何检查失败
    Consumed --> [*]
    Dropped --> [*]
~~~

必须先规划待删除 clone，再消费触及这些 clone 的轨迹，最后才真正删帧。若先删帧，
Observation/Feature 引用会消失，轨迹尚未形成后验就被破坏。

### 10.3 三自由度 landmark 参数化

所有模式仍保存世界 XYZ，只改变局部增量 \(\delta\lambda\)：

- World XYZ：\(p_f=\lambda\)；
- Anchored XYZ：
  \(p_f=p_{wa}+R_{wa}\lambda\)；
- Anchored inverse depth 3D：

\[
p_{c,a}=
\begin{bmatrix}\alpha/\rho&\beta/\rho&1/\rho\end{bmatrix}^{T};
\]

- Anchored log depth 3D：

\[
p_{c,a}=
\begin{bmatrix}\alpha e^\eta&\beta e^\eta&e^\eta\end{bmatrix}^{T}.
\]

锚定参数化改变点时也依赖锚帧位姿，必须加入锚帧雅可比；只换 Hll 坐标而漏掉锚帧项会
得到错误 gauge。

### 10.4 RD-VIO 风格 R/N 调度

用旋转补偿后的公共 bearing 视差判定：

\[
\theta_k=
\arccos\left(
b_{k}^{T}R_{c_kc_{k-1}}b_{k-1}
\right).
\]

小视差为 R 帧，足够平移为 N 帧。实现 RR、NN、RN、NR 转移、R 子窗压缩和可选零平移
约束，但明确它只是 Schur-ESKF 适配，不等于论文中的分层非线性 BA。

### 10.5 本阶段的已知陷阱

第一版一次性 MSCKF 仍沿用 10 clone 和 8° 三角化门限。20 Hz 下轨迹约 0.5 s 就触边，
多数 Circle-out 点还没积累足够基线便永久删除。短仿真可能看不出，100 秒会视觉饥饿并
发散。因此本阶段不能作为长期终点。

必读：[视觉调度](VISUAL_UPDATE_SCHEDULING.md)、
[Landmark 参数化](LANDMARK_PARAMETERIZATION.md)、
[RD-VIO 调度](RDVIO_SCHEDULING.md)。

## 11. 第 8 阶段：修复 100 秒视觉饥饿并拆分源码

对应提交：**32498c400fafdfa5db4cd008e84bebb13d7bede8**

### 11.1 根因不是 Schur 求解器

历史故障组合是：

\[
N_{clone}=10,\qquad
\theta_{tri,min}=8^\circ.
\]

一次性轨迹触窗后必须结算或永久丢弃。短窗口和大视差门限共同导致：

\[
\text{轨迹到期速度}>\text{形成可用深度速度}.
\]

Circle-out 100 秒历史回归中，错误组合只接受 488/20739 个三角化候选，位置 RMSE 达
238.94 m。改为

\[
N_{clone}=20,\qquad
\theta_{tri,min}=2^\circ
\]

后接受 10342/10680 个候选，RMSE 降到 0.64 m 左右；正深度、重投影、条件数和位置方差
门限仍继续过滤坏点。

### 11.2 调度和帧策略彻底解耦

默认 MSCKF 解析为 KeyframeOnly，只对关键帧创建 clone，保留 20 个。SchurVINS、
VINS-Mono 和 RD-VIO 各自选择帧策略，但不改变“一次性还是重复窗口”的后验语义。

VINS-Mono 三角化重试应按“最新观测 FrameID 是否变化”触发，不能只比较观测数量；删除旧帧
和加入新帧可能让数量不变，但几何已经变化。

### 11.3 源码按职责拆分

拆分顺序应在数值算法稳定后进行：

- 外层数据流和注入；
- IMU 传播；
- 三角化；
- 视觉调度、线性化和 Schur；
- 影子地图；
- 帧策略、RD-VIO 调度和约束。

拆分前后要保持矩阵累加顺序，先证明轨迹和统计等价，再进一步重构函数作用域。

必读：[源码布局](SOURCE_LAYOUT.md)。

## 12. 第 9 阶段：实现默认 Hybrid MSCKF

对应提交：**bf319f3afb783beed3dd3531902c84e5ac6f7e81**

纯结构消元 MSCKF 对短轨迹高效，但低视差轨迹容易浪费，且无法长期复用少量高质量点。
混合结构保留普通一次性轨迹，同时只把少量成熟点加入联合状态。

### 12.1 联合状态

\[
\delta\chi=
\begin{bmatrix}
\delta x_M\\
\delta p_{L_0}\\
\vdots\\
\delta p_{L_{m-1}}
\end{bmatrix},
\qquad
P=
\begin{bmatrix}
P_{MM}&P_{ML}\\
P_{LM}&P_{LL}
\end{bmatrix}.
\]

预算默认 \(m\le20\)。只保存 \(P_{LL}\) 而不保存 \(P_{ML}\) 不叫联合滤波。

### 12.2 一帧的真实顺序

~~~mermaid
flowchart TD
    A["当前图像与 clone"] --> B["规划删帧"]
    B --> C["区分持久 ID 与普通轨迹"]
    C --> D["普通轨迹筛选、三角化<br/>旋转候选与延迟计划"]
    D --> E["已有持久点直接联合 EKF"]
    E --> F["普通轨迹线性化并 Schur 消临时点"]
    F --> G["联合 Joseph 更新完整 P"]
    G --> H["用本次正规方程条件初始化新持久点"]
    H --> I["清轨迹，再删 clone"]
~~~

已有持久点像素和普通轨迹像素必须是互斥集合。普通 Schur 因子虽然直接雅可比在持久点列
为零，但 \(P_{LM}\) 会让 Kalman 增益的持久点行非零；这是相关性传播，不是像素重复使用。

### 12.3 低视差延迟

当轨迹因触边请求消费、但当前仍可见且仅因低视差失败时，可以撤销删除，等待未来基线。
需要上限防止轨迹永久占用 Map。真正 lost 或超过上限时，轨迹最终消费、转旋转约束或归档。

### 12.4 无深度旋转约束

纯旋转近似下

\[
b_j\approx R_{c_jc_i}b_i.
\]

在 \(b_j\) 的球面切平面取两个正交向量 \(t_1,t_2\)，构造

\[
r_k=t_k^T(b_j-R_{c_jc_i}b_i),\qquad k=1,2.
\]

它只约束相邻 clone 姿态，不观测平移、深度和尺度。默认用 0.02 信息倍率吸收小平移和
R/N 误判。已经用于旋转残差的像素要标记；以后深度变可观时，只允许剩余新像素进入 Schur。

### 12.5 丢失轨迹摘要

旧 clone 被边缘化后，不能把其固定 pose 快照重新当作当前状态因子。归档只保存首尾 bearing、
相机中心、姿态和质量摘要，用于未来 ID 再出现时更新影子候选；绝不重建导航 H/g。

### 12.6 候选成熟和网格预算

候选至少经过：

1. 当前轨迹几何质量：视差、条件数、重投影、位置方差；
2. 跨独立轨迹的一致性 NIS；
3. 稳定更新次数和质量 EMA；
4. 图像网格配额与总预算。

这避免 20 个持久点都集中在同一小区域。

### 12.7 条件初始化

普通轨迹已经完成一次 Schur 后验。晋升不能再做第二次导航量测更新，而是从同一正规方程
回代点均值：

\[
\delta l=H_{ll}^{-1}(g_l-H_{lx}\delta x_M).
\]

把局部参数变到世界 XYZ 的雅可比记为 \(T\)，则

\[
J_x=-T H_{ll}^{-1}H_{lx},
\]

\[
P_{l\chi}=J_xP_{M\chi},
\]

\[
P_{ll}=J_xP_{MM}J_x^T+
T(\sigma_v^2H_{ll}^{-1})T^T.
\]

旧协方差左上块保持不变，只追加新行列，所以不会第二次收紧导航状态。

### 12.8 已有持久点更新

量测雅可比同时写入当前 clone 和对应持久点：

\[
H=[0,\ldots,H_C,\ldots,H_L,\ldots].
\]

用完整联合 \(P\) 计算

\[
S=HPH^T+R,\qquad
K=PH^TS^{-1},
\]

再做 Joseph 更新。长期重复观测使用 64 倍普通像素方差和二维卡方门控，降低未建模相关性
造成的过度自信。

必读：[混合 MSCKF](HYBRID_MSCKF.md)、
[影子与持久点边界](SHADOW_LANDMARKS.md)、
[无深度旋转约束](DEPTH_FREE_ROTATION_CONSTRAINT.md)。

## 13. 第 10 阶段：补全可维护性文档和源码注释

对应提交：

- **a8839d15786836a3207a61dd76dcaeb8f4de30db**
- **ca58c0ed6739103cd80a3f8ffe2d4d38a94ec0ac**
- **c2d948999b5c3601a374b8f9f937be5f35e60d3d**
- **2615ba6321a97ce398cf7b1fac9faf2dc73e9d13**
- **419edb603197bffb79f42f2a0c4e0856bbfc1147**

这些提交主要补数学、状态机、观测分流边界和中文注释。重实现时不要把它们当成“无关文档”：
它们记录了 Hybrid MSCKF 最容易写错的五个不变量：

- 已有持久点像素与普通 MSCKF 集合互斥；
- 晋升是条件初始化，不是第二次量测；
- 低视差旋转像素和后续深度像素不能重复；
- 归档 pose 不允许重新进入导航后验；
- 持久点与普通轨迹的更新顺序必须使用更新后的同一完整协方差。

## 14. 第 11 阶段：实验性冗余感知关键帧删除

对应提交：**ebab4a04e487ca4a934106151240a99a159de6d0**

默认 KeyframeOnly 始终删除最老关键帧。实验策略只改变“删哪一帧”，不改变只增广关键帧、
20 clone 预算和一次性后验。

对候选帧构造可解释评分：

\[
s=
w_r\rho_{redundant}
+w_l\rho_{lowSupport}
+w_a\rho_{age}
-w_p\rho_{lowParallax}
-w_u\rho_{unique}
-w_d\rho_{parallaxLoss}
-w_t\rho_{gap}.
\]

其中“低视差风险”和“独特几何”是保护项，冗余、低支持和年龄是删除奖励。最新两帧保护，
候选只取较老半窗；只有明显优于最老帧才允许删除中间帧，否则回退 FIFO。

100 秒严格 A/B 中它与默认最老删除结果完全相同，因为候选时最老帧已没有活跃轨迹引用。
因此该策略保留为实验项，默认仍是 KeyframeOnly；不要为了“算法更复杂”强行设为默认。

必读：[冗余感知关键帧](KEYFRAME_REDUNDANCY_POLICY.md)。

## 15. 当前 Hpp/Hll 求解器应该怎样重建

### 15.1 Hll

每个临时点独立形成 3×3 块：

\[
H_{ll}=\sum J_l^TwJ_l.
\]

小视差时最弱特征向量接近平均视线，深度信息约随 \(\sin^2\theta\) 增长。Schur 消元当前
固定使用自伴特征分解构造带阈值伪逆，因为它需要明确判断半正定有效子空间。

**USE_LDLT_FOR_HLL** 只影响保留的旧 Independent EKF 消融路径，不控制默认 Schur
伪逆。不要根据早期性能文档误把默认 Schur Hll 直接换成无秩判定 LDLT。

### 15.2 Hpp

消元后

\[
H_s=BDB^T.
\]

默认用带置换 LDLT：

\[
H_s=P^TLDL^TP=M D M^T,\qquad M=P^TL.
\]

梯度必须通过三角回代得到

\[
\tilde g=M^{-1}g_s,
\]

不能用 \(M^Tg_s\) 代替。第 \(i\) 个有效方向变成

\[
z_i=\tilde g_i/d_i,\quad h_i=m_i,\quad R_i=\sigma_v^2/d_i.
\]

LDLT 的 D 无序，逐项用相对阈值

\[
d_i>\tau_p d_{max}
\]

判断。D 主元跳过数不严格等于特征值零空间维数；两者基不同，不能把多跳过一两个方向
直接解释为新物理 gauge。

### 15.3 当前默认零空间不是 31

历史 31 来自

\[
18\text{ INS}+6\text{ 单空槽}+7\text{ 纯视觉 gauge}.
\]

当前默认关闭重力、固定容量 30、通常活跃 20 或关键帧 push 后短暂 21 个 clone：

\[
n_0(k)\approx15+6(30-k)+7.
\]

所以典型结构值是 82 或 76，再叠加少量局部几何退化。矩阵仍是 195×195；把空槽数值填满
不会提速，只有真正动态缩小矩阵维数才会减少分解计算。

必读：[Hpp 零空间](HPP_NULLSPACE.md)、
[LDLT 优化](OPT_LDLT.md)。

## 16. 为什么保留 QR，但默认只验证 Schur

逐点 QR 消去 3 维 landmark，再拼接所有剩余行做大 QR，是标准 MSCKF 做法，数学上没有错：

\[
Q^TJ_l=
\begin{bmatrix}R\\0\end{bmatrix},\qquad
Q_2^TJ_p\delta x=Q_2^Tr.
\]

它慢的主要原因是所有点消元后的大量行仍进入一次高矩阵 QR；Schur 在每点 3×3 局部块处
就压缩成固定状态维数的信息矩阵。稀疏性只是次要因素。

保留 QR 用于历史 A/B，但后续默认算法修改先保证 Schur 正确。曾尝试：

- 去列选主元：有效，QR 明显提速；
- 增广残差列和去零行：几乎无收益；
- Hpl 手工块稀疏 Schur：因每点平均观测帧多且小块索引开销，反而慢；
- 直接正规方程代替 QR：条件数平方，退化场景发散。

必读：[QR 与 Schur](QR_VS_SCHUR.md)。

## 17. 每阶段的验证阶梯

不要一上来只跑 100 秒。推荐：

### 17.1 数学单元检查

- 投影与坐标变换；
- 解析雅可比对有限差分；
- 三角化合成点；
- FEJ 零空间泄漏；
- 增广 cross covariance；
- LDLT 与特征分解后验数值对比。

### 17.2 5–15 秒 smoke

确认无崩溃、NaN、负深度风暴和协方差不定。它只能发现明显错误，不能决定最终参数。

### 17.3 30 秒四场景

观察 Huber/硬拒绝、三角化、NIS/NEES、后验改善率和轨迹。

### 17.4 100 秒 / 600 点长期回归

统一命令：

~~~powershell
tools/run_multi_scenario_analysis.ps1 -Duration 100 -Features 600
~~~

2026-08-11 在算法端点 ebab4a0 的默认
MSCKF + AUTO(KeyframeOnly) + WORLD_XYZ + Hybrid 配置下得到：

| 场景 | 位置 RMSE | 速度 RMSE | 姿态 RMSE | 最大位置误差 | neg_cov |
|---|---:|---:|---:|---:|---:|
| Circle-out | 0.3262 m | 0.0585 | 0.00423 | 0.6316 m | 0 |
| Circle-in | 0.3124 m | 0.0531 | 0.00431 | 0.6157 m | 0 |
| Helix-3D | 0.0867 m | 0.0492 | 0.00413 | 0.3025 m | 0 |
| Stop-go | 0.0604 m | 0.0581 | 0.00578 | 0.1543 m | 0 |

四场景 reused=0、blocked=0。Stop-go 产生 5772 个无深度旋转约束，证明弱视差降级路径
确实被覆盖。耗时与机器负载相关，不应作为跨机器验收阈值。

### 17.5 失败时按首次异常定位

~~~mermaid
flowchart TD
    A["长时 RMSE 异常"] --> B{"reused/blocked 非零?"}
    B -- 是 --> C["先修观测生命周期"]
    B -- 否 --> D{"三角化成功率骤降?"}
    D -- 是 --> E["检查 clone 寿命、视差门限、删帧顺序"]
    D -- 否 --> F{"neg_cov 或对称误差异常?"}
    F -- 是 --> G["检查传播/增广/联合 cross covariance"]
    F -- 否 --> H{"NIS 极低或极高?"}
    H -- 是 --> I["检查噪声语义、重复窗口与鲁棒权重"]
    H -- 否 --> J["检查 FEJ、线性化和场景几何"]
~~~

## 18. 提交到专题的索引

| 实现阶段 | 参考提交 | 主要专题 |
|---|---|---|
| 多视图三角化 | b7e2e7e | [TRIANGULATION.md](TRIANGULATION.md) |
| 多场景与噪声 | 7b9861f | [SIMULATION_SCENARIOS.md](SIMULATION_SCENARIOS.md) |
| 严格消融 | 2f8ea8e | [ABLATION_STUDY.md](ABLATION_STUDY.md) |
| FEJ | b725fa2 | [OBSERVABILITY_CONSTRAINT.md](OBSERVABILITY_CONSTRAINT.md) |
| Landmark 修正 | f7e9d54 | [LANDMARK_UPDATE_STRATEGIES.md](LANDMARK_UPDATE_STRATEGIES.md) |
| 一致子空间 | 8df873a | [CONSISTENT_SUBSPACE_AND_LANDMARK_COVARIANCE.md](CONSISTENT_SUBSPACE_AND_LANDMARK_COVARIANCE.md) |
| 调度/参数化/RD-VIO | 8a652f9 | [VISUAL_UPDATE_SCHEDULING.md](VISUAL_UPDATE_SCHEDULING.md)、[LANDMARK_PARAMETERIZATION.md](LANDMARK_PARAMETERIZATION.md)、[RDVIO_SCHEDULING.md](RDVIO_SCHEDULING.md) |
| 长时修复和源码拆分 | 32498c4 | [SOURCE_LAYOUT.md](SOURCE_LAYOUT.md) |
| Hybrid MSCKF | bf319f3 | [HYBRID_MSCKF.md](HYBRID_MSCKF.md) |
| 混合数学补全 | a8839d1–419edb6 | [DEPTH_FREE_ROTATION_CONSTRAINT.md](DEPTH_FREE_ROTATION_CONSTRAINT.md) |
| 冗余删帧实验 | ebab4a0 | [KEYFRAME_REDUNDANCY_POLICY.md](KEYFRAME_REDUNDANCY_POLICY.md) |

## 19. 最容易“重实现成功但数学已经变了”的地方

1. 把 \(R_{wi}\) 当成 \(R_{iw}\)；
2. 名义姿态右乘和误差左乘混为一谈；
3. 用时间下标代替物理 **ordering**；
4. 增广只复制 6×6 自协方差，漏掉与 bias、clone、持久点的 cross block；
5. IMU 只传播 \(P_{II}\)；
6. 先删 clone 再消费轨迹；
7. 三角化失败后仍写入默认世界点；
8. 锚定参数化漏掉锚帧位姿雅可比；
9. Hll 删除了方向，但 \(g_l\) 仍保留该方向；
10. LDLT 使用 \(M^Tg\) 而不是三角回代 \(M^{-1}g\)；
11. 把 uv_var=0.01 当成默认 MSCKF 的单帧方差；
12. 把晋升当成第二次量测更新；
13. 用归档旧 pose 重新构造导航因子；
14. 只保存 \(P_{LL}\)，却声称持久点与导航联合；
15. 只看一条短轨迹或末帧位置；
16. 看到 Hpp 零方向变多就盲目降低阈值；
17. 顺手加入严格 SO(3) 积分或 reset 雅可比，却没有独立 A/B。

## 20. 当前仍然是近似或实验项的内容

重实现到当前版本不等于所有理论问题都已终结。以下边界应原样记录：

- 名义 SO(3) 的 J1/J2 是小步长工程近似；
- ESKF 过程噪声是状态空间对角密度近似，不是完整 \(GQ_cG^T\) 离散积分；
- 误差注入后暂未显式应用姿态 reset Jacobian；
- 外参雅可比保留但外参尚未进入状态；
- 硬可观性投影保留实验开关，默认关闭；
- 持久点 64 倍方差是仿真标定的保守值；
- RD-VIO 和 VINS-Mono 模式是 ESKF 调度适配，不等于其完整非线性优化器；
- 冗余感知关键帧删除尚未在当前场景显示优于最老删除；
- 固定 30 槽位造成 9～10 组常见空列；紧凑活跃视觉坐标可能提速，但尚未完成
  slot 映射、基向量补回完整协方差和长期 A/B，因此当前仍保留固定布局。

把这些边界保留下来，比为了“文档看起来完整”而把近似写成严格结论更重要。

## 21. 最终交付检查表

当你从 137bfea 逐步实现完成后，应同时满足：

- 默认构建是 Schur、Hpp LDLT、MSCKF、AUTO→KeyframeOnly、WORLD_XYZ；
- 重力固定，外参估计关闭但 J_ext 源码保留；
- 在线算法不读取 landmark GT；
- 四场景和 100 秒回归不发散；
- neg_cov=0；
- reused=0、blocked=0；
- 默认 clone 预算 20，三角化视差门限 2°；
- 低视差轨迹可延迟、可做无深度旋转约束、最终有界清理；
- 持久点不超过 20，完整维护 \(P_{xL}/P_{LL}\)；
- 当前报告能显示轨迹、状态修正、噪声敏感度、三角化、landmark、生命周期和耗时；
- 源码职责与 [SOURCE_LAYOUT.md](SOURCE_LAYOUT.md) 一致；
- 历史失败实验被记录，不被误当成当前默认。
