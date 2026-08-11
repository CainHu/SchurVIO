# ESKF 状态传播、协方差与 clone 增广

本文补齐当前工程中最容易因“符号默认值不同”而误读的部分：坐标系方向、左乘误差、
IMU 名义状态积分、联合协方差传播、固定物理槽位增广、视觉后验注入，以及当前尚未显式
实现的误差重置雅可比。公式描述的是 Windows 工程当前实现；理想连续模型与工程近似会
明确分开。

## 1. 坐标系与外参约定

下标约定：

- \(w\)：世界系；
- \(i\)：IMU/机体系；
- \(c\)：相机系；
- \(R_{ab}\)：把 \(b\) 系向量旋转到 \(a\) 系。

当前 **INSState::orientation** 和每个 clone 的 **Frame::q()** 都表示

\[
R_{wi}\colon\mathbb R^3_i\rightarrow\mathbb R^3_w.
\]

因此世界系中的 IMU 位姿满足

\[
p_w=R_{wi}p_i+p_{wi}.
\]

外参 **q_ic/t_ic** 的含义是

\[
R_{ic}\colon\mathbb R^3_c\rightarrow\mathbb R^3_i,\qquad
t_{ic}={}^{i}p_{ic},
\]

其中 \(t_{ic}\) 是从 IMU 原点指向相机原点、在 IMU 系表达的向量。相机世界位姿为

\[
R_{wc}=R_{wi}R_{ic},\qquad
p_{wc}=p_{wi}+R_{wi}t_{ic}.
\]

世界点 \(p_f\) 在相机系中的坐标是

\[
p_c
=R_{wc}^{T}(p_f-p_{wc})
=R_{ic}^{T}\left[R_{wi}^{T}(p_f-p_{wi})-t_{ic}\right].
\]

这正是三角化、普通 Schur 重投影、持久点更新和影子地图共同使用的投影模型。工程世界系
采用 \(g=[0,0,+9.81]^T\)，即 \(+z\) 为重力方向；它更接近 NED 的“向下为正”，不要把
公式直接套成常见的 ENU \(g_z=-9.81\)。

~~~mermaid
flowchart LR
    C["相机系 c"] -->|"R_ic, t_ic"| I["IMU 系 i"]
    I -->|"R_wi, p_wi"| W["世界系 w"]
    C -->|"R_wc=R_wi R_ic"| W
~~~

## 2. 名义状态、误差状态与排列

默认 **ESTIMATE_GRAVITY=false**，名义状态为

\[
x_I=(R_{wi},p_{wi},v_w,b_g,b_a),
\]

误差状态按

\[
\delta x_I=
\begin{bmatrix}
\delta\theta^T&
\delta p^T&
\delta v^T&
\delta b_g^T&
\delta b_a^T
\end{bmatrix}^{T}\in\mathbb R^{15}
\]

排列。若开启重力估计，再在末尾追加 \(\delta g\)，维数变为 18。每个 clone 为

\[
\delta x_{C_j}=
\begin{bmatrix}
\delta\theta_j^T&\delta p_j^T
\end{bmatrix}^{T}\in\mathbb R^6.
\]

混合后端启用后，完整联合误差状态是

\[
\delta\chi=
\begin{bmatrix}
\delta x_I^T&
\delta x_{C_0}^T&\cdots&\delta x_{C_{29}}^T&
\delta p_{L_0}^T&\cdots&\delta p_{L_{m-1}}^T
\end{bmatrix}^{T}.
\]

前 \(15+30\times6=195\) 维是固定物理布局；末尾持久点按需动态追加。默认只保留 20 个
活跃 clone，但固定容量仍为 30，所以“未使用槽位为零”不会自动缩小 LDLT 或 Schur
矩阵维数。

当前把 \(q,p\) 放在 INS 误差状态最前面是有意的：clone 正好复制连续的前 6 维，
增广可用整块拷贝。把状态重排成 \(g,b_g,b_a,v,q,p\) 不会降低矩阵维数或渐近复杂度，
却会修改所有偏移、传播分块和报告字段；除非有完整性能证据，否则不值得仅为“看起来和
clone 尾部相邻”而重排。

外参当前是固定参数，不在协方差中。将来若估计外参，更自然的布局是

\[
[\delta x_I,\delta x_E,\delta x_C,\delta p_L],
\]

其中外参块位于 INS 与 clone 容量之间；普通 clone 仍保存 IMU 位姿，视觉雅可比再同时
写入 clone 和外参列。仅把现有 **J_ext** 打开还不够，必须同步扩展状态、协方差传播、
注入和所有固定偏移。

## 3. 左乘姿态误差

当前误差定义为世界系左乘小角度：

\[
R_{wi}^{true}=\operatorname{Exp}(\delta\theta)R_{wi}.
\]

位置、速度和零偏使用加法误差：

\[
p^{true}=p+\delta p,\quad
v^{true}=v+\delta v,\quad
b_g^{true}=b_g+\delta b_g,\quad
b_a^{true}=b_a+\delta b_a.
\]

这里有两个常见混淆：

1. IMU 角速度在机体系，所以名义姿态传播是右乘
   \(R_{k+1}=R_k\operatorname{Exp}(\hat\omega\Delta t)\)；
2. 误差定义在世界系，所以视觉后验注入是左乘
   \(R^+=\operatorname{Exp}(\widehat{\delta\theta})R^-\)。

“名义状态右乘积分”和“误差左乘注入”并不矛盾，它们描述的是不同对象。

## 4. IMU 名义状态传播

去偏量为

\[
\hat\omega=\omega_m-b_g,\qquad
\hat a=a_m-b_a,\qquad
\phi=\hat\omega\Delta t.
\]

连续模型是

\[
\dot R_{wi}=R_{wi}[\hat\omega]_\times,\qquad
\dot p=v,\qquad
\dot v=R_{wi}\hat a+g.
\]

当前非调试路径使用

\[
R_{k+1}=R_k\operatorname{Exp}(\phi),
\]

\[
v_{k+1}=v_k+\left(R_kJ_1^{impl}(\phi)\hat a+g\right)\Delta t,
\]

\[
p_{k+1}=p_k+v_k\Delta t+\frac12g\Delta t^2
+R_kJ_2^{impl}(\phi)\hat a\Delta t^2,
\]

其中代码中的工程近似为

\[
J_1^{impl}=I+\frac12[\phi]_\times+\frac16\phi\phi^T,
\qquad
J_2^{impl}=\frac12I+\frac16[\phi]_\times+\frac1{24}\phi\phi^T.
\]

严格的 SO(3) Taylor 二阶项应分别含 \([\phi]_\times^2/6\) 和
\([\phi]_\times^2/24\)。当前实现用 \(\phi\phi^T\) 近似，少了与
\(-\|\phi\|^2I\) 对应的各向同性二阶项。200 Hz 下 \(\|\phi\|\) 很小，现有长时回归
未显示可见问题；但它应被视为已知工程近似，而不是精确的二阶积分。若以后替换为严格
闭式 \(J_1/J_2\)，应作为独立算法改动做交错 A/B，不能在重构时顺手改变。

**CONFIG_DEBUG=true** 使用较直观的欧拉/梯形积分，主要用于对照，不是默认性能路径。

## 5. 离散误差转移

按当前一阶离散实现，INS 误差转移矩阵 \(A\) 的非单位块为

\[
A_{\theta b_g}=-R_{wi,k+1}\Delta t,
\qquad
A_{pv}=I\Delta t,
\]

\[
A_{v\theta}=-[R_{wi,k}\hat a]_\times\Delta t,
\qquad
A_{vb_a}=-R_{wi,k+1}\Delta t.
\]

若估计重力，再有

\[
A_{vg}=I\Delta t.
\]

于是

\[
\delta x_{I,k+1}=A_k\delta x_{I,k}+w_k.
\]

该离散模型没有把姿态和加速度误差对位置的二阶耦合写入 \(A_{p\theta}\)、
\(A_{pb_a}\)。这与当前小步长一阶 ESKF 设计一致；若升级为更高阶离散化，名义积分、
\(A\) 和 \(Q_d\) 必须一起升级，否则只改其中一项会失去一致性。

## 6. 过程噪声的准确语义

当前实现使用

\[
Q_d=\operatorname{diag}(q_{\delta x})\,
\Delta t\,s_{proc},
\]

其中 \(q_{\delta x}\) 来自 **INSState::var_proc**，\(s_{proc}\) 是
**proc_noise_scale_**。这些量是“直接作用在误差状态各分量上的连续时间方差密度”：
姿态、位置、速度、两个 bias，必要时再加重力。

它不是严格从原始 IMU 噪声

\[
\dot{\delta x}=F\delta x+G n,\qquad
Q_d=\int_0^{\Delta t}\Phi(\tau)GQ_cG^T\Phi(\tau)^T\,d\tau
\]

完整积分得到的矩阵。尤其当前保留了一个很小的“位置直接随机游走”作为模型裕度。
仿真器中的 gyro/accel 白噪声密度与 bias random walk 是传感器生成模型；
ESKF 的 **var_proc** 则是经过保守放大的状态空间近似，两者数量级相关但不能逐项视为
同一个变量。

这一区分解释了为什么报告中扫描 **proc_scale** 有意义，但不能把某个仿真 RMSE 最优倍率
直接当作真实 IMU 标定结果。真实设备应先用 Allan 方差确定传感器密度，再决定是否保留
额外模型裕度。

## 7. 联合协方差传播

把 INS 之外的 clone 和持久点统称为 \(r\)，联合转移为

\[
F_J=
\begin{bmatrix}
A&0\\
0&I
\end{bmatrix}.
\]

若

\[
P=
\begin{bmatrix}
P_{II}&P_{Ir}\\
P_{rI}&P_{rr}
\end{bmatrix},
\]

则正确传播是

\[
P_{II}^{+}=AP_{II}A^T+Q_d,
\]

\[
P_{Ir}^{+}=AP_{Ir},\qquad
P_{rI}^{+}=P_{Ir}^{+T},\qquad
P_{rr}^{+}=P_{rr}.
\]

此前出现负协方差的根因正是只传播 \(P_{II}\) 而遗漏 \(P_{Ir}\)，并不是 Joseph 视觉
更新本身。遗漏交叉块会让一个原本合法的联合协方差不再对应同一个线性变换，后续视觉
增益就会使用相互矛盾的自协方差和互协方差。

~~~mermaid
flowchart LR
    P["P_II, P_Ir, P_rr"] --> A["INS 转移 A"]
    A --> II["P_II'=A P_II A^T+Qd"]
    A --> IR["P_Ir'=A P_Ir"]
    P --> RR["P_rr'=P_rr"]
    II --> S["恢复对称联合 P'"]
    IR --> S
    RR --> S
~~~

当前四种执行情况数学目标相同：

| 情况 | 实现思路 | 是否构造局部 \(A\) |
|---|---|---|
| 已有持久点，协方差动态扩维 | 显式传播 INS 行和全部 cross 列 | 是，仅 \(15\times15\) |
| **USE_STABLE_COVARIANCE_PREDICTION=true** | 保存旧块后做合同变换 | 是，仅 INS 局部矩阵 |
| **CONFIG_DEBUG=true** | 先算 \(AP\)，再按非零块右乘 \(A^T\) | 否 |
| 默认优化路径 | 先原位更新受影响列，再更新受影响行 | 否 |

默认无持久点时，**CONFIG_DEBUG=false** 且
**USE_STABLE_COVARIANCE_PREDICTION=false** 计算量最小。出现持久点后，代码自动走动态扩维
的局部 \(A\) 路径，以避免遗漏新增的 \(P_{IL}\) 列；两个开关不会让这部分相关性消失。

合同变换 \(APA^T\) 对半正定 \(P\) 保持半正定，\(Q_d\succeq0\) 只会增加不确定度。
实现仍应在关键边界检查

\[
\frac{\|P-P^T\|_F}{\max(\|P\|_F,\epsilon)},\qquad
\min\operatorname{diag}(P),\qquad
\lambda_{\min}\!\left(\frac{P+P^T}{2}\right).
\]

其中 \(10^{-12}\) 量级、相对最大特征值接近机器精度的负值属于舍入噪声；量级显著且持续
扩大的负值才是真正的不定。

## 8. clone 增广

相机时刻先把当前 IMU 位姿克隆到一个固定物理槽位。当前 clone 是 IMU 位姿，不是已经
乘外参后的相机位姿，因此

\[
\delta x_C=J_C\delta x_I,\qquad
J_C=
\begin{bmatrix}
I_3&0&0&0&0\\
0&I_3&0&0&0
\end{bmatrix}.
\]

对“全部旧联合状态” \(\chi\)，增广协方差应满足

\[
P_{C\chi}=J_CP_{I\chi},
\qquad
P_{CC}=J_CP_{II}J_C^T.
\]

这条公式必须同时覆盖 INS、其他 clone 和全部持久点：

\[
P_{CL}=J_CP_{IL}.
\]

当前 **Frame::ordering** 是物理协方差槽号，**SlidingWindow::active_idx** 才是时间顺序。
删除中间帧后两者通常不相等。所有雅可比写块、增广和状态注入必须使用 **ordering**；
删帧策略使用时间下标，二者不可混用。

`popFrame()` 不会在协方差上再做一次 Schur 补：视觉因子已在删帧前更新到后验，协方差
形式丢弃随机变量只需保留主子块；固定容量实现则把该主子块留在原物理位置并把槽标为空闲。
槽位复用时必须用上述 (P_{C\chi}) 公式覆盖整行整列，包括动态持久点尾部。完整推导见
[Clone 删除、协方差边缘化与固定槽位复用](CLONE_REMOVAL_AND_SLOT_REUSE.md)。

为什么当前仍是“先增广、再视觉后验”：

- 当前图像的残差需要一个与该图像曝光时刻对应的位姿变量；
- 默认一次性 MSCKF 会把当前观测与历史观测放在同一轨迹批次；
- 若先后验再增广，就必须让当前残差直接作用于 INS 位姿，并重新推导 INS/clone 混合
  雅可比和删除时序，不能仅交换两行函数调用。

因此“先更新再复制 q/p”表面上少了一个 clone 参与后验，实际上改变了状态图。当前固定
维数实现中，它也不会让 LDLT 的矩阵维数变小；没有证据表明值得为此重构。

## 9. 视觉后验、Joseph 更新与误差注入

每个有效标量伪量测使用

\[
S=h^TPh+R,\qquad
K=Ph/S,
\]

\[
\delta\chi\leftarrow
\delta\chi+K(z-h^T\delta\chi),
\]

\[
P^+=(I-Kh^T)P(I-Kh^T)^T+KRK^T.
\]

代码把 Joseph 形式展开成低秩上三角更新，循环结束后一次性恢复下三角。只要循环内所有
矩阵向量乘都通过上三角自伴视图读取，这与每一步复制完整下三角数学等价。

完成一批量测后，姿态和 clone 使用左乘注入：

\[
R^+=\operatorname{Exp}(\widehat{\delta\theta})R^-,
\]

其余欧氏状态和持久点使用加法注入。普通 MSCKF 的直接雅可比只在固定主状态块非零，但
若已有 \(P_{Lx}\)，Kalman 增益的 landmark 行通常非零，所以普通轨迹仍会通过相关性
一致地修正持久点。

## 10. 误差重置雅可比：当前近似与边界

严格 ESKF 在把 \(\widehat{\delta\theta}\) 注入名义姿态后，还应把协方差从旧误差切空间
映射到新切空间。对当前左乘误差，一阶重置块为

\[
G_{\theta}\approx
I+\frac12[\widehat{\delta\theta}]_\times,
\]

完整重置为

\[
P\leftarrow G_{reset}PG_{reset}^T,
\]

且所有与该姿态块相关的 cross covariance 都必须一起变换。每个被修正的 clone 也有各自
的 \(G_{\theta_j}\)。

当前代码在 **updateState() / applyJointStateCorrection()** 后没有显式执行这一步，
等价于采用 \(G_{reset}\approx I\) 的一阶小修正近似。当前仿真中单次姿态修正较小，100 秒
回归保持稳定；但这是实现边界，不应在文档中假装已经完成严格 reset。

若将来补充 reset，必须：

1. 保存本批实际注入的所有姿态增量；
2. 对 INS、每个活跃 clone 以及它们与持久点的 cross block 同时做合同变换；
3. 在持久点直接 EKF 和普通 Schur 两次可能连续注入的路径上都执行；
4. 比较轨迹、NEES/NIS、最小协方差特征值和耗时，避免只看末帧位置。

## 11. 推荐验证不变量

每次修改传播、增广或状态布局后，至少验证：

- 相机模型从世界点投影到归一化像平面的数值与仿真器一致；
- 静止时 \(a_m\approx R_{wi}^T(-g)+b_a\)，传播后的世界加速度接近零；
- **ordering** 指向的物理块与被更新的 **Frame** 一致；
- 增广前后旧协方差左上块不变，新 clone 与全部旧状态的 cross block 满足 \(J_CP\)；
- IMU 传播后 \(P_{Ir}=AP_{Ir}\)，而不是只传播 \(P_{II}\)；
- **neg_cov=0**，对称误差接近机器精度；
- 一次性 MSCKF 下 **reused=0**、**blocked=0**；
- 关闭/开启显式稳定传播时，在浮点容差内得到相同轨迹。

相关专题：

- [数学总流程](MATHEMATICAL_PIPELINE.md)
- [视觉残差与噪声模型](VISUAL_RESIDUAL_NOISE_MODEL.md)
- [混合 MSCKF](HYBRID_MSCKF.md)
- [Hpp 零空间](HPP_NULLSPACE.md)
- [视觉后验分析](ANALYSIS_REPORT.md)
