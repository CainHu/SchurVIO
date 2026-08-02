# QR 路径 (`USE_QR`) 视觉更新优化记录

对应提交：`f2298c0`（分支 `plan4_zero_copy`）

> 本文的 `198 = INS(18) + 30×6` 是当时开启重力估计的历史性能配置。当前默认关闭重力
> 估计，`COV_SIZE=195`；算法结论和瓶颈位置不变，文中的尺寸与耗时保留为原实验记录。

## 结果

| 指标 | 优化前 | 优化后 |
|---|---|---|
| 总耗时 (`t_cost`) | 203.5 s | **61.8 s** |
| 平均每次后验更新 | 102.0 ms | **31.0 ms** |
| 精度 (`POS EST`) | `2.04499 4.58437 0.986966` | 逐位相同 |

交错 A/B 三轮验证：61.5 / 62.9 s vs 115.0 / 115.9 s（基线当时机器较空闲，跑出 115 s；
同一轮次内交错对比才有意义，见文末「测量方法」）。

## 一、先测量，再优化

优化前先加分阶段计时（`t_perlmk_qr_` / `t_bigqr_` / `t_seq_state_` / `t_lmk_update_`），
得到耗时分布：

| 阶段 | 耗时 | 占比 |
|---|---|---|
| 每个 landmark 的小 QR（2K×3） | 9.80 s | 4.8 % |
| **`J_STATE` 的大 QR** | **172.71 s** | **84.9 %** |
| 序贯更新 state（198×198 协方差） | 14.91 s | 7.3 % |
| 更新 landmark 位置 | 0.90 s | 0.4 % |
| 非关键帧 refine（方案4） | 0.07 s | 0.03 % |

结论很明确：**唯一的瓶颈是 `J_STATE` 的 QR 分解**，其余全部加起来不到 15 %。

`J_STATE` 的规模：`(2 · num_obs) × 180`，其中 `num_obs ≈ 1600`
**实测 `J_STATE` 的平均行数是 8678**，即约 **8678 × 180**。

> 更正：本文档早先写的"约 3200 × 180"是算错的。
> 正确的推导：每个 landmark 有 `2K = 30` 行观测，小 QR 消掉 3 维 landmark 后
> 剩 `2K − 3 = 27` 行，共 `326 × 27 ≈ 8700` 行，与实测 8678 吻合。

## 二、有效的改动

### 1. `colPivHouseholderQr` → `householderQr`（唯一真正有效的改动）

```cpp
// 改前
auto qr = J_STATE.colPivHouseholderQr();
const MatXX H_red = R_red * qr.colsPermutation().transpose();

// 改后
auto qr = JE.householderQr();
const MatXX H_red = QR.topLeftCorner(n_eff, n_cols).triangularView<Eigen::Upper>();
```

列选主元（column pivoting）对 8678×180 的矩阵开销极大，而这里**不需要 rank-revealing**。
顺带省掉了 `R * P^T` 这次矩阵乘法。

**为什么去掉列选主元是安全的：**

`J_STATE` 降秩时，`R` 的对应行会趋于 0，于是序贯更新中
`hT ≈ 0` ⟹ `PhT ≈ 0` ⟹ `K ≈ 0`，该行自然不贡献修正量。
退化是**平滑**的，不会数值爆炸。

实测 `mean seq rows = 179.978`（满列 180），说明这条轨迹上 `J_STATE` 一直接近满秩。

> ⚠️ **这是基于当前仿真数据的观察，不是通用保证。**
> 若换到特征点稀疏或退化运动（纯旋转、匀速直线）的场景，`J_STATE` 可能真正降秩。
> 建议届时加一道监控：检查 `|R(j,j)|` 的最小值，低于阈值时告警，
> 先确认是否真的发生，再决定要不要处理。

### 2. 增广矩阵 + 只分解有效行（**实测无收益，仅为代码整洁**）

```cpp
MatXX JE(m_eff, n_cols + 1);
JE.leftCols(n_cols) = J_STATE.topRows(m_eff);
JE.col(n_cols)      = E_STATE.head(m_eff);
auto qr = JE.householderQr();
const VecX e_red = QR.col(n_cols).head(n_eff);   // 即 (Q^T · E_STATE).head(n)
```

- 把 `E_STATE` 拼进增广矩阵一起分解，省掉单独对 `E_STATE` 应用一次 Householder 序列
- `J_STATE` 按 `2·num_obs` 行分配但只填了 `row_idx` 行，后面是零行，只分解前 `row_idx` 行

### 3. `hT` 提到循环外

`hT` 的 INS 段（前 18 维）恒为 0，每次迭代只需重写 `tail`，不必重新分配整个 198 维向量。

## 三、一次被数据推翻的判断（重要）

我最初预估改动 2 能带来 5–10 % 的提升，**实测完全不成立**。

做了一个只换 QR 类型、保留零行和独立 `Q^T·E` 的隔离变体：

| 版本 | `t_cost` | `big QR` |
|---|---|---|
| 仅换 QR 类型 | 61.8 / 64.5 / 62.0 s | 45.05 s |
| + 增广矩阵 + 去零行 | 61.7 / 61.9 / 63.1 s | 45.03 s |

**两者在噪声内完全无差别。1.86 倍的提速几乎全部来自改动 1。**

另外修正一个我先前的错误说法：曾说
`qr.householderQ().transpose() * E_STATE`「显式构造了 Q」——**这是错的**。
Eigen 的 `householderQ()` 返回 `HouseholderSequence`，乘向量时就地应用 Householder 反射，
从不展开成 m×m 矩阵。那里本来就没有多余的矩阵构造。

改动 2、3 保留是因为代码更紧凑、逻辑上确实少一趟 O(m·n) 运算，
**这是可读性理由，不是性能理由**。若追求最小改动，只改第 1 点（单行）即可拿到同样速度。

## 四、优化后的剩余分布

| 阶段 | 耗时 |
|---|---|
| per-lmk 小 QR | 4.98 s |
| big QR (`J_STATE`) | 45.03 s |
| 序贯更新 state | 6.90 s |
| lmk 位置更新 | 0.42 s |

大 QR 仍占 73 %。下一个可能的方向是**限制每个 landmark 的观测数**
（QR 是 O(m·n²)，行数砍半则时间砍半），但那会改变滤波器行为、影响精度，
属于精度/速度取舍，需要先定策略。

## 五、测量方法

这台机器单次计时波动极大（同一个二进制文件可在 115 s ~ 204 s 之间浮动）。
因此所有结论都基于**交错 A/B**：

```bash
for i in 1 2 3; do
  echo -n "base: "; /tmp/base.exe 2>&1 | grep "^t_cost"
  echo -n "opt:  "; /tmp/opt.exe  2>&1 | grep "^t_cost"
done
```

单独跑一次基线、再单独跑一次优化版，得到的比值不可信。

## 相关

- Schur 路径的优化见 [OPT_SCHUR_PATH.md](OPT_SCHUR_PATH.md)
- **注意：Schur 路径本身就比优化后的 QR 路径快 3.4 倍**（18.3 s vs 61.8 s）
