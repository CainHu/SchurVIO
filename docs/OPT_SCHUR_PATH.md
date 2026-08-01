# Schur 路径 (`USE_SCHUR`) 视觉更新优化记录

对应提交：`e7f5a7c`（分支 `schur_opt`）

## 结果

| 指标 | 优化前 | 优化后 |
|---|---|---|
| 总耗时 (`t_cost`) | 18.3 s | **11.9 s** |
| 平均每次后验更新 | 9.2 ms | **6.0 ms** |
| 精度 (`POS EST`) | `2.04503 4.58435 0.986966` | 逐位相同 |

交错 A/B 三轮：18.3 / 18.3 / 18.3 s vs 12.0 / 11.9 / 12.0 s（**1.53 倍**）。

## 零、先说最重要的结论

**Schur 路径原封不动就比精心优化过的 QR 路径快 3.4 倍。**

| 路径 | 耗时 |
|---|---|
| QR（优化前） | 203.5 s |
| QR（优化后，见 [OPT_QR_PATH.md](OPT_QR_PATH.md)） | 61.8 s |
| **Schur（原样）** | **18.3 s** |
| **Schur（本次优化后）** | **11.9 s** |

两条路径精度基本一致（`POS` 差 4e-5）。若以性能为准，应该用 Schur 路径。

## 一、优化前的耗时分布

新增分阶段计时 `t_build_H_` / `t_schur_` / `t_eig_state_` / `t_eig_lmk_`：

| 阶段 | 耗时 |
|---|---|
| 构建 `Hpp` / `Hpl` / `Hll` | 6.15 s |
| Schur 补 | 2.46 s |
| `Hpp` 特征分解 + 序贯更新 state | 6.08 s（其中特征分解仅 2.94 s，**序贯循环 3.1 s**） |
| `Hll` 特征分解 + 更新 landmark | 0.50 s |

与 QR 路径不同，这里**没有单一压倒性瓶颈**，需要逐项处理。

## 二、有效的改动

### 1. `Hll` 改为只存对角块（收益最大）

`Hll` 是**块对角矩阵**——landmark 之间没有直接耦合，只通过 pose 间接耦合。
但原实现按稠密矩阵分配：

```cpp
// 改前：lmk_size × lmk_size = (3×326)² ≈ 96 万个 double，每帧全部清零
MatXX Hll(lmk_size, lmk_size);
Hll.setZero();

// 改后：只存对角线上的 326 个 3×3 块 = 2934 个 double
Eigen::Matrix<TYPE, Eigen::Dynamic, LMK_SIZE> Hll_diag(lmk_size, LMK_SIZE);
Hll_diag.setZero();
```

**省掉 99.7 % 的分配和清零。** 访问方式相应从
`Hll.block<3,3>(index, index)` 改为 `Hll_diag.middleRows<3>(index)`。

> `build H`: 5.88 s → **2.96 s**

### 2. 序贯更新 state 循环去掉对称性浪费

这个循环跑 198 次（`COV_SIZE`），每次迭代原本这样：

```cpp
// 改前
VecX PhT = cov_p * hT;                    // 稠密矩阵×向量，读了全部 198×198
cov_p -= K * PhT.transpose();             // 更新了完整 198×198
PhT = cov_p * hT;
cov_p.triangularView<Upper>() += (K*R - PhT) * K.transpose();
cov_p.triangularView<StrictlyLower>() = cov_p.triangularView<StrictlyUpper>().transpose();
//  ↑ 每次迭代都重建一次下三角
```

`cov_p` 全程保持对称，所以对称部分实际算了两遍。改为：

```cpp
// 改后
PhT.noalias() = cov_p.selfadjointView<Eigen::Upper>() * hT;   // 只读上三角
cov_p.triangularView<Eigen::Upper>() -= K * PhT.transpose();  // 只写上三角
PhT.noalias() = cov_p.selfadjointView<Eigen::Upper>() * hT;
cov_p.triangularView<Eigen::Upper>() += (K*R - PhT) * K.transpose();
// 循环内不再重建下三角
...
// 循环结束后统一恢复一次
cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();
```

**数学上完全等价**：中间过程只有 `selfadjointView` 在读 `cov_p`，而它只看上三角。
另外把 `PhT` / `K` 提到循环外复用，避免 198 次堆分配。

> `eig+seq state`: 8.98 s → **6.08 s**

### 3. `build H` 循环内的不变量提升

```cpp
// 改前：每个观测都重算
const auto Ric = ext_.q_ic.toRotationMatrix();                       // 外参是常量！
Mat2_3 J_lmk = J * (frm->q() * ext_.q_ic).inverse().toRotationMatrix();  // 四元数乘+求逆+转矩阵

// 改后
const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();   // 提到所有循环外，整帧只算一次
...
J_lmk.noalias() = J * (Ric.transpose() * Rwi.transpose());  // Rwc^T = Ric^T · Rwi^T，复用已有量
```

> `build H`: 6.15 s → 5.88 s（小幅）

### 4. `J_ext` 用 `if constexpr` 屏蔽（而非删除）

`J_ext`（外参雅可比）每个观测都在计算，但**写完从未被任何代码读取**——
外参目前不在状态里。QR 路径的 `J_EXT` 同理，只写不读。

考虑到后续有可能把外参加入状态，**保留代码但默认不运行**：

```cpp
// common.h
struct ExtState {
    constexpr static bool ESTIMATE_EXTRINSIC = false;
    constexpr static int Q = 0, P = Q + 3, SIZE = P + 3;
    ...
};

// schur_vins.cpp（两条路径都是这样）
if constexpr (ExtState::ESTIMATE_EXTRINSIC) {
    Mat2_6 J_ext;
    J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
    J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);
    // QR 路径还有: J_EXT.middleRows<2>(row_start) = J_ext;
}
```

**为什么用 `if constexpr` 而不是 `#ifdef`：** 代码始终参与语法和类型检查，
不会因为长期不启用而悄悄腐烂。已验证把开关改成 `true` 能正常编译通过。
沿用了项目中 `INSState::ESTIMATE_GRAVITY` 的既有模式。

开启外参估计时还需要：
1. 把外参的 6 维加进 `COV_SIZE` 和协方差布局
2. 在 `updateState` 里更新 `q_ic` / `t_ic`
3. 把 `J_EXT` / `J_ext` 填进对应的 Jacobian 列（Schur 路径是累加进 `Hpp`/`Hpl`/`gp` 的外参块）

## 三、试过但失败的改动（已回退）

### 利用 `Hpl` 的块稀疏性做 Schur 补 —— 反而变慢

**设想：** 每个 landmark 只被 K 个关键帧观测到，`Hpl` 该列块中只有 K 个 6×3 块非零，
其余全零。原实现对 198×3 和 198×198 做稠密运算，绝大部分在乘零。
改成只在 K 个非零块上运算，复杂度从 O(WIN_SIZE²) 降到 O(K²)。

**实测结果：Schur 补 2.46 s → 3.49 s，变慢了 42 %。**

**原因：** 加计数器测出 **mean K = 14.8**，而窗口是 30——
我原本估计 K ≈ 5，**估错了 3 倍**。稀疏度只有 2 倍，
而 K×K ≈ 219 次 6×6 小块乘法的标量索引开销超过了省下的乘零。
稠密 GEMM 的向量化更划算。

> 教训：稀疏化只在稀疏度足够高时才划算。动手前先测密度。

## 四、优化后的剩余分布

| 阶段 | 耗时 |
|---|---|
| build H | 2.96 s |
| Schur 补 | 2.53 s |
| `Hpp` 特征分解 + 序贯更新 | 5.96 s（其中特征分解 2.87 s） |
| `Hll` 特征分解 + 更新 lmk | 0.47 s |

分布已相当均匀，没有单一大瓶颈。

**下一个可能方向：** `Hpp` 的 198×198 特征分解那 2.87 s。
它只用于把 `Hpp` 对角化后做序贯更新，改成 Cholesky（`LDLT`）会快得多，
但会改变滤波器对退化情况的处理方式——现在靠特征值阈值
（`> 1e-6 * λ_max`）过滤零空间。这是**数值行为的改变，不是纯优化**，
需要先确认是否接受。

## 五、测量方法

这台机器单次计时波动极大。**所有结论都基于交错 A/B**：

```bash
for i in 1 2 3; do
  echo -n "base: "; /tmp/schur_base.exe 2>&1 | grep "^t_cost"
  echo -n "opt:  "; /tmp/schur_opt.exe  2>&1 | grep "^t_cost"
done
```

### 过程中的一次测量错误

第一次建基线时，`git checkout eskf/schur_vins.cpp` 在编译**之前**就把宏改回去了，
导致「基线」二进制实际走的是 QR 路径，跑出 61.5–62.5 s。
发现方式：`big QR = 45.48 s` 不该出现在 Schur 路径里。
重建正确基线后得到 18.3 s。本文档所有数字均来自修正后的测量。

另：最早一次测 Schur 基线得到 36.4 s，是机器负载偏高所致，真实值 18.3 s。

## 相关

- QR 路径的优化见 [OPT_QR_PATH.md](OPT_QR_PATH.md)
- 两条路径通过 `schur_vins.cpp` 顶部的 `USE_QR` / `USE_SCHUR` 宏切换
