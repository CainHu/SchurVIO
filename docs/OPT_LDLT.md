# `Hpp` 分解: 特征分解 → LDLT

对应分支：`ldlt_experiment`（从 `optimized` 切出）

## 结论

**可用，建议采纳。** 分解本身快 7 倍，总耗时降 20 %，精度无实质变化，
且没有引入新的数值风险。

| 指标 | 特征分解 | LDLT |
|---|---|---|
| `Hpp` 分解耗时 | 2.92 s | **0.39 s**（7.5×） |
| `eig+seq state` | 6.02 s | **3.47 s** |
| 总耗时 `t_cost` | 12.03 s | **9.67 s**（1.25×） |
| 最终 `POS EST` | `2.04503 4.58435 0.986966` | `2.04495 4.58319 0.986941` |
| 与 GT 的误差 | ~2 cm | ~2 cm |

交错 A/B 三轮：12.07 / 12.09 / 12.02 s vs 9.63 / 9.69 / 9.68 s。

## 一、为什么 LDLT 可以替换特征分解

序贯更新要求各标量量测互不相关，即把 `Cov[e] = σ²·Hpp` 对角化。
**这一点特征分解和 LDLT 都能做到**，区别只在用哪组基：

```
特征分解  Hpp = V·λ·V^T   =>  Cov[V^T·e]  = σ²·V^T·Hpp·V   = σ²·λ   (对角)
LDLT      Hpp = L·D·L^T   =>  Cov[L^-1·e] = σ²·L^-1·Hpp·L^-T = σ²·D   (对角)
```

于是两者都给出 `COV_SIZE` 个独立的标量量测：

| | 量测方向 `h_i` | 量测值 `z_i` | 量测噪声 `R_i` |
|---|---|---|---|
| 特征分解 | `V.col(i)` | `(V^T·gp)_i / λ_i` | `σ²/λ_i` |
| LDLT | `M.col(i)` | `(M^-1·gp)_i / D_i` | `σ²/D_i` |

其中 `M = P^T·L`——**注意 Eigen 的 LDLT 带主元置换**，`A = P^T·L·D·L^T·P`，
直接用 `matrixL()` 是错的。

`M^-1·gp` 用三角回代求，不显式求逆：

```cpp
Eigen::LDLT<MatXX> ldlt(Hpp);
H_BASIS = ldlt.transpositionsP().transpose() * MatXX(ldlt.matrixL());
H_BASIS_diag = ldlt.vectorD();

// M·rhs = gp  =>  P^T·L·rhs = gp  =>  L·rhs = P·gp
rhs = ldlt.transpositionsP() * ep;
ldlt.matrixL().solveInPlace(rhs);
```

**两者不逐位等价**（用的是不同的基），但都是同一个信息矩阵的合法分解，
最终后验在数值误差内一致。实测末帧位置差约 1.2 mm，
而估计本身与 GT 的误差是 2 cm 量级——差异被淹没在估计误差里。

## 二、退化处理：一个改变了我判断的实测结果

我原先的担心是：特征分解靠特征值阈值（`> 1e-6·λ_max`）过滤零空间，
换成 LDLT 会丢掉这个保护。**这个担心站不住脚，但理由和我预想的不一样。**

加诊断计数器统计后发现，**`Hpp` 本来就不是半正定的，两条路径都在跳过大量方向**：

| | 跳过的方向总数 | 其中严格为负 |
|---|---|---|
| 特征分解 | 68,122 | 10,957 |
| LDLT | 70,090 | 6,967 |

（全程 1996 次更新 × 198 维，即平均每次跳过 34~35 个方向，约占 18 %。）

也就是说：
- **不是 LDLT 引入了不定性**，`Hpp` 原本就不定
- 两种分解**都**在用同一套逻辑处理（跳过 `d <= 1e-6·d_max` 的方向）
- LDLT 没有让情况变坏

两者数量略有差异（70,090 vs 68,122）是正常的——不同基下"能量"分布不同，
被阈值切掉的方向数自然不同。

### 实现上的一个差别

特征分解的 `eigenvalues()` **已升序排列**，所以原代码用一个起始下标
`zero_end` 扫过前缀即可；而 **LDLT 的 `D` 无序**，必须逐个判断。
因此循环改成了 `continue` 跳过的写法：

```cpp
const TYPE d_thresh = TYPE(1e-6) * H_BASIS_diag.maxCoeff();
for (size_t i = 0; i < COV_SIZE; ++i) {
    const auto d = H_BASIS_diag(i);
    if (d <= d_thresh) continue;   // 零空间方向，不提供信息
    ...
}
```

这个写法对两条路径都正确（特征分解只是恰好前缀连续）。

## 三、开关

```cpp
// eskf/schur_vins.h
constexpr static bool USE_LDLT_FOR_HPP = true;   // false = 回到特征分解
```

两种分解的代码都保留，可随时切换对比。

## 四、遗留问题（值得单独看）

诊断顺带暴露了一件事：**`Hpp` 有 18 % 的方向被判定为零空间，其中相当一部分特征值为负。**

对一个信息矩阵而言，负特征值意味着数值问题（Schur 补的累积误差、
或 `Hll` 求逆时的病态）。当前靠阈值把它们全部丢弃，滤波器能正常工作、
不发散，但这**掩盖了一个可能的隐患**，而不是解决了它。

值得单独排查的方向：
- `Hll` 的条件数（代码开头的开发日志第 4 条已提到 "Hll 的条件数会比较大"）
- Schur 补 `Hpp - Hpl·Hll^-1·Hpl^T` 的数值稳定性
- `completeOrthogonalDecomposition().pseudoInverse()` 对病态 `Hll` 的处理

这与 LDLT 的采纳与否无关（两条路径表现一致），但建议后续跟进。

## 相关

- [OPT_SCHUR_PATH.md](OPT_SCHUR_PATH.md) — Schur 路径其余优化
- [README.md](README.md) — 总览
