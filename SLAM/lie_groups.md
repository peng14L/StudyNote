# 李群与李代数：读懂 VINS-Mono 所需的最小集合

自学讲义。面向**有线性代数、多元微积分、优化基础、无 SLAM 背景**的读者。读完应能：

- 说明为什么位姿不能当作 $\mathbb{R}^6$ 或 $\mathbb{R}^{12}$ 里的普通向量来优化；
- 熟练使用 $\mathrm{SO}(3)$、$\mathrm{SE}(3)$、$(\cdot)^\wedge$、$\exp/\log$、伴随 $\mathrm{Ad}$；
- 在**右扰动**约定下，推导点变换与复合变换的雅可比思路；
- 对照本仓库 [`VINS-Mono`](../../VINS-Mono/) 中 `utility.h`、`pose_local_parameterization.cpp`、`integration_base.h`、`projection_factor.cpp` 读懂实现。

**不必现在掌握**：一般李群公理化、$\mathrm{Sim}(3)$ 全套推导、流形上的测地线 BA 理论。那些留到读 ORB-SLAM 回环或论文附录时再补。

## 目录

1. [为什么需要李群](#1-为什么需要李群)
2. [符号表](#2-符号表)
3. [SO(3)：旋转群](#3-so3旋转群)
4. [so(3)：旋转的李代数](#4-so3旋转的李代数)
5. [指数映射与 Rodrigues 公式](#5-指数映射与-rodrigues-公式)
6. [对数映射与角度轴](#6-对数映射与角度轴)
7. [SE(3)：刚体位姿群](#7-se3刚体位姿群)
8. [se(3)：位姿的李代数](#8-se3位姿的李代数)
9. [复合、逆与伴随](#9-复合逆与伴随)
10. [扰动模型：优化的核心](#10-扰动模型优化的核心)
11. [必会雅可比：推导套路](#11-必会雅可比推导套路)
12. [四元数与 SO(3) 的双覆盖](#12-四元数与-so3-的双覆盖)
13. [与 VINS-Mono 代码对照](#13-与-vins-mono-代码对照)
14. [掌握清单与阅读顺序](#14-掌握清单与阅读顺序)
15. [自测](#15-自测)
16. [自测参考答案](#16-自测参考答案)

---

## 1. 为什么需要李群

### 1.1 从线性空间到流形

在 $\mathbb{R}^n$ 上做 Gauss–Newton 时，更新是

$$
\mathbf{x} \leftarrow \mathbf{x} + \delta\mathbf{x}.
$$

这依赖「加法」保持你在空间里。但**旋转**不满足这一点：

- 两个旋转矩阵相乘仍是旋转，但两个旋转矩阵**逐元素相加**一般不再是旋转；
- 欧拉角 $(roll, pitch, yaw)$ 可以相加，但会在万向节锁附近奇异，且相加后不保证仍是合法旋转。

因此旋转属于**流形**（manifold）$\mathrm{SO}(3)$，不是向量空间。优化时：

1. **存储（表示）**：用群元素 $R \in \mathrm{SO}(3)$（或四元数、旋转矩阵）；
2. **更新（增量）**：在**切空间** $\mathfrak{so}(3) \cong \mathbb{R}^3$ 里取 small $\delta\boldsymbol{\phi}$，再通过 $\exp$ 映回群。

```mermaid
flowchart LR
  tangent["切空间 R3 增量 phi"]
  exp["exp 映射"]
  group["SO3 上的旋转 R"]
  log["log 映射"]
  tangent --> exp --> group
  group --> log --> tangent
```

VINS-Mono 里每一帧的 `Rs[i]`、IMU 预积分的 `delta_q`、Ceres 里对位姿的 `delta`，全是这一套。

### 1.2 李群在 VINS 里出现在哪

| 数学对象 | VINS 中的对应 |
| --- | --- |
| $R \in \mathrm{SO}(3)$ | `Matrix3d Rs[i]`，或 `Quaterniond` |
| $T \in \mathrm{SE}(3)$ | 位姿 $(R, \mathbf{p})$，相机–IMU 外参 `(ric, tic)` |
| $[\boldsymbol{\omega}]_\times$ | `Utility::skewSymmetric()` |
| $\exp(\boldsymbol{\phi})$ | Rodrigues / `deltaQ` 小角度近似 |
| 右扰动 $R \leftarrow R \exp(\delta\boldsymbol{\phi}^\wedge)$ | `PoseLocalParameterization::Plus` 中的 `q = (_q * dq)` |
| 点变换雅可比 | `projection_factor.cpp` 中 `skewSymmetric` 块 |
| 伴随 $\mathrm{Ad}$ | 预积分 Jacobian 推导（`integration_base.h`） |

---

## 2. 符号表

| 符号 | 含义 |
| --- | --- |
| $R \in \mathrm{SO}(3)$ | 三维旋转矩阵，$R^\top R = I,\ \det R = 1$ |
| $\mathbf{p}, \mathbf{t} \in \mathbb{R}^3$ | 平移向量 |
| $T = \begin{pmatrix} R & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{pmatrix} \in \mathrm{SE}(3)$ | 齐次刚体变换 |
| $\boldsymbol{\phi}, \boldsymbol{\theta} \in \mathbb{R}^3$ | 旋转增量（so(3) 坐标） |
| $\boldsymbol{\rho}, \delta\mathbf{p} \in \mathbb{R}^3$ | 平移增量 |
| $\boldsymbol{\xi} = \begin{pmatrix} \boldsymbol{\rho} \\ \boldsymbol{\phi} \end{pmatrix} \in \mathbb{R}^6$ | se(3) 坐标（多种分块顺序存在，本文与 Barfoot 一致：先平移后旋转） |
| $(\cdot)^\wedge$ | 向量 → 反对称矩阵（hat） |
| $(\cdot)^\vee$ | 反对称矩阵 → 向量（vee） |
| $\exp(\cdot)$ | 李代数 → 李群 |
| $\log(\cdot)$ | 李群 → 李代数 |
| $\mathrm{Ad}_T$ | 伴随变换，$6 \times 6$ |
| $[\mathbf{a}]_\times$ | 与 $\mathbf{a} \times (\cdot)$ 等价的反对称矩阵 |

**坐标系约定**（与 VINS、本仓库针孔讲义一致）：世界系 $w$，IMU 体系 $b$，相机系 $c$。点变换

$$
\mathbf{p}_a = R_{ab}\,\mathbf{p}_b + \mathbf{t}_{ab}
$$

表示「把 $b$ 坐标下的点变到 $a$ 坐标下」。复合：$R_{ac} = R_{ab} R_{bc}$。

---

## 3. SO(3)：旋转群

### 3.1 定义

$$
\mathrm{SO}(3) = \{\, R \in \mathbb{R}^{3\times 3} \mid R^\top R = I,\ \det R = 1 \,\}.
$$

这是一个**群**：对乘法封闭，有单位元 $I$，每个元素有逆 $R^{-1} = R^\top$。

### 3.2 几何意义

$R$ 保持向量长度与 handedness，描述三维刚体**纯旋转**。作用在点上：$\mathbf{p}' = R\mathbf{p}$。

### 3.3 参数化对比（知道即可）

| 表示 | 自由度 | 优点 | 缺点 |
| --- | --- | --- | --- |
| 旋转矩阵 | 3（9 元过参数化 + 约束） | 作用点直接 | 优化需约束 $R^\top R=I$ |
| 欧拉角 | 3 | 直观 | 万向节锁、不唯一 |
| 轴角 / 旋量 $\boldsymbol{\phi}$ | 3 | 与 $\exp$ 自然衔接 | $\|\boldsymbol{\phi}\| > \pi$ 时不唯一 |
| 单位四元数 $q$ | 3（4 元 + 约束） | 无奇点、插值 smooth | $q$ 与 $-q$ 同一旋转 |

**VINS 存储**：优化变量用 `Vector3d p` + `Quaterniond q`（7 个数），增量用 6 维 `delta`（3 平移 + 3 旋转）。

---

## 4. so(3)：旋转的李代数

### 4.1 定义

$$
\mathfrak{so}(3) = \{\, \boldsymbol{\Phi} \in \mathbb{R}^{3\times 3} \mid \boldsymbol{\Phi}^\top = -\boldsymbol{\Phi} \,\},
$$

即全体 $3\times 3$ 反对称矩阵。它与 $\mathbb{R}^3$ 同构。

### 4.2 Hat 与叉积

对 $\boldsymbol{\phi} = (\phi_1, \phi_2, \phi_3)^\top$，

$$
\boldsymbol{\phi}^\wedge = [\boldsymbol{\phi}]_\times =
\begin{pmatrix}
0 & -\phi_3 & \phi_2 \\
\phi_3 & 0 & -\phi_1 \\
-\phi_2 & \phi_1 & 0
\end{pmatrix}.
$$

核心恒等式（**必须熟练**）：

$$
[\boldsymbol{\phi}]_\times \mathbf{a} = \boldsymbol{\phi} \times \mathbf{a}.
$$

VINS 中 `Utility::skewSymmetric(q)` 就是 $[\mathbf{q}]_\times$。

### 4.3 李括号

对 $\boldsymbol{\phi}, \boldsymbol{\delta} \in \mathbb{R}^3$，

$$
[\boldsymbol{\phi}, \boldsymbol{\delta}]^\wedge = \boldsymbol{\phi}^\wedge \boldsymbol{\delta}^\wedge - \boldsymbol{\delta}^\wedge \boldsymbol{\phi}^\wedge = [\boldsymbol{\phi} \times \boldsymbol{\delta}]_\times.
$$

---

## 5. 指数映射与 Rodrigues 公式

### 5.1 定义

指数映射把切空间小旋转映到群上：

$$
\exp(\boldsymbol{\phi}^\wedge) = R \in \mathrm{SO}(3).
$$

### 5.2 Rodrigues 公式（必背）

令 $\theta = \|\boldsymbol{\phi}\|$，$\mathbf{n} = \boldsymbol{\phi}/\theta$（$\theta \to 0$ 时取一阶近似），

$$
\exp(\boldsymbol{\phi}^\wedge) = I + \frac{\sin\theta}{\theta}[\boldsymbol{\phi}]_\times + \frac{1-\cos\theta}{\theta^2}[\boldsymbol{\phi}]_\times^2.
$$

**小角度近似**（VINS `deltaQ` 用的就是这个）：

$$
\exp(\boldsymbol{\phi}^\wedge) \approx I + [\boldsymbol{\phi}]_\times \approx \text{四元数 } \bigl(1,\ \tfrac{\phi_1}{2},\ \tfrac{\phi_2}{2},\ \tfrac{\phi_3}{2}\bigr).
$$

对应代码：

```cpp
// utility.h — 小角度四元数，等价于 exp(phi^) 的一阶近似
dq.w() = 1.0;
dq.vec() = theta / 2.0;
```

### 5.3 物理含义

若 $\boldsymbol{\phi} = \boldsymbol{\omega}\, \Delta t$，则 $\exp(\boldsymbol{\phi}^\wedge)$ 是**角速度 $\boldsymbol{\omega}$ 在 $\Delta t$ 内产生的旋转增量**。IMU 预积分里 `result_delta_q = delta_q * Quaterniond(1, un_gyr*dt/2)` 正是中值积分下的旋转增量。

---

## 6. 对数映射与角度轴

### 6.1 定义

对 $R \in \mathrm{SO}(3)$，$\log(R) = \boldsymbol{\phi} \in \mathbb{R}^3$ 满足 $\exp(\boldsymbol{\phi}^\wedge) = R$。

### 6.2 常用公式

由 $\mathrm{tr}(R) = 1 + 2\cos\theta$，

$$
\theta = \arccos\!\left(\frac{\mathrm{tr}(R)-1}{2}\right),\qquad
\boldsymbol{\phi} = \frac{\theta}{2\sin\theta}(R - R^\top)^\vee.
$$

工程上常用 Eigen 的 `AngleAxisd aa(R); aa.angle() * aa.axis()`。

### 6.3 预积分里的相对旋转误差

初始化对齐陀螺 bias 时出现（`initial_aligment.cpp`）：

$$
\mathbf{r}_R = 2\,\bigl(\Delta \bar{q}^{-1} \otimes q_{ij}\bigr)_{vec}
$$

这是把四元数误差映到 $\mathbb{R}^3$ 的常见形式，小角度下与 $\log(\Delta \bar{R}^{-1} R_{ij})$ 等价。

---

## 7. SE(3)：刚体位姿群

### 7.1 定义

$$
\mathrm{SE}(3) = \left\{\, T =
\begin{pmatrix} R & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{pmatrix}
\;\Big|\; R \in \mathrm{SO}(3),\ \mathbf{p} \in \mathbb{R}^3 \,\right\}.
$$

### 7.2 作用在点上

齐次坐标 $\bar{\mathbf{p}} = (\mathbf{p}^\top, 1)^\top$，

$$
\bar{\mathbf{p}}' = T\,\bar{\mathbf{p}}, \qquad \text{即} \quad \mathbf{p}' = R\mathbf{p} + \mathbf{t}.
$$

VINS 重投影链（`projection_factor.cpp`）：

$$
\mathbf{p}_{c_j} = R_{ic}^\top \bigl( R_j^\top (R_i \mathbf{p}_{imu,i} + \mathbf{p}_i - \mathbf{p}_j) - \mathbf{t}_{ic} \bigr),
$$

就是多次 $\mathrm{SE}(3)$ 复合。

### 7.3 乘法与逆

$$
T_{ab} T_{bc} = T_{ac}, \qquad
T^{-1} = \begin{pmatrix} R^\top & -R^\top \mathbf{p} \\ \mathbf{0}^\top & 1 \end{pmatrix}.
$$

---

## 8. se(3)：位姿的李代数

### 8.1 Hat 映射

对 $\boldsymbol{\xi} = (\boldsymbol{\rho}^\top, \boldsymbol{\phi}^\top)^\top \in \mathbb{R}^6$，

$$
\boldsymbol{\xi}^\wedge =
\begin{pmatrix}
[\boldsymbol{\phi}]_\times & \boldsymbol{\rho} \\
\mathbf{0}^\top & 0
\end{pmatrix} \in \mathfrak{se}(3).
$$

### 8.2 指数映射（闭式）

记 $\boldsymbol{\phi}$ 的 Rodrigues 结果为 $R = \exp(\boldsymbol{\phi}^\wedge)$，定义

$$
J = \int_0^1 \exp\bigl((1-\alpha)\boldsymbol{\phi}^\wedge\bigr)\, d\alpha
= I + \frac{1-\cos\theta}{\theta^2}[\boldsymbol{\phi}]_\times + \frac{\theta - \sin\theta}{\theta^3}[\boldsymbol{\phi}]_\times^2
$$

（$\theta \to 0$ 时 $J \to I$），则

$$
\exp(\boldsymbol{\xi}^\wedge) =
\begin{pmatrix}
R & J\boldsymbol{\rho} \\
\mathbf{0}^\top & 1
\end{pmatrix}.
$$

**小角度**：若 $\boldsymbol{\rho} = \mathbf{v}\,\Delta t$，$\boldsymbol{\phi} = \boldsymbol{\omega}\,\Delta t$，则 $J \approx I$，平移增量 $\approx \mathbf{v}\,\Delta t$ — 与 IMU 运动学一致。

### 8.3 你需要到什么程度

- **必须**：知道 $\mathrm{SE}(3)$ 是 $R$ 与 $\mathbf{p}$ 的组合，优化时用 6 维增量；
- **建议**：知道 $\exp(\boldsymbol{\xi}^\wedge)$ 的块结构（上面公式），读 Forster 预积分论文时对照；
- **可后读**：se(3) 对数映射完整级数展开。

---

## 9. 复合、逆与伴随

### 9.1 复合的切空间传播

设 $T_{ac} = T_{ab} T_{bc}$。对 $T_{ab}$ 做右扰动 $\exp(\boldsymbol{\xi}_{ab}^\wedge)$，一阶有

$$
\log(T_{ac}' T_{ac}^{-1}) \approx \boldsymbol{\xi}_{ac} \approx \boldsymbol{\xi}_{ab} + \mathrm{Ad}_{T_{ab}}\,\boldsymbol{\xi}_{bc}.
$$

### 9.2 伴随 $\mathrm{Ad}_T$

对 $T = (R, \mathbf{p})$，

$$
\mathrm{Ad}_T =
\begin{pmatrix}
R & \mathbf{0} \\
[\mathbf{p}]_\times R & R
\end{pmatrix} \in \mathbb{R}^{6\times 6}.
$$

**用途**：预积分 Jacobian 的链式法则；把 $b$ 系下的扰动变到 $w$ 系。

### 9.3 旋转的伴随

在 $\mathrm{SO}(3)$ 上，$\mathrm{Ad}_R = R$（作用在 $\mathbb{R}^3$ 轴角上：$R \exp(\boldsymbol{\phi}^\wedge) R^\top = \exp((R\boldsymbol{\phi})^\wedge)$）。

---

## 10. 扰动模型：优化的核心

### 10.1 为什么不能「$R$ 的三个元素 + $\delta$」

Gauss–Newton 需要 $\mathbf{x} \oplus \delta$ 与雅可比 $\partial \mathbf{r}/\partial \delta$。对 $R \in \mathrm{SO}(3)$，$\oplus$ 必须是群上的乘法，不是加法。

### 10.2 右扰动（VINS / Barfoot 主流约定）

**旋转**：

$$
R \leftarrow R \exp([\boldsymbol{\phi}]_\times), \qquad \boldsymbol{\phi} \in \mathbb{R}^3.
$$

**位姿**（右扰动 on $T$）：

$$
T \leftarrow T \exp(\boldsymbol{\xi}^\wedge).
$$

**平移在 VINS 里的工程做法**：存储 $\mathbf{p} \in \mathbb{R}^3$，更新 $\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$。这不是严格的 $\mathrm{SE}(3)$ 右扰动（平移部分被简化为欧式加法），但在小增量下与 $J\boldsymbol{\rho}$ 等价，且实现简单。读代码时要意识到这一点。

### 10.3 左扰动（了解即可）

$$
R \leftarrow \exp([\boldsymbol{\phi}]_\times)\, R.
$$

左、右扰动雅可比差一个 $\mathrm{Ad}$ 变换。**同一篇论文、同一份代码必须全程用一种**；VINS 视觉因子雅可比按右扰动 + 平移加法推导。

### 10.4 Ceres 中的 LocalParameterization

Ceres 优化变量是 7 维 `[px, py, pz, qx, qy, qz, qw]`，但切空间是 6 维。`Plus(x, delta, x_plus_delta)` 实现：

```cpp
p = _p + dp;                    // 平移：欧式加法
q = (_q * dq).normalized();     // 旋转：右乘小四元数
```

`ComputeJacobian` 返回 $7 \times 6$ 的 $[\,I_6;\ \mathbf{0}\,]$：表示前 6 维切空间与「名义上」7 维存储的局部线性化关系（四元数冗余维由约束隐式处理）。

---

## 11. 必会雅可比：推导套路

目标：给定误差 $\mathbf{r}(\mathbf{x})$，求 $\partial \mathbf{r}/\partial \boldsymbol{\xi}$，其中 $\boldsymbol{\xi} \in \mathbb{R}^6$ 是位姿切空间增量。

### 11.1 套路总览

1. 写出**不含增量**的 $\mathbf{r}(T)$；
2. 插入右扰动 $T' = T \exp(\boldsymbol{\xi}^\wedge)$，一阶展开 $\exp(\boldsymbol{\xi}^\wedge) \approx I + \boldsymbol{\xi}^\wedge$；
3. 用 $[\mathbf{a}]_\times \mathbf{b} = -\mathbf{a} \times \mathbf{b}$ 整理成 $\mathbf{r}' \approx \mathbf{r} + \mathbf{J}\boldsymbol{\xi}$；
4. 或用**扰动法**：$\partial \mathbf{r}/\partial \boldsymbol{\xi} \approx (\mathbf{r}(\boldsymbol{\xi}+\epsilon) - \mathbf{r}(\boldsymbol{\xi}))/\epsilon$，与手推对照（有限差分调试）。

### 11.2 点变换 $\mathbf{p}_c = R\mathbf{p}_b + \mathbf{t}$

对 $T = (R, \mathbf{t})$ 右扰动，令 $\mathbf{p}_b$ 固定，

$$
\mathbf{p}_c' \approx R\mathbf{p}_b + \mathbf{t} + \delta\mathbf{p} + R[\boldsymbol{\phi}]_\times \mathbf{p}_b
= \mathbf{p}_c + \delta\mathbf{p} - R[\mathbf{p}_b]_\times \boldsymbol{\phi}.
$$

故

$$
\frac{\partial \mathbf{p}_c}{\partial \boldsymbol{\xi}} =
\begin{bmatrix}
I & -R[\mathbf{p}_b]_\times
\end{bmatrix} \in \mathbb{R}^{3\times 6}.
$$

**记忆**：旋转部分总是 $-R[\mathbf{p}]_\times$（右扰动、点在本体系下固定）。

### 11.3 重投影误差

归一化平面观测（VINS `projection_factor` 非 `UNIT_SPHERE` 分支）：

$$
\mathbf{u} = \pi(\mathbf{p}_c) = \begin{pmatrix} x/z \\ y/z \end{pmatrix}, \qquad
\mathbf{r} = \mathbf{u} - \mathbf{u}_{obs}.
$$

链式法则：

$$
\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}} =
\underbrace{\frac{\partial \pi}{\partial \mathbf{p}_c}}_{2\times 3}
\underbrace{\frac{\partial \mathbf{p}_c}{\partial \boldsymbol{\xi}}}_{3\times 6}
= \text{reduce} \cdot
\begin{bmatrix} R_{ic}^\top R_j^\top & R_{ic}^\top R_j^\top R_i (-[\mathbf{p}_{imu,i}]_\times) \end{bmatrix}.
$$

代码里 `reduce` 即 $\partial \pi / \partial \mathbf{p}_c$（除以 $z$ 的透视除法雅可比），`jaco_i.rightCols<3>()` 里的 `skewSymmetric(pts_imu_i)` 即 $-[\mathbf{p}]_\times$ 块。

### 11.4 旋转复合 $R_{wc} = R_{wi} R_{ic}$

对 $R_{wi}$ 右扰动：

$$
R_{wc}' \approx R_{wi}(I + [\boldsymbol{\phi}]_\times) R_{ic} = R_{wc} + R_{wi}[\boldsymbol{\phi}]_\times R_{ic}.
$$

用 $[\boldsymbol{\phi}]_\times R = -[R^\top \boldsymbol{\phi}]_\times R$ 可改写成对 $\boldsymbol{\phi}$ 的 $-[\cdot]_\times$ 形式。四元数乘法雅可用 `Qleft` / `Qright`（`utility.h`）系统化。

### 11.5 你需要手推几遍

| 序号 | 函数 | 建议 |
| --- | --- | --- |
| 1 | $\mathbf{p}_c = R\mathbf{p} + \mathbf{t}$ | 必推，所有视觉因子基础 |
| 2 | $\pi(R\mathbf{p}+\mathbf{t})$ | 必推，理解 `reduce` |
| 3 | $R_1 R_2$ 对 $R_1$ | 建议推，理解预积分 |
| 4 | $\exp(\boldsymbol{\xi})$ 对 $\boldsymbol{\xi}$ | 读 Forster 时再推 |

---

## 12. 四元数与 SO(3) 的双覆盖

### 12.1 定义

单位四元数 $q = (w, \mathbf{v})$，$\|q\|=1$。与轴角关系：

$$
q = \left(\cos\frac{\theta}{2},\ \mathbf{n}\sin\frac{\theta}{2}\right).
$$

### 12.2 双覆盖

$q$ 与 $-q$ 表示**同一**旋转。优化中若 $q$ 跳变符号但 $R$ 不变，属正常现象；`positify` 可固定半球，VINS 里大多直接归一化。

### 12.3 乘法与旋转作用

- 旋转复合：$q_{ac} = q_{ab} \otimes q_{bc}$（Hamilton 积，注意 Eigen 顺序与论文一致）；
- 旋转向量：$\mathbf{p}' = q \otimes (0, \mathbf{p}) \otimes q^*$。

VINS 预积分：`result_delta_q = delta_q * dq`（右乘增量）。

### 12.4 Qleft / Qright

四元数 $q \otimes p$ 可写成 $4\times 4$ 矩阵乘 4 维向量：

$$
q \otimes p = Q_{\mathrm{left}}(q)\, p = Q_{\mathrm{right}}(p)\, q.
$$

预积分误差对四元数的 Jacobian 常出现 `Qleft`、`Qright`（见 `integration_base.h` 中 `evaluate`）。

---

## 13. 与 VINS-Mono 代码对照

### 13.1 文件地图

```
VINS-Mono/vins_estimator/src/
├── utility/utility.h              # skewSymmetric, deltaQ, Qleft, Qright
├── factor/pose_local_parameterization.cpp   # 位姿 7D 存储 + 6D 更新
├── factor/integration_base.h      # IMU 预积分，exp 的离散实现
├── factor/projection_factor.cpp   # 重投影 + 李代数雅可比
├── factor/imu_factor.h            # 预积分残差对状态
└── estimator.cpp                  # processIMU 中值积分传播
```

### 13.2 processIMU：运动学 + 小旋转

```cpp
Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
Vs[j] += dt * un_acc;
```

- 旋转：$\Delta R \approx \exp([\boldsymbol{\omega}]\times \Delta t)$；
- 平移/速度：欧式积分（重力、bias 在 `un_acc` 里）。

### 13.3 预积分 midPointIntegration

在**相对**增量 `(delta_p, delta_q, delta_v)` 上累积，而不是每步都在世界系重算 — 这是 Forster 预积分的离散版。Jacobian 块里的 `[w_x]_\times`、`[a_x]_\times` 来自对 $\exp$ 的一阶扰动链式法则。

### 13.4 读代码顺序建议

1. `skewSymmetric`、`deltaQ` — 10 分钟；
2. `PoseLocalParameterization::Plus` — 理解 6D→7D 更新；
3. `projection_factor.cpp` 的 `Evaluate` — 对照 §11.3；
4. `integration_base.h` 的 `midPointIntegration` — 与预积分讲义 §9（世界系 `processIMU`）和 §14（`delta_q` 中值）对照；
5. `imu_factor.h` 的残差定义 — 读预积分讲义 §17–§18 后回来；雅可比见该讲义附录 B。

---

## 14. 掌握清单与阅读顺序

### 14.1 必须掌握（读懂 VINS 的最低线）

- [ ] $\mathrm{SO}(3)$、$\mathrm{SE}(3)$ 定义与群乘法
- [ ] $[\boldsymbol{\phi}]_\times$ 与叉积等价
- [ ] Rodrigues 公式及小角度近似
- [ ] 右扰动 $R \leftarrow R\exp([\boldsymbol{\phi}]_\times)$
- [ ] 点变换雅可比 $\begin{bmatrix} I & -R[\mathbf{p}]_\times \end{bmatrix}$
- [ ] 四元数表示旋转、右乘更新
- [ ] 伴随 $\mathrm{Ad}_T$ 的块形式（知道用途即可）

### 14.2 建议掌握（读预积分论文）

- [ ] $\exp(\boldsymbol{\xi}^\wedge)$ 的 $R$ 与 $J\boldsymbol{\rho}$ 块
- [ ] 预积分相对旋转、速度、位置误差的 $\mathbb{R}^3$ / $\mathbb{R}^6$ 形式
- [ ] $Q_{\mathrm{left}}$ / $Q_{\mathrm{right}}$ 四元数 Jacobian

### 14.3 可暂缓

- $\mathrm{Sim}(3)$ 完整推导（单目尺度 + 回环）
- 左扰动全套公式
- 李群上 Gauss–Newton 的收敛性证明

### 14.4 外部材料（按优先级）

| 优先级 | 材料 | 章节 |
| --- | --- | --- |
| 1 | Solà, [A micro Lie theory](https://arxiv.org/abs/1812.01537) | 全文 |
| 2 | Barfoot, *State Estimation for Robotics* | 第 6–7 章 |
| 3 | Forster et al., 预积分 TRO 2017 | 先读 [imu_preintegration.md](imu_preintegration.md)，再看论文附录 |
| 辅助 | 十四讲第 2–4 讲 | 建立直觉 |

### 14.5 动手练习

1. **Python/NumPy**：实现 `hat`、`exp_so3`（Rodrigues）、`log_so3`，与 `scipy.spatial.transform.Rotation` 互验。
2. **有限差分**：对 $\mathbf{p}_c = R\mathbf{p} + \mathbf{t}$ 求 $\partial \mathbf{p}_c / \partial \boldsymbol{\xi}$，与 §11.2 手推结果对比，误差应 $<10^{-6}$。
3. **最小 BA**：2 帧、1 个点，Ceres 优化 `PoseLocalParameterization`，只含重投影因子。

---

## 15. 自测

1. 证明 $\mathrm{SO}(3)$ 对矩阵乘法构成群（封闭性、单位元、逆）。
2. 验证 $[\mathbf{a}]_\times \mathbf{b} = \mathbf{a} \times \mathbf{b}$，并写出 $[\mathbf{a}]_\times$ 的显式矩阵。
3. 用 Rodrigues 公式：$\boldsymbol{\phi} = (0, 0, \pi/2)^\top$，手算 $R = \exp(\boldsymbol{\phi}^\wedge)$。
4. 对 $\mathbf{p}' = R\mathbf{p} + \mathbf{t}$，推导右扰动下 $\partial \mathbf{p}' / \partial \boldsymbol{\xi}$（§11.2）。
5. 为什么欧拉角不适合做优化变量？举万向节锁或约束破坏的例子。
6. 四元数 $q$ 与 $-q$ 为何对应同一 $R$？对优化有何影响？
7. VINS 里平移用加法、旋转用右乘四元数 — 这与严格 $\mathrm{SE}(3)$ 右扰动差在哪？小增量下为何可接受？
8. 写出 $\mathrm{Ad}_T$ 的 $6\times 6$ 分块矩阵，并说明预积分 Jacobian 链里为何出现 $[\mathbf{p}]_\times R$。
9. 解释 `deltaQ` 与 `exp([θ]_×)` 的关系；何时一阶近似不够？
10. 打开 `projection_factor.cpp`，指出 `jaco_i.rightCols<3>()` 对应 §11 哪一项。

---

## 16. 自测参考答案

**1.** 旋转矩阵乘积正交且 $\det(RS)=\det R \det S = 1$；$I \in \mathrm{SO}(3)$；$R^{-1} = R^\top \in \mathrm{SO}(3)$。

**2.** 直接展开矩阵乘法即得。显式形式见 §4.2。

**3.** 绕 $z$ 轴转 $\pi/2$：

$$
R =
\begin{pmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{pmatrix}.
$$

**4.** 见 §11.2：$\begin{bmatrix} I & -R[\mathbf{p}]_\times \end{bmatrix}$。

**5.** 欧拉角 $(\psi,\theta,\phi)$ 在 $\theta = \pm\pi/2$ 处 gimbal lock，雅可比降秩；且 $(\alpha,\beta,\gamma)$ 与 $(\alpha+\pi, \pi-\beta, \gamma+\pi)$ 可对应同一 $R$，不唯一。加法更新不保持正交性。

**6.** 四元数 $q$ 与 $-q$ 在映射 $q \mapsto R$ 中成对；优化轨迹可能在 $q$ 空间走捷径穿过 $q \to -q$，但 $R$ 连续。归一化后残差不变；必要时 `positify` 固定 $w \ge 0$。

**7.** 严格右扰动 $\exp(\boldsymbol{\xi}^\wedge)$ 的平移块是 $J\boldsymbol{\rho}$，不是简单 $\delta\mathbf{p}$。VINS 把平移与旋转 decouple：$\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$，$R \leftarrow R\exp([\boldsymbol{\phi}]_\times)$。当 $\|\boldsymbol{\phi}\| \ll 1$ 且步长小时，与真实 se(3) 右扰动一阶一致；大旋转步长时 Ceres 会拆小步迭代。

**8.**

$$
\mathrm{Ad}_T =
\begin{pmatrix}
R & 0 \\
[\mathbf{p}]_\times R & R
\end{pmatrix}.
$$

预积分把 IMU 系下增量传到世界系或相邻帧时，速度/位置扰动需经 $R$ 与杠杆臂 $[\mathbf{p}]_\times$ 修正 — 即 Forster 论文中 $\mathrm{Ad}_{T_{ij}}$ 的传播。

**9.** `deltaQ(θ)` 设 $q = (1, \theta/2)$，归一化后 $\approx (\cos\|\theta\|/2, \frac{\theta}{\|\theta\|}\sin\|\theta\|/2)$，Taylor 展开 $w \approx 1 - \|\theta\|^2/8$，向量部 $\approx \theta/2$，对应 $\exp([\theta]_\times)$ 到二阶。$\|\theta\| > 10°$ 量级时建议用完整 Rodrigues 或 Eigen `AngleAxisd`。

**10.** `jaco_i.rightCols<3>()` = `ric.transpose() * Rj.transpose() * Ri * (-skewSymmetric(pts_imu_i))`，即链式法则中 $\partial \mathbf{p}_{c_j}/\partial \boldsymbol{\phi}_i$ 的 $-R[\mathbf{p}]_\times$ 块，再左乘 $\partial\pi/\partial\mathbf{p}_{c_j}$（`reduce`）。

---

## 下一步

- 相机模型与投影：[`Camera/pinhole_distortion.md`](../Camera/pinhole_distortion.md)
- IMU 预积分（Forster / VINS 中值积分）：[`imu_preintegration.md`](imu_preintegration.md)
- SLAM 总路径：[`learning_path.md`](learning_path.md) 阶段 0–3
- 读完本文 + 预积分讲义后，对照 VINS 论文 Section III–V 与 `factor/imu_factor.h`
