# VINS-Mono：公式与代码对照

自学讲义。面向已经能看懂旋转矩阵、最小二乘和针孔模型的读者。本文以本仓库 [`VINS-Mono`](../../VINS-Mono/) 为对象，按一次图像帧从进入系统到写出位姿的顺序，把论文里的公式落到具体函数上。

配套材料，本文不重复推导：

- 亮度恒定、Lucas–Kanade、金字塔与 Shi–Tomasi：见 [光流法讲义](optical_flow.md)。
- IMU 连续运动方程、中值积分、协方差递推 $F,V$：见 [IMU 预积分讲义](imu_preintegration.md)。
- 右扰动、$\delta\mathbf{q}$、四元数左乘矩阵：见 [李群讲义](lie_groups.md)。
- 去畸变与 `liftProjective`：见 [针孔与畸变讲义](../Camera/pinhole_distortion.md)。

读完应能：

- 指出三个 ROS 节点各做什么，一条特征从像素变成滑窗里的逆深度；
- 写出滑窗状态，并说出 Ceres 里每个参数块的维度和内存布局；
- 从两帧位姿推出 2 维重投影残差，对照 `ProjectionFactor::Evaluate`；
- 说明初始化为什么先标定陀螺 bias，再线性解速度、重力和尺度；
- 说明滑窗为什么有时丢掉最老帧、有时丢掉次新帧，以及边缘化先验怎么变成下一次的残差；
- 说明回环只优化 4 个自由度的原因。

公式以 Qin 等人 *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*（IEEE TRO 2018）为准，符号改写成和代码一致的写法。代码行号对应当前仓库里的实现。

---

## 目录

1. [系统在估什么](#1-系统在估什么)
2. [三个节点与一帧的数据流](#2-三个节点与一帧的数据流)
3. [状态、坐标系、参数块](#3-状态坐标系参数块)
4. [前端：光流、外点、归一化坐标](#4-前端光流外点归一化坐标)
5. [图像进估计器：关键帧判定与 IMU 传播](#5-图像进估计器关键帧判定与-imu-传播)
6. [IMU 预积分与 15 维残差](#6-imu-预积分与-15-维残差)
7. [初始化](#7-初始化)
8. [滑窗优化与视觉残差](#8-滑窗优化与视觉残差)
9. [边缘化](#9-边缘化)
10. [回环与 4 自由度位姿图](#10-回环与-4-自由度位姿图)
11. [失败检测](#11-失败检测)
12. [建议的阅读顺序](#12-建议的阅读顺序)

---

## 1. 系统在估什么

单目相机只能给出无尺度的方向。IMU 能给出短时间的旋转和比力，但比力里混着重力，加速度计和陀螺仪都有随时间缓慢变化的零偏。VINS-Mono 把两类测量写进同一个滑窗，同时估计：

- 每一帧 IMU 在世界系下的位置、姿态、速度；
- 加速度计零偏、陀螺仪零偏；
- 相机到 IMU 的外参（可选）；
- 每个特征在**第一次被观测的相机系**里的逆深度；
- 相机与 IMU 的时间偏移 $t_d$（可选）。

世界系不是任意的。初始化结束时，重力被旋到 $+z$，第一帧的航向被旋到 0。之后滚转和俯仰由重力约束，长期漂移主要留在位置和航向上。这就是后面回环只做 4 自由度优化的原因。

滑窗长度是编译期常量：

```11:14:VINS-Mono/vins_estimator/src/parameters.h
const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
```

窗口里始终有 $11$ 个位姿，下标 $0,\ldots,10$。`WINDOW_SIZE` 是最老帧到最新帧的间隔，不是“帧的个数”。

---

## 2. 三个节点与一帧的数据流

系统拆成三个 ROS 节点，估计器和回环故意分开，这样里程计可以 10–30 Hz 跑，回环慢一点也不堵住前端。

| 节点 | 入口 | 输入 | 输出 |
| --- | --- | --- | --- |
| `feature_tracker` | `feature_tracker_node.cpp` | 原始图像 | `/feature_tracker/feature`，每个点一条观测 |
| `vins_estimator` | `estimator_node.cpp` | 特征点云 + IMU | 滑窗位姿、速度、bias、点云 |
| `pose_graph` | `pose_graph_node.cpp` | 关键帧图像与位姿 | DBoW2 回环、4-DoF 位姿图、漂移修正 |

一帧图像的路径：

1. `FeatureTracker::readImage` 做金字塔 LK 光流，RANSAC 剔外点，补新角点，再 `liftProjective` 得到归一化平面坐标和像素速度。
2. 只发布跟踪次数大于 1 的点。每个点是 `geometry_msgs::Point32` 的 $(x,y,1)$，五个 channel 依次是 id、像素 $u$、像素 $v$、速度 $v_x$、速度 $v_y$。
3. `feature_callback` 把它们收成 `map<id, vector<pair<camera_id, Matrix<7,1>>>>`。7 维向量是

$$
[x,\; y,\; z{=}1,\; u,\; v,\; v_x,\; v_y].
$$

4. `process()` 把这张图像时刻之前的 IMU 全部吃掉。图像时刻用的是 $t_{\mathrm{img}}+t_d$。若某条 IMU 落在图像时刻之后，就在 $t_{\mathrm{img}}+t_d$ 上做一次线性插值，只把插值前的那一段送进 `processIMU`。
5. `processImage` 决定这帧是关键帧还是要被丢掉的次新帧，然后要么做初始化，要么 `solveOdometry`（三角化 + Ceres）再滑窗。
6. 关键帧还会送给 `pose_graph`。回环成功后，位姿图算出漂移 $(R_{\mathrm{drift}}, \mathbf{t}_{\mathrm{drift}})$，估计器输出在发布前乘上这个漂移。

插值权重在 `estimator_node.cpp` 的 `process()`：

```247:262:VINS-Mono/vins_estimator/src/estimator_node.cpp
double dt_1 = img_t - current_time;
double dt_2 = t - img_t;
current_time = img_t;
double w1 = dt_2 / (dt_1 + dt_2);
double w2 = dt_1 / (dt_1 + dt_2);
dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
// ... 角速度同样 ...
estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
```

这里 $w_1$ 是离图像时刻更远的那一侧 IMU 的权重，所以插值结果落在图像时刻，而不是落在后一条 IMU 上。

---

## 3. 状态、坐标系、参数块

### 3.1 滑窗状态

论文里的状态写成

$$
\begin{aligned}
\mathcal{X}
&=
\big[\mathbf{x}_0,\ldots,\mathbf{x}_n,\;
\mathbf{x}^b_c,\;
\lambda_1,\ldots,\lambda_m,\;
t_d\big],\\
\mathbf{x}_k
&=
\big[
\mathbf{p}^w_{b_k},\;
\mathbf{q}^w_{b_k},\;
\mathbf{v}^w_{b_k},\;
\mathbf{b}_{a},\;
\mathbf{b}_{g}
\big],\\
\mathbf{x}^b_c
&=
\big[
\mathbf{p}^b_c,\;
\mathbf{q}^b_c
\big].
\end{aligned}
$$

代码里同一组量拆开放：

| 符号 | 成员 | 含义 |
| --- | --- | --- |
| $\mathbf{p}^w_{b_k},\mathbf{R}^w_{b_k}$ | `Ps[k]`, `Rs[k]` | 第 $k$ 帧 IMU 在世界系的位置和旋转 |
| $\mathbf{v}^w_{b_k}$ | `Vs[k]` | 世界系速度 |
| $\mathbf{b}_a,\mathbf{b}_g$ | `Bas[k]`, `Bgs[k]` | 这一帧的加计、陀螺零偏。模型假设相邻帧之间零偏随机游走，所以每帧各存一份 |
| $\mathbf{p}^b_c,\mathbf{R}^b_c$ | `tic[0]`, `ric[0]` | 相机在 IMU 系下的平移和旋转。$NUM\_OF\_CAM=1$ |
| $\lambda$ | `FeaturePerId::estimated_depth` 的倒数 | 逆深度，定义在该特征**第一次出现**的相机系 |
| $t_d$ | `td` | 相机时间相对 IMU 时间的偏移，单位秒 |

`SolverFlag` 只有两个值：`INITIAL` 和 `NON_LINEAR`。窗口攒满且 `initialStructure()` 成功之后才进入非线性优化。在那之前，`processIMU` 仍会积分，但那些位姿只是传播出来的初值，没有被视觉残差修正。

### 3.2 三个坐标系之间怎么变

记 IMU 到相机的外参为 $R^b_c=\texttt{ric}$、$\mathbf{p}^b_c=\texttt{tic}$。一个写在相机系、深度为 $1/\lambda$ 的点，变到世界系再变回另一帧相机系，就是后面视觉残差的整条链：

$$
\begin{aligned}
\mathbf{P}^{c_i}
&=
\frac{1}{\lambda}\begin{bmatrix}x\\ y\\ 1\end{bmatrix},\\
\mathbf{P}^{b_i}
&=
R^b_c\,\mathbf{P}^{c_i}+\mathbf{p}^b_c,\\
\mathbf{P}^{w}
&=
R^w_{b_i}\,\mathbf{P}^{b_i}+\mathbf{p}^w_{b_i},\\
\mathbf{P}^{b_j}
&=
{R^w_{b_j}}^{\!\top}\big(\mathbf{P}^{w}-\mathbf{p}^w_{b_j}\big),\\
\mathbf{P}^{c_j}
&=
{R^b_c}^{\!\top}\big(\mathbf{P}^{b_j}-\mathbf{p}^b_c\big).
\end{aligned}
$$

归一化观测是 $\mathbf{P}^{c}/Z$，不是像素。畸变已经在前端去掉了。

### 3.3 送进 Ceres 的内存布局

`vector2double()` 把上面的 Eigen 量拷进裸数组。位姿不是 6 维，而是 7 维，因为四元数有 4 个分量，优化时再用局部参数化压回 6 维。

```46:47:VINS-Mono/vins_estimator/src/parameters.h
SIZE_POSE = 7,
SIZE_SPEEDBIAS = 9,
```

`para_Pose[k]` 的 7 个数是

$$
[p_x,\; p_y,\; p_z,\; q_x,\; q_y,\; q_z,\; q_w].
$$

Eigen 的 `Quaterniond` 构造函数参数顺序是 $(w,x,y,z)$，所以每个因子里都写成

```cpp
Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
```

也就是第 6 个元素是 $w$。读雅可比时如果把 $w$ 当成 $x$，后面所有旋转导数都会错一格。

`para_SpeedBias[k]` 是 $[v_x,v_y,v_z,b_{ax},b_{ay},b_{az},b_{gx},b_{gy},b_{gz}]$，欧氏空间，没有局部参数化。

`para_Feature[i]` 只有一个数：逆深度 $\lambda$。

局部参数化是右扰动。`PoseLocalParameterization::Plus`：

$$
\mathbf{p}\leftarrow \mathbf{p}+\delta\mathbf{p},\qquad
\mathbf{q}\leftarrow \mathbf{q}\otimes \delta\mathbf{q}(\delta\boldsymbol{\theta}),\qquad
\delta\mathbf{q}(\delta\boldsymbol{\theta})
=
\begin{bmatrix}1\\ \delta\boldsymbol{\theta}/2\end{bmatrix}.
$$

`ComputeJacobian` 返回 $7\times 6$ 矩阵，上面 $6\times 6$ 是单位阵，最后一行是 0。Ceres 因此知道：全局状态 7 维，切空间增量只有 6 维，四元数的第 4 个分量不独立。

---

## 4. 前端：光流、外点、归一化坐标

前端不做描述子匹配。VINS-Mono 的短时关联靠光流，描述子（BRIEF）只在回环节点里用。Lucas–Kanade 的推导、金字塔和角点准则见 [光流法讲义](optical_flow.md)。本节只按 `FeatureTracker::readImage` 的调用顺序往下看。

`FeatureTracker::readImage` 的顺序：

1. 可选 CLAHE，把对比度拉平，弱纹理时角点更稳。
2. `cv::calcOpticalFlowPyrLK`，窗口 $21\times 21$，金字塔 3 层，把上一帧的点跟到当前帧。出界的点直接标失败。
3. `rejectWithF`：先把像素 `liftProjective` 到归一化平面，再乘一个虚拟焦距 $460$ 投回一张假想的无畸变图像，然后 `findFundamentalMat`（RANSAC）剔掉不满足对极约束的点。虚拟焦距只服务于 RANSAC 的像素阈值，不进入估计器。
4. `setMask`：按跟踪次数从长到短把点画进 mask，已有点周围 `MIN_DIST` 像素内不再检测。跟踪得久的点优先保留。
5. 不够 `MAX_CNT` 时，`goodFeaturesToTrack` 补 Shi-Tomasi 角点。
6. `undistortedPoints`：再次 `liftProjective`，存 $(x/z,\, y/z)$。若上一帧也有同一 id，像素速度是

$$
v_x=\frac{x_k-x_{k-1}}{\Delta t},\qquad
v_y=\frac{y_k-y_{k-1}}{\Delta t}.
$$

新点没有上一帧，速度记 0。`feature_callback` 会丢掉系统收到的第一帧特征，因为那一帧的速度还不可用。

跟踪次数 `track_cnt > 1` 才发布。只出现一帧的点既不能三角化，也不能提供视差，送进估计器只会占 id。

归一化坐标进估计器之后，视觉残差就在这个平面上计算，不再回到像素。`FOCAL_LENGTH / 1.5` 会在后面当作这条残差的信息矩阵平方根，把“归一化平面上的 1 个单位”换算成大约 $1.5$ 像素的不确定度。

---

## 5. 图像进估计器：关键帧判定与 IMU 传播

### 5.1 这帧留不留

`processImage` 第一件事是 `addFeatureCheckParallax`。返回真，`marginalization_flag = MARGIN_OLD`，这帧当关键帧，滑窗时丢掉最老的一帧。返回假，`marginalization_flag = MARGIN_SECOND_NEW`，视差不够，滑窗时丢掉**次新**帧，把最新帧留在窗口末尾。

判定用的是倒数第二帧和倒数第三帧之间的平均视差，不是最新帧和上一帧。代码在补偿旋转的那一行被注释掉了，实际就是归一化平面上的位移：

$$
\text{parallax}
=
\frac{1}{N}\sum_i
\big\|
\boldsymbol{\pi}(\mathbf{f}_i^{(k-2)})
-
\boldsymbol{\pi}(\mathbf{f}_i^{(k-1)})
\big\|.
$$

配置文件里的 `keyframe_parallax` 单位是像素，读入时除以 `FOCAL_LENGTH`：

```56:57:VINS-Mono/vins_estimator/src/parameters.cpp
MIN_PARALLAX = fsSettings["keyframe_parallax"];
MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
```

所以比较发生在归一化平面上。另外两条短路：窗口还不到 2 帧，或者这帧能跟到的旧点数少于 20，直接当成关键帧。跟踪崩了的时候宁可边缘化老帧，也不要把一个几乎没有共视的帧卡在窗口中间。

`MARGIN_OLD = 0`，`MARGIN_SECOND_NEW = 1`。日志里 `marginalization_flag ? "Non-keyframe" : "Keyframe"` 就是这个约定：非零表示次新帧策略。

### 5.2 两套积分，不要混

`processIMU` 对同一条 IMU 做两件不同的事。

**世界系前向传播**，只更新当前帧 `Ps[j], Rs[j], Vs[j]`，给下一轮优化当初值：

```107:114:VINS-Mono/vins_estimator/src/estimator.cpp
Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
Vs[j] += dt * un_acc;
```

对应的连续模型是

$$
\dot{\mathbf{R}}
=
\mathbf{R}\,[\boldsymbol{\omega}-\mathbf{b}_g]_\times,\qquad
\dot{\mathbf{v}}
=
\mathbf{R}(\mathbf{a}-\mathbf{b}_a)-\mathbf{G},\qquad
\dot{\mathbf{p}}=\mathbf{v}.
$$

`G` 是静止时加速度计在世界系应读到的比力。初始化把重力旋到 $+z$ 之后，`G = (0,0,g_norm)`。静止、零偏正确时，$\mathbf{R}(\mathbf{a}-\mathbf{b}_a)-\mathbf{G}=\mathbf{0}$，速度不再被积分拉跑。

**预积分**写在 `pre_integrations[frame_count]` 里，参考系是这一段起点的 IMU 系，**不含重力**。重力留在残差里，用两端位姿现算。这样 bias 或重力方向更新时，不必把几百个 IMU 样本重积一遍。`tmp_pre_integration` 是同一段数据的另一份拷贝，挂在 `ImageFrame` 上，初始化对齐时要用到窗口之外的每一帧，不只有 11 个关键帧。

中值一步（`IntegrationBase::midPointIntegration`）：

$$
\begin{aligned}
\boldsymbol{\omega}
&=
\frac{1}{2}(\tilde{\boldsymbol{\omega}}_k+\tilde{\boldsymbol{\omega}}_{k+1})-\mathbf{b}_g,\\
\mathbf{a}_k
&=
\Delta\mathbf{R}_k\,(\tilde{\mathbf{a}}_k-\mathbf{b}_a),\\
\Delta\mathbf{R}_{k+1}
&=
\Delta\mathbf{R}_k\,
\mathrm{Exp}(\boldsymbol{\omega}\,\Delta t),\\
\mathbf{a}_{k+1}
&=
\Delta\mathbf{R}_{k+1}\,(\tilde{\mathbf{a}}_{k+1}-\mathbf{b}_a),\\
\bar{\mathbf{a}}
&=
\frac{1}{2}(\mathbf{a}_k+\mathbf{a}_{k+1}),\\
\Delta\mathbf{p}
&\leftarrow
\Delta\mathbf{p}+\Delta\mathbf{v}\,\Delta t+\tfrac12\bar{\mathbf{a}}\,\Delta t^2,\\
\Delta\mathbf{v}
&\leftarrow
\Delta\mathbf{v}+\bar{\mathbf{a}}\,\Delta t.
\end{aligned}
$$

代码里的四元数增量是 $\mathrm{Quaterniond}(1,\, \omega_x\Delta t/2,\, \omega_y\Delta t/2,\, \omega_z\Delta t/2)$，这是 $\mathrm{Exp}(\boldsymbol{\omega}\Delta t)$ 的一阶形式，小角度下和指数映射一致。每步结束会 `normalize()`。

$F$ 和噪声矩阵 $V$ 的逐块形式、以及为什么 bias 改变时可以用雅可比一阶修正而不必重积分，见预积分讲义。本文从 `evaluate` 的 15 维残差接着写。

---

## 6. IMU 预积分与 15 维残差

### 6.1 预积分量与两端状态的关系

在 bias 等于线性化点 $(\mathbf{b}_a,\mathbf{b}_g)$ 时，两端状态应满足

$$
\begin{aligned}
\boldsymbol{\alpha}
&=
{R_i}^{\!\top}
\Big(
\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t+\tfrac12\mathbf{G}\Delta t^2
\Big),\\
\boldsymbol{\gamma}
&=
\mathbf{q}_i^{-1}\otimes \mathbf{q}_j,\\
\boldsymbol{\beta}
&=
{R_i}^{\!\top}
\big(
\mathbf{v}_j-\mathbf{v}_i+\mathbf{G}\Delta t
\big),\\
\mathbf{b}_{a,j}&=\mathbf{b}_{a,i},\qquad
\mathbf{b}_{g,j}=\mathbf{b}_{g,i}.
\end{aligned}
$$

$\boldsymbol{\alpha},\boldsymbol{\gamma},\boldsymbol{\beta}$ 就是 `delta_p, delta_q, delta_v`。注意位置式子里重力是 **加** $\tfrac12 G\Delta t^2$：因为前向传播用的是 $\mathbf{a}_{\mathrm{world}}=R(\mathbf{a}-\mathbf{b}_a)-\mathbf{G}$，把不含重力的 $\boldsymbol{\alpha}$ 乘回世界系时，重力项要单独加回去。移项之后，$R_i^\top$ 作用在 $(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t+\tfrac12 G\Delta t^2)$ 上。

### 6.2 bias 的一阶修正

优化里的 $\mathbf{b}_{a,i}$ 会离开当初积分用的 `linearized_ba`。记

$$
\delta\mathbf{b}_a=\mathbf{b}_{a,i}-\mathbf{b}_a^{\mathrm{lin}},\qquad
\delta\mathbf{b}_g=\mathbf{b}_{g,i}-\mathbf{b}_g^{\mathrm{lin}}.
$$

`evaluate` 用预积分雅可比做一阶修正，而不是调用 `repropagate`（那段重积分被 `#if 0` 关掉了）：

$$
\begin{aligned}
\hat{\boldsymbol{\alpha}}
&=
\boldsymbol{\alpha}
+
J^{\alpha}_{b_a}\delta\mathbf{b}_a
+
J^{\alpha}_{b_g}\delta\mathbf{b}_g,\\
\hat{\boldsymbol{\beta}}
&=
\boldsymbol{\beta}
+
J^{\beta}_{b_a}\delta\mathbf{b}_a
+
J^{\beta}_{b_g}\delta\mathbf{b}_g,\\
\hat{\boldsymbol{\gamma}}
&=
\boldsymbol{\gamma}
\otimes
\delta\mathbf{q}\!\left(J^{\gamma}_{b_g}\delta\mathbf{b}_g\right).
\end{aligned}
$$

$J^{\alpha}_{b_a}$ 就是 `jacobian.block<3,3>(O_P, O_BA)`，其余四块同理。旋转对加计 bias 的导数是 0，所以没有 $J^{\gamma}_{b_a}$。

### 6.3 残差本身

```180:184:VINS-Mono/vins_estimator/src/factor/integration_base.h
residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
```

写成向量，顺序与 `O_P=0, O_R=3, O_V=6, O_BA=9, O_BG=12` 一致：

$$
\mathbf{r}_{\mathcal{B}}
=
\begin{bmatrix}
{R_i}^{\!\top}(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}_i\Delta t+\tfrac12\mathbf{G}\Delta t^2)-\hat{\boldsymbol{\alpha}}\\
2\,\mathrm{vec}\big(\hat{\boldsymbol{\gamma}}^{-1}\otimes(\mathbf{q}_i^{-1}\otimes\mathbf{q}_j)\big)\\
{R_i}^{\!\top}(\mathbf{v}_j-\mathbf{v}_i+\mathbf{G}\Delta t)-\hat{\boldsymbol{\beta}}\\
\mathbf{b}_{a,j}-\mathbf{b}_{a,i}\\
\mathbf{b}_{g,j}-\mathbf{b}_{g,i}
\end{bmatrix}
\in\mathbb{R}^{15}.
$$

旋转残差取 $\hat{\boldsymbol{\gamma}}^{-1}\otimes\mathbf{q}_{ij}$ 的虚部再乘 2。小旋转时 $2\,\mathrm{vec}(\delta q)\approx\delta\boldsymbol{\theta}$，和右扰动的切空间一致。零偏残差是随机游走：相邻两帧的 bias 应该几乎一样，差多少由协方差里的 `ACC_W`、`GYR_W` 决定。

### 6.4 信息矩阵怎么乘上去

原始残差的单位不统一：位置是米，旋转是弧度，bias 是 $\mathrm{m/s^2}$ 和 $\mathrm{rad/s}$。`IMUFactor::Evaluate` 用协方差的逆做白化：

```64:66:VINS-Mono/vins_estimator/src/factor/imu_factor.h
Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
residual = sqrt_info * residual;
```

`covariance` 在 `midPointIntegration` 里按

$$
\mathbf{P}\leftarrow F\mathbf{P}F^{\top}+V\,Q\,V^{\top}
$$

递推。$Q$ 由配置里的 `acc_n, gyr_n, acc_w, gyr_w` 组成。`LLT` 把 $\mathbf{P}^{-1}=LL^{\top}$ 分解开，$L^{\top}\mathbf{r}$ 就是马氏距离里的那一项：

$$
\mathbf{r}^{\top}\mathbf{P}^{-1}\mathbf{r}
=
\|L^{\top}\mathbf{r}\|^2.
$$

雅可比同样左乘 `sqrt_info`。IMU 因子**没有** Cauchy 核，15 维残差被当成内点。视觉因子才有鲁棒核。两帧之间 `sum_dt > 10` 的预积分直接不加入问题，避免丢帧太久时协方差和雅可比都不可信。

`IMUFactor` 的四个参数块是 `SizedCostFunction<15, 7, 9, 7, 9>`：帧 $i$ 的位姿、帧 $i$ 的速度零偏、帧 $j$ 的位姿、帧 $j$ 的速度零偏。挂到问题里的写法：

```711:717:VINS-Mono/vins_estimator/src/estimator.cpp
for (int i = 0; i < WINDOW_SIZE; i++)
{
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0)
        continue;
    IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
}
```

预积分存在 `pre_integrations[j]`，约束的是帧 $j-1$ 和帧 $j$。第 0 帧没有“从上一帧积到我”的量。

---

## 7. 初始化

单目 SfM 给出的平移没有米制，重力方向未知，速度未知，陀螺 bias 未知。`initialStructure` 在窗口第一次攒满时把这些量一次性求出来。失败就滑掉一帧再试，两次尝试至少间隔 0.1 秒。

### 7.1 激励检查

对每一段预积分算 $\Delta\mathbf{v}/\Delta t$，看这些向量的标准差。小于 0.25 就打印 `IMU excitation not enough`。这段检查**不会** `return false`（返回被注释掉了）。它只是提醒：设备如果几乎不动，后面的尺度和重力会病态。真正把初始化打回去的是视差不够、SfM 失败，或对齐出的尺度为负、重力模长偏离 $g$ 超过 $1\,\mathrm{m/s^2}$。

### 7.2 选一对有视差的帧，做 SfM

`relativePose` 从最老帧往最新帧找。某一帧和最新帧的共视点超过 20，且平均视差换成像素后大于 30（`average_parallax * 460 > 30`），就用五点法 `solveRelativeRT` 求相对位姿。找到的那一帧下标记成 $l$。

`GlobalSFM::construct` 以帧 $l$ 和最新帧为基准，三角化，再对其余帧 PnP，最后做一次视觉 BA。产物是每帧相机位姿 $(\mathbf{Q}_i,\mathbf{T}_i)$ 和一批三维点，尺度任意。

窗口里不只有 11 个关键帧。`all_image_frame` 还存了视差不够、被标成次新帧丢掉的图像。对那些帧，用 SfM 点做 `solvePnP`（内参是单位阵，因为点已经在归一化平面上），补上相机位姿。

存进 `ImageFrame` 时把相机旋转乘回 IMU：

```297:298:VINS-Mono/vins_estimator/src/estimator.cpp
frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
frame_it->second.T = T[i];
```

因为 $R^w_c = R^w_b R^b_c$，所以 $R^w_b = R^w_c (R^b_c)^{\top}$。平移 `T` 在这一步仍是相机位置，尺度还没恢复。外参旋转若未知，要先走 §7.6，否则这里的 $R^b_c$ 是错的，后面整段对齐都会偏。

### 7.3 陀螺 bias

视觉旋转短时间比较可靠，预积分旋转则被 $\mathbf{b}_g$ 直接积分污染。`solveGyroscopeBias` 令二者一致。

视觉给出的相对旋转是 $\mathbf{q}_{ij}=R_i^{\top}R_j$。预积分给出 $\boldsymbol{\gamma}_{ij}(\mathbf{b}_g)$。在当前线性化点附近

$$
\boldsymbol{\gamma}_{ij}(\mathbf{b}_g+\delta\mathbf{b}_g)
\approx
\boldsymbol{\gamma}_{ij}
\otimes
\delta\mathbf{q}(J^{\gamma}_{b_g}\delta\mathbf{b}_g).
$$

令 $\boldsymbol{\gamma}\otimes\delta\mathbf{q}(J\delta\mathbf{b}_g)=\mathbf{q}_{ij}$，小角度下

$$
J^{\gamma}_{b_g}\,\delta\mathbf{b}_g
\approx
2\,\mathrm{vec}\big(\boldsymbol{\gamma}^{-1}\otimes\mathbf{q}_{ij}\big).
$$

每一对相邻帧贡献一个 $3\times 3$ 的方程，堆成最小二乘：

```19:26:VINS-Mono/vins_estimator/src/initial/initial_aligment.cpp
Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
A += tmp_A.transpose() * tmp_A;
b += tmp_A.transpose() * tmp_b;
```

解出的 $\delta\mathbf{b}_g$ 加到窗口内每一帧的 `Bgs` 上，然后 `repropagate(0, Bgs[0])`。这一步**会**重积分：初始化还没有一个可信的线性化点，一阶修正不够。加计 bias 在初始化里保持 0，留给后面的非线性优化。

### 7.4 速度、重力、尺度

bias 修正之后，预积分的 $\boldsymbol{\alpha},\boldsymbol{\beta}$ 可以和视觉平移对齐。未知量是每一帧在**自身 IMU 系**下的速度 $\mathbf{v}^{b_k}$、世界系重力 $\mathbf{g}^w$、以及尺度 $s$。相机位置和 IMU 位置差一个外参：

$$
\mathbf{p}^{b_k}
=
s\,\mathbf{T}_k
-
R_k\,\mathbf{p}^b_c.
$$

代进 §6.1 的位置、速度关系，并把 $\mathbf{v}^w_k=R_k\mathbf{v}^{b_k}$ 代进去，得到对相邻帧 $(i,j)$ 的两个 3 维方程。速度未知量在机体系，所以 $R_i^{\top}\mathbf{v}^w_i\Delta t$ 变成 $\mathbf{v}^{b_i}\Delta t$，平移差里的外参变成 $R_i^{\top}R_j\mathbf{p}^b_c-\mathbf{p}^b_c$。整理成 $A\mathbf{x}=\mathbf{b}$ 的形式就是 `LinearAlignment` 里的 `tmp_A, tmp_b`：

位置三行，未知量顺序是 $(\mathbf{v}^{b_i},\,\mathbf{v}^{b_j},\,\mathbf{g},\,s)$：

$$
\begin{aligned}
-\Delta t\,\mathbf{v}^{b_i}
+
R_i^{\top}\frac{\Delta t^2}{2}\mathbf{g}
+
s\,R_i^{\top}(\mathbf{T}_j-\mathbf{T}_i)
&=
\boldsymbol{\alpha}
+
R_i^{\top}R_j\mathbf{p}^b_c
-
\mathbf{p}^b_c.
\end{aligned}
$$

速度三行：

$$
-\mathbf{v}^{b_i}
+
R_i^{\top}R_j\,\mathbf{v}^{b_j}
+
R_i^{\top}\Delta t\,\mathbf{g}
=
\boldsymbol{\beta}.
$$

代码里尺度那一列乘了 $1/100$，法方程整体再乘 1000，都是数值缩放：SfM 平移和米制尺度量级差很多，不缩放时 $s$ 的列会比速度的列小几个数量级。解完之后要除回来：

```180:181:VINS-Mono/vins_estimator/src/initial/initial_aligment.cpp
double s = x(n_state - 1) / 100.0;
g = x.segment<3>(n_state - 4);
```

失败条件：尺度为负，或 $\|\mathbf{g}\|$ 与配置里的 `g_norm` 相差超过 $1$。

### 7.5 把重力模长固定住再细化

上一步的 $\mathbf{g}$ 是自由 3 维向量，模长只是事后检查。`RefineGravity` 把它收成

$$
\mathbf{g}
=
\|\mathbf{G}\|\,
\frac{\mathbf{g}_0+b_1 w_1+b_2 w_2}{\|\cdots\|},
$$

$b_1,b_2$ 是 $\mathbf{g}_0$ 的切平面基，由 `TangentBasis` 用一次叉乘得到。未知量从 3 维重力变成 2 维 $(w_1,w_2)$。迭代 4 次，每次在当前 $\mathbf{g}_0$ 处重新线性化。尺度列同样除了 100，所以 `visualInitialAlign` 里真正用的尺度是 `x.tail<1>() / 100` 已经在 `LinearAlignment` 末尾写回 `x` 之后的那个值。

### 7.6 外参旋转未知时

`ESTIMATE_EXTRINSIC == 2` 表示外参旋转也没有。在 SfM 之前，`InitialEXRotation::CalibrationExRotation` 先把相机相对旋转和 IMU 相对旋转对齐。

相邻帧：相机求出 $R^c_{k,k+1}$，IMU 预积分给出 $R^b_{k,k+1}$。外参 $R^b_c$ 应满足

$$
R^c
=
(R^b_c)^{\top}
R^b
R^b_c.
$$

写成四元数就是 $\mathbf{q}_c\otimes\mathbf{q}_{bc}=\mathbf{q}_{bc}\otimes\mathbf{q}_b$，用左右乘矩阵：

$$
\big(L(\mathbf{q}_c)-R(\mathbf{q}_b)\big)\mathbf{q}_{bc}=\mathbf{0}.
$$

每对帧堆 4 行。角度差大于 5° 的帧，权重用 Huber 式的 $5/\Delta\theta$ 缩小。对堆出来的 $A$ 做 SVD，最小奇异值对应的右奇异向量就是 $\mathbf{q}_{bc}$。代码再取逆，得到 `ric`。

成功条件：帧数至少达到 `WINDOW_SIZE`，且奇异值里第二小的那个大于 0.25。最小奇异值接近 0 只说明存在零空间；第二小的值够大，才说明这个零空间是一维的，旋转激励足够把外参定住。成功后 `ESTIMATE_EXTRINSIC` 改成 1，后面的非线性优化仍会refine 外参，只是不再从单位阵瞎猜。

平移外参初始化不在这里做。配置里给一个初值，滑窗优化再修。

### 7.7 把解写回滑窗

`visualInitialAlign` 做完这几件事后，`solver_flag` 才能变成 `NON_LINEAR`：

1. 所有帧的 `Ps, Rs` 换成对齐后的 IMU 位姿。
2. 深度全部清成 $-1$，强制按 IMU 位姿重新三角化。三角化时暂时把 `tic` 置 0，因为 SfM 平移还在相机中心上。
3. 用尺度把位置从相机中心变到 IMU，并令第 0 帧 IMU 落在原点：

$$
\mathbf{p}^{b_i}
\leftarrow
s\mathbf{p}^{c_i}
-
R_i\mathbf{p}^b_c
-
\big(s\mathbf{p}^{c_0}-R_0\mathbf{p}^b_c\big).
$$

4. 速度从机体系转回世界系：$\mathbf{v}^w=R\,\mathbf{v}^{b}$。
5. 所有特征的深度乘 $s$。
6. 用 `g2R(g)` 得到一个把重力转到 $+z$ 的旋转，再绕重力轴把第 0 帧的航向转成 0。位置、旋转、速度、重力一起左乘这个 `rot_diff`。

最后一步是整个系统世界系的定义。此后 `G` 在世界系里几乎就是 $(0,0,g)$，滚转和俯仰不再自由漂移。

---

## 8. 滑窗优化与视觉残差

`solveOdometry` 在窗口满且已经非线性之后，先 `triangulate`，再 `optimization`。

### 8.1 三角化给逆深度一个初值

只处理“至少被两帧看到，且起始帧早于 `WINDOW_SIZE-2`”的特征。已经有正深度的不重算。

对第一次观测建立相机系，投影矩阵取 $[I\mid 0]$。其他每一帧观测 $\mathbf{f}=(f_x,f_y,f_z)$ 和投影矩阵 $P$ 提供两条独立方程

$$
f_x P_{3}-f_z P_{1}=\mathbf{0},\qquad
f_y P_{3}-f_z P_{2}=\mathbf{0}.
$$

堆成 $A_{2N\times 4}\mathbf{X}=0$，取 $A$ 最小奇异向量。在第一帧相机系里 $P_0=[I\mid 0]$，所以深度就是

$$
\lambda^{-1}
=
\frac{X_z}{X_w}
=
\frac{\texttt{svd\_V[2]}}{\texttt{svd\_V[3]}}.
$$

小于 0.1 米的结果改成 `INIT_DEPTH = 5.0`。这个初值只负责让高斯–牛顿开得了步，真正的深度由后面的重投影残差决定。

### 8.2 视觉残差

`ProjectionFactor` 是 `SizedCostFunction<2, 7, 7, 7, 1>`：两个位姿、外参、一个逆深度，残差 2 维。

```35:47:VINS-Mono/vins_estimator/src/factor/projection_factor.cpp
Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
double dep_j = pts_camera_j.z();
residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
```

`pts_i` 是归一化坐标 $(x,y,1)$，除以逆深度等于乘深度，得到相机 $i$ 系下的三维点。投影到相机 $j$ 后再除掉 $z$，和观测 $(x_j,y_j)$ 相减：

$$
\mathbf{r}_C
=
\begin{bmatrix}
X_j/Z_j\\
Y_j/Z_j
\end{bmatrix}
-
\begin{bmatrix}
x_j\\
y_j
\end{bmatrix}.
$$

`UNIT_SPHERE_ERROR` 默认没开。打开之后残差改成单位球切平面上的差，大视角时比“除以 $z$”更不奇异。当前配置走的是上面这个针孔形式。

残差再左乘

$$
\sqrt{\mathbf{W}}
=
\frac{460}{1.5}\,I_2.
$$

含义：归一化平面上 $1.5/460\approx 0.0033$ 的误差，权重大约相当于 1 像素。所有视觉残差共用这个常数，没有按特征各自的协方差缩放。

鲁棒核是 `CauchyLoss(1.0)`，作用在**乘过** $\sqrt{W}$ 之后的残差上。IMU 和边缘化先验不加核。

一个特征若在帧 $i$ 首次出现，并在 $i$ 之后的帧 $j$ 再次出现，就加一条从 $(i,j)$ 出发的残差。$i=j$ 的那次观测是定义逆深度的参考帧，不构成残差。起始帧晚于 `WINDOW_SIZE-2` 的特征不进优化：它们马上可能被滑窗丢掉，雅可比不值得算。

### 8.3 视觉雅可比在对谁求导

记 $\mathbf{P}^{c_j}=(X,Y,Z)$，投影对相机系点的导数是

$$
\frac{\partial \boldsymbol{\pi}}{\partial \mathbf{P}^{c_j}}
=
\begin{bmatrix}
1/Z & 0 & -X/Z^2\\
0 & 1/Z & -Y/Z^2
\end{bmatrix}.
$$

代码里叫 `reduce`。它再左乘 $\sqrt{W}$。后面所有雅可比都是 `reduce` 乘上“$\mathbf{P}^{c_j}$ 对某个参数块的导数”。

对帧 $i$ 的位姿（平移 3 维 + 右扰动 3 维）：

$$
\frac{\partial \mathbf{P}^{c_j}}{\partial \delta\mathbf{p}_i}
=
{R^b_c}^{\!\top}{R_j}^{\!\top},
\qquad
\frac{\partial \mathbf{P}^{c_j}}{\partial \delta\boldsymbol{\theta}_i}
=
{R^b_c}^{\!\top}{R_j}^{\!\top} R_i\,[-\mathbf{P}^{b_i}]_\times.
$$

对帧 $j$：

$$
\frac{\partial \mathbf{P}^{c_j}}{\partial \delta\mathbf{p}_j}
=
-{R^b_c}^{\!\top}{R_j}^{\!\top},
\qquad
\frac{\partial \mathbf{P}^{c_j}}{\partial \delta\boldsymbol{\theta}_j}
=
{R^b_c}^{\!\top}[\mathbf{P}^{b_j}]_\times.
$$

对逆深度。$\mathbf{P}^{c_i}=\mathbf{f}_i/\lambda$，所以 $\partial\mathbf{P}^{c_i}/\partial\lambda=-\mathbf{f}_i/\lambda^2$。这个向量再经过 $R_i$、$R_j$、$R^b_c$ 送到相机 $j$：

```112:112:VINS-Mono/vins_estimator/src/factor/projection_factor.cpp
jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
```

$7$ 维参数块的雅可比是 $2\times 7$，最后一列（四元数 $w$ 的那一列）置 0，因为局部参数化已经把增量限制在 6 维。Ceres 会再用 `PoseLocalParameterization` 的 $7\times 6$ 雅可比把它们乘到切空间上。

外参那一块更长：平移和旋转同时出现在“从 $i$ 变到世界”和“从世界变回 $j$”两条路上，所以 $\partial\mathbf{P}^{c_j}/\partial\mathbf{p}^b_c$ 里有 $R_j^{\top}R_i-I$。读 `jaco_ex` 时按这个结构对，不必把每一项都重新推导一遍。

### 8.4 时间偏移

`ESTIMATE_TD` 打开时，用的是 `ProjectionTdFactor`，多一个 1 维参数 $t_d$。特征在归一化平面上的速度已经由前端算好。把观测时刻从图像时间补偿到 IMU 时间：

$$
\mathbf{f}(t_d)
=
\mathbf{f}
-
\big(t_d-t_d^{\mathrm{feat}}+\tfrac{t_r}{H}(v_{\mathrm{row}}-H/2)\big)
\mathbf{v}_{\mathrm{pixel}}.
$$

`td_i` 是这条特征被记录时的 $t_d$ 快照，$(t_d-t_d^{\mathrm{feat}})$ 是优化过程中时间偏移又移动了多少。`TR/ROW * row` 是卷帘快门：图像不同行的曝光时刻差一个行时间。`row` 在构造函数里已经减去了 `ROW/2`，以图像中线为零点。

补偿后的 $\mathbf{f}_i(t_d)$、$\mathbf{f}_j(t_d)$ 再走和 §8.2 相同的投影。残差对 $t_d$ 的导数就是这条链对补偿量的导数，主项正比于光流速度。速度估计很差时，在线 $t_d$ 会跟着漂，配置里可以把它固定住。

### 8.5 求解器

```803:818:VINS-Mono/vins_estimator/src/estimator.cpp
options.linear_solver_type = ceres::DENSE_SCHUR;
options.trust_region_strategy_type = ceres::DOGLEG;
options.max_num_iterations = NUM_ITERATIONS;
if (marginalization_flag == MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
else
    options.max_solver_time_in_seconds = SOLVER_TIME;
```

`DENSE_SCHUR` 是因为逆深度数量远多于位姿。把路标消元之后，相机块只有 $11\times(6+9)$ 量级，再加外参和 $t_d$。Dogleg 是信赖域步长策略，比纯高斯–牛顿更能接受一个较差的初始深度。

时间预算故意留一块给边缘化：关键帧那次优化只用 `SOLVER_TIME` 的五分之四，剩下的时间算 Schur 补。次新帧那次边缘化很轻，优化可以用满预算。

解完 `double2vector()` 把增量写回 `Ps, Rs, Vs, Bas, Bgs`。四元数在这里要处理符号：优化可能把 $w$ 翻到负半圈，写回前会保证和优化前的四元数点积为正，避免姿态在 $\mathbf{q}$ 和 $-\mathbf{q}$ 之间跳。

若这一帧带了回环约束，`relocalization_info` 为真。回环帧的位姿 `relo_Pose` 也放进同一个问题，用普通 `ProjectionFactor` 把滑窗里的特征和回环帧的匹配点连起来。它改的是滑窗，不是全局位姿图。全局一致性靠下一节的 4 自由度图，滑窗只吸收一个局部的重定位拉力。

---

## 9. 边缘化

窗口只有 11 帧。新帧进来就必须有帧出去，否则法方程会随时间长大。直接删状态会丢掉那些帧曾经提供的信息，所以先把与被删变量有关的因子线性化，做成一个先验因子，加进下一次优化。

### 9.1 两种滑窗

**丢掉最老帧**（`MARGIN_OLD`），这帧是关键帧。先验里放进三类因子：

- 上一次的边缘化先验，其中涉及 `para_Pose[0]` 和 `para_SpeedBias[0]` 的部分继续被消掉；
- 帧 0 和帧 1 之间的 `IMUFactor`，参数块 0 和 1（位姿与速度零偏）标记为要丢掉；
- 所有起始帧是 0 的视觉因子。丢掉的是帧 0 的位姿和该特征的逆深度（`drop_set = {0, 3}`）。帧 $j$ 的位姿和外参留下。

逆深度定义在第一次观测的相机系。起始帧被删掉之后，这个 $\lambda$ 没有宿主，必须一起边缘化，不能留在窗口里。

**丢掉次新帧**（`MARGIN_SECOND_NEW`）。这帧视差太小，和前一帧几乎是重复观测。代码只把上一次先验里碰到 `para_Pose[WINDOW_SIZE-1]` 的部分消掉，**不**把这帧的 IMU 和视觉因子加进新先验。那些测量直接随着 `slideWindowNew` 扔掉。保留它们会在几乎相同的两个位姿之间塞进一组强相关的视觉约束，数值上很差，信息量又很小。

滑窗之后地址要平移。最老帧被删时，原来的 `para_Pose[i]` 会变成 `para_Pose[i-1]`。`getParameterBlocks` 用 `addr_shift` 把先验里保存的指针改到新地址。这一步如果漏掉，下一次 `AddResidualBlock` 会读到已经滑走的内存。

### 9.2 Schur 补

把所有要边缘化的因子在当前线性化点展开：

$$
\mathbf{r}(\mathbf{x}_m,\mathbf{x}_r)
\approx
\mathbf{r}_0+J_m\delta\mathbf{x}_m+J_r\delta\mathbf{x}_r.
$$

法方程分成“丢掉的 $m$ 维”和“留下的 $n$ 维”：

$$
\begin{bmatrix}
A_{mm} & A_{mr}\\
A_{rm} & A_{rr}
\end{bmatrix}
\begin{bmatrix}
\delta\mathbf{x}_m\\
\delta\mathbf{x}_r
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{b}_m\\
\mathbf{b}_r
\end{bmatrix}.
$$

消掉 $\delta\mathbf{x}_m$：

$$
\begin{aligned}
A
&=
A_{rr}-A_{rm}A_{mm}^{-1}A_{mr},\\
\mathbf{b}
&=
\mathbf{b}_r-A_{rm}A_{mm}^{-1}\mathbf{b}_m.
\end{aligned}
$$

`Amm` 先对称化，再用特征分解求逆，小于 `eps` 的特征值当成 0。这是在处理视觉约束不足时 $A_{mm}$ 的零空间，避免把无穷大的信息写进先验。

Ceres 需要的是残差和雅可比，不是 $A$ 和 $\mathbf{b}$。再对留下的 $A$ 做一次特征分解 $A=VSV^{\top}$：

```287:290:VINS-Mono/vins_estimator/src/factor/marginalization_factor.cpp
Eigen::VectorXd S_sqrt = S.cwiseSqrt();
Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
```

这样 $J^{\top}J=A$，$J^{\top}\mathbf{r}_0=\mathbf{b}$。下一次优化里，先验因子不再保存原来的 IMU 或重投影，只保存

$$
\mathbf{r}_{\mathrm{prior}}
=
\mathbf{r}_0+J\,\delta\mathbf{x}.
$$

$\delta\mathbf{x}$ 相对**边缘化发生时**的参数值。位姿不是做减法：平移直接相减，旋转用

$$
\delta\boldsymbol{\theta}
=
2\,\mathrm{vec}\big(\mathbf{q}_0^{-1}\otimes\mathbf{q}\big),
$$

并且强制 $\mathbf{q}_0^{-1}\otimes\mathbf{q}$ 的实部非负（`positify`），否则虚部会反号，先验把姿态往相反方向拉。

这个先验是线性化一次之后就冻结的。后续迭代如果离线性化点很远，先验会偏。VINS 接受这个近似，换来固定尺寸的滑窗。首次估计雅可比（FEJ）一类的做法是为了减轻这种不一致，本仓库的实现没有单独做 FEJ，线性化点就是边缘化那一刻 `vector2double` 写下的值。

---

## 10. 回环与 4 自由度位姿图

滑窗里的边缘化把老信息压成了先验，老帧之间的原始约束已经不在了。长距离漂移要靠回环单独修。

### 10.1 检测

关键帧上另提 BRIEF 描述子，放进 DBoW2。`detectLoop` 先 `query` 再 `add`，避免自己和自己比。查询排除最近 50 帧（`frame_index - 50`）。

接受回环的条件同时满足：

- 最近邻得分 `ret[0].Score > 0.05`（这一帧和数据库里“很像的那些”整体足够像）；
- 除最近邻之外还有一帧得分 `> 0.015`；
- 当前帧号大于 50。

返回的是满足阈值的候选里**最老**的那一帧，而不是得分最高的那一帧。最老的匹配通常离当前帧更远，拉回来的漂移修正更有意义。

随后在回环帧和当前帧之间做描述子匹配，再用 fundamental matrix 和 PnP 求一个相对位姿。这个相对位姿不直接改里程计，而是作为位姿图里的一条回环边。

### 10.2 只优化航向和位置

重力把滚转和俯仰定住了，这两维若再放进位姿图，回环的错误匹配会把一个本可观测的绝对方向拧弯。`optimize4DoF` 的参数因此是每帧一个航向角和三维平移。俯仰、滚转来自 VIO，优化过程中当作常数。

顺序边：每一帧和它前面最多 4 个**同一 sequence** 的关键帧之间，加一条 `FourDOFError`。相对平移和相对航向在加边时由当时的 VIO 位姿算好，之后作为测量，不再改：

$$
\begin{aligned}
\mathbf{r}_t
&=
R(y_i,\,p_i,\,r_i)^{\top}(\mathbf{t}_j-\mathbf{t}_i)
-
\mathbf{t}_{ij}^{\mathrm{vio}},\\
r_y
&=
\mathrm{Normalize}(y_j-y_i-y_{ij}^{\mathrm{vio}}).
\end{aligned}
$$

$p_i,r_i$ 是常数。残差 4 维，参数是 $(y_i,\mathbf{t}_i,y_j,\mathbf{t}_j)$。

回环边用 `FourDOFWeightError`，形式相同，套 `HuberLoss(0.1)`。航向残差额外除以 10，让角度（度）和位置（米）的数值不要差一个数量级。顺序边没有鲁棒核：它们来自短时间的 VIO，本来就该被强约束保住局部形状。

最早一个回环帧，以及 sequence 0 的帧，航向和平移都设成常数，用来定住这张图的规范。求解器是 `SPARSE_NORMAL_CHOLESKY`，最多 5 次迭代。位姿图不追求把点重三角化，只把关键帧骨架拉一致。

### 10.3 漂移怎么作用到输出上

优化后，当前帧的位姿图位姿和它的 VIO 位姿之间差一个航向和一个平移：

```554:556:VINS-Mono/pose_graph/src/pose_graph.cpp
yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
t_drift = cur_t - r_drift * vio_t;
```

之后发布的 VIO 位姿左乘 $(r_{\mathrm{drift}}, t_{\mathrm{drift}})$。滑窗内部的 `Ps, Rs` **不**被位姿图覆盖。这样里程计线程保持平滑，回环只改变“这段轨迹在世界系里怎么摆”。快速运动时如果把滑窗状态硬改掉，IMU 预积分和视觉残差会在下一帧突然不一致。

---

## 11. 失败检测

`failureDetection` 在每次非线性求解之后调用。触发则 `clearState`、`setParameter`，系统回到 `INITIAL`，前端通过 restart 话题把跟踪也清掉。

| 条件 | 阈值 | 是否真的重启 |
| --- | --- | --- |
| 最近一帧跟踪到的特征数 | $<2$ | 只打印，`return true` 被注释 |
| 加计 bias 模长 | $>2.5\,\mathrm{m/s^2}$ | 重启 |
| 陀螺 bias 模长 | $>1.0\,\mathrm{rad/s}$ | 重启 |
| 最新帧与上一帧的位移 | $>5\,\mathrm{m}$ | 重启 |
| 同一位移的 $z$ 分量 | $>1\,\mathrm{m}$ | 重启 |
| 两帧之间的转角 | $>50^\circ$ | 只打印，不重启 |

bias 阈值是在抓发散，不是在抓缓慢漂移。$z$ 方向单独收紧，是因为错误尺度或错误重力会首先表现为高度飞掉。转角阈值被关掉，是因为快速转头在手持和无人机上都合法，用它重启会把正常运动杀掉。

---

## 12. 建议的阅读顺序

按调用链，而不是按目录字母序：

1. `feature_tracker_node.cpp` 的 `img_callback`，看 7 维观测怎么组出来。
2. `estimator_node.cpp` 的 `process`，看 IMU 如何对齐到 $t_{\mathrm{img}}+t_d$。
3. `Estimator::processIMU` 与 `processImage`，分清世界系传播和预积分。
4. `integration_base.h` 的 `midPointIntegration` 与 `evaluate`。协方差递推对照 [预积分讲义](imu_preintegration.md)。
5. `initial_ex_rotation.cpp`，再 `initialStructure` → `initial_aligment.cpp` 的三步：陀螺 bias、线性对齐、重力细化。
6. `FeatureManager::triangulate`，然后 `ProjectionFactor::Evaluate`。先不看雅可比，把残差的五个坐标系变过去。
7. `Estimator::optimization` 前半：参数块、IMU 因子、视觉因子、求解器选项。
8. 同函数后半和 `marginalization_factor.cpp` 的 `marginalize`：谁被放进 `drop_set`，Schur 补如何变成 $J$ 和 $\mathbf{r}_0$。
9. `pose_graph.cpp` 的 `detectLoop` 与 `optimize4DoF`，最后看 `yaw_drift` 如何加到发布位姿上。

对照代码时有三个容易看反的约定：

- 四元数在数组里是 $(x,y,z,w)$，构造 `Quaterniond` 时 $w$ 在最前。
- `pre_integrations[j]` 连接的是帧 $j-1$ 和帧 $j$，不是帧 $j$ 和帧 $j+1$。
- `addFeatureCheckParallax` 返回真表示“当关键帧、边缘化最老帧”。返回假才是丢掉次新帧。

---

## 参考文献

1. Qin, Li, Shen. *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator*. IEEE TRO, 2018.
2. Forster, Carlone, Dellaert, Scaramuzza. *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*. IEEE TRO, 2017.
3. 本仓库实现：`VINS-Mono/vins_estimator`、`feature_tracker`、`pose_graph`。
