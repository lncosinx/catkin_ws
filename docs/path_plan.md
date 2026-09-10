# 路径规划子系统技术文档（path_planner_node）

`path_planner_node` 是流水线的**大脑**：把策略节点给的「抓取像素 / 放置格子」全部解算成
**base 系可直接执行的位姿**，决定腕部 180° 翻转，并对整条抓放序列做**关节空间运动时间**
优化，输出 `lucky::MotionPlan` 给纯执行器控制节点。本文说明数据经过哪些解算步骤、每步的
方法与**数学公式**。

参考代码：
- 节点：[path_planner_node.cpp](../src/lucky/src/path_planner_node.cpp)
- 运动学：[include/lucky/xarm6_kinematics.hpp](../src/lucky/include/lucky/xarm6_kinematics.hpp)
- 测试：[test_path.launch](../src/lucky/launch/test_path.launch)

```
vision(/vision/board_state) ┐
strategy(/tetris_plan[_candidates]) ┼─► path_planner ─► /motion_cmds (MotionPlan) ─► controller
joint_states, camera_info, depth, TF ┘
```

---

## 0. 数据流总览

```
每个任务 (shape, way, place_row/col, pick_u/v/angle, geom_u/v, 4 目标格)
   │
   ▼ [A] 坐标解算（board 系）
   │     抓取 XY: 单应性 + 高度视差 | 抓取 Z: 深度/平面 | yaw: 单应性投影
   │     放置 XY: 双线性格心 + 几何中心对齐 | 放置 Z: 逐格取最高 | TCP 补偿
   ▼ [B] board → base 位姿  T_base = board_to_base · T_table
   ▼ [C] 运动时间代价  沿 move_line 直线路径逐点 IK 积分 Δt
   ▼ [D] 联合优化  放置顺序 × 抓取分配 × 腕部翻转，最小化总时间
   ▼ 发布 MotionPlan（每任务 4 位姿：pick/pick_hover/place/place_hover）
```

坐标系：`BOARD_POSE_BASE` 以常量 `tf2::Transform`（`board_to_base_`）加载，唯一需要的动态
TF 是 `camera → base`（相机随臂动，eye-on-hand）。

---

## A. 坐标解算（board 局部系）

### A.1 像素反投影为射线
用相机投影矩阵 $P$ 的内参 $(f_x,f_y,c_x,c_y)$，把像素 $(u,v)$ 反投影为归一化相机射线：

$$\mathbf r_\text{cam}=\Big(\tfrac{u-c_x}{f_x},\ \tfrac{v-c_y}{f_y},\ 1\Big)^\top$$

### A.2 抓取 XY — 平面单应性（默认）
抓取侧标定了一张**单应性** $H\in\mathbb R^{3\times3}$（`PICK_HOMOGRAPHY`，pixel → board-XY，
标定在 board 系 $z=0$ 发光板平面）。射影变换：

$$\begin{bmatrix}X\,W\\ Y\,W\\ W\end{bmatrix}=H\begin{bmatrix}u\\ v\\ 1\end{bmatrix},\qquad
X=\frac{h_0 u+h_1 v+h_2}{h_6 u+h_7 v+h_8},\quad Y=\frac{h_3 u+h_4 v+h_5}{h_6 u+h_7 v+h_8}$$

**为什么用单应性而非 3D 反投影**：手眼 3D 投影在板面实测偏 1~2cm（URDF 名义运动学≠固件
出厂标定），而平面单应性直接从像素标定到 board-XY，是精确路径。

**2.5D 高度视差补偿**：单应性标定在 $z=0$ 平面，而方块有厚度 $z_\text{blk}$。相机在板上方
高度 $z_\text{cam}$ 时，抬高的块表面点在图像里相对光心 $(c_x,c_y)$ 被放大。把观测像素按
相似三角形投回 $z=0$ 平面应有的位置再套单应性：

$$\text{ratio}=\frac{z_\text{cam}-z_\text{blk}}{z_\text{cam}},\qquad
u_\text{flat}=c_x+(u-c_x)\,\text{ratio},\quad v_\text{flat}=c_y+(v-c_y)\,\text{ratio}$$

（仅当 $z_\text{cam}>z_\text{blk}+0.05$ 时启用。）

### A.3 抓取 Z — 深度优先、平面回退
- **深度**（`use_depth_pick_z`）：对齐深度邻域中值 $Z$，反投影 $p_\text{cam}=Z\,\mathbf r_\text{cam}$，
  再 $p_\text{table}=R\,p_\text{cam}+T$，取其 $z$。
- **真实抓取平面回退**（`use_true_pick_plane`）：标定的触面平面（点 $\mathbf p_0$、法向
  $\mathbf n$，base 系）。**射线-平面求交**：射线 $\mathbf p(t)=\mathbf c+t\,\mathbf r_\text{base}$，
  代入平面方程 $\mathbf n\cdot(\mathbf p-\mathbf p_0)=0$ 解

  $$t=\frac{\mathbf n\cdot(\mathbf p_0-\mathbf c)}{\mathbf n\cdot\mathbf r_\text{base}},\qquad \text{hit}=\mathbf c+t\,\mathbf r_\text{base}$$

- **水平平面回退**：常值平面 $z=\texttt{PICK\_Z}$，$t=(\texttt{PICK\_Z}-c_z)/r_z$。

### A.4 抓取 yaw — 单应性投影测角
图像方向经透视 ≠ 物理方向，不能直接用图像角。做法：从抓取像素沿图像角探一小段
长 $L$，把两端点都经单应性映到 board 平面，用板面位移测角：

$$(u_1,v_1)=(u+L\cos\theta_\text{img},\ v+s\,L\sin\theta_\text{img}),\quad
\theta_\text{pick}=\operatorname{atan2}(Y_1-Y_0,\ X_1-X_0)$$

其中 $s=\texttt{yaw\_homography\_v\_sign}$，再叠加 `pick_yaw_offset`。

### A.5 放置 XY — 双线性格心 + 几何中心对齐
放置网格标定了 $14\times10$ 格心 `BOARD_CENTERS`。对（可为分数的）目标行列 $(r,c)$，设
$r_0=\lfloor r\rfloor,\ c_0=\lfloor c\rfloor,\ t_r=r-r_0,\ t_c=c-c_0$，**双线性插值**：

$$\mathbf p=(1{-}t_r)(1{-}t_c)\,\mathbf p_{00}+t_r(1{-}t_c)\,\mathbf p_{10}+(1{-}t_r)t_c\,\mathbf p_{01}+t_r t_c\,\mathbf p_{11}$$

**几何中心对齐**：hybrid 抓取点偏离方块几何中心，直接放会平移。设 board 系抓取点减几何
中心偏移 $\boldsymbol\Delta=\mathbf p_\text{pick}-\mathbf p_\text{geom}$，因放置比抓取多转了
$\delta=\theta_\text{place}-\theta_\text{pick}$，把偏移旋转后补到放置 XY：

$$\begin{bmatrix}\Delta x'\\ \Delta y'\end{bmatrix}=
\begin{bmatrix}\cos\delta&-\sin\delta\\ \sin\delta&\cos\delta\end{bmatrix}
\begin{bmatrix}\Delta x\\ \Delta y\end{bmatrix}$$

使方块**几何中心**（而非吸点）对齐网格。$\delta$ 不受 180° 翻转影响。

### A.6 放置 Z — 逐格取最高
放置面是白板凸起顶面（比抓取的发光板平面高一个白板厚）。逐格高度图 `PLACE_Z_MAP`；任务
带 4 目标格时取这些格的**最高** place-Z（保证不压到凸起），再加余量与 TCP 补偿：

$$z_\text{place}=\max_{(r,c)\in\text{cells}}\text{PLACE\_Z\_MAP}[r][c]+\texttt{place\_release\_z\_margin}+\texttt{tcp\_place\_offset\_z}$$

### A.7 放置 yaw 与 TCP 侧向补偿
放置偏航 $\theta_\text{place}=-\,\texttt{way}\cdot 90°$。TCP 标定残差补偿：偏移
$(o_x,o_y)$ 在工具系，按当前 yaw 旋转后加到 XY：

$$x\mathrel{+}=o_x\cos\theta-o_y\sin\theta,\qquad y\mathrel{+}=o_x\sin\theta+o_y\cos\theta$$

### A.8 下压朝向与悬停
- roll/pitch：默认取收到 plan 时**臂此刻位姿**（即单应性标定位姿）的工具朝向换算到 board
  系固定（来源固件 `xarm_states.pose`，消 URDF↔固件 ~1° 差）；关掉则用 `fixed_roll/pitch`
  （默认 $\pi,0$）。yaw 每任务算。
- 悬停 `hoverFromTable`：把位姿抬到 `HOVER_Z`（board +Z 即板面法向），至少高出目标 10mm。

---

## B. board → base 位姿

board 系位姿 $T_\text{table}$（含旋转四元数与平移）经常值刚体变换合成到 base：

$$T_\text{base}=T_{\text{board}\to\text{base}}\cdot T_\text{table}$$

其中 $T_{\text{board}\to\text{base}}$ 由 `BOARD_POSE_BASE`（原点 + RPY）构造。四位姿
（pick/pick_hover/place/place_hover）各自换算。

---

## C. 运动时间代价（关节空间）

关节代价 = **沿 `move_line` 直线路径积分的真实运动时间**（取代旧的 XY 直线距离），因为
xArm 各轴**同时到达**，多转的关节会成为整段速度瓶颈；且直线平移里 J1 非线性摆动、
J6≈heading−J1 中途可越两端点撞限位——端点式代价看不见，必须沿路径采样。

### C.1 xArm6 正运动学（FK）
6 关节精确链乘：每关节 $T_i(q_i)=\text{preTf}(i)\cdot R_z(q_i)$（`preTf` 为 URDF 连杆固定
变换），末端

$$T_{\text{base}\gets\text{tcp}}(\mathbf q)=\Big(\textstyle\prod_{i=0}^{5}\text{preTf}(i)\,R_z(q_i)\Big)\cdot T_\text{tool}$$

### C.2 逆运动学（阻尼最小二乘 DLS / Levenberg–Marquardt）
从 seed（上一路点或测量关节）迭代，把解锁在 seed 分支上——**复刻固件笛卡尔指令的「就近
解」行为**。每步：

**位姿误差** 6 维（平移 + 旋转向量，base 系）：$\mathbf e=[\mathbf e_p;\ \mathbf e_r]$，
$\mathbf e_p=\mathbf p_t-\mathbf p_\text{cur}$，旋转误差取 $R_e=R_t R_\text{cur}^\top$ 的轴角向量

$$\alpha=\arccos\tfrac{\operatorname{tr}(R_e)-1}{2},\qquad
\mathbf e_r=\frac{\alpha}{2\sin\alpha}\,(R_{e,21}{-}R_{e,12},\ R_{e,02}{-}R_{e,20},\ R_{e,10}{-}R_{e,01})^\top$$

**几何雅可比**（全旋转关节）第 $i$ 列（$\mathbf z_i$ 关节轴、$\mathbf p_i$ 关节位置、
$\mathbf p_\text{tcp}$ 末端）：

$$J_i=\begin{bmatrix}\mathbf z_i\times(\mathbf p_\text{tcp}-\mathbf p_i)\\ \mathbf z_i\end{bmatrix}\in\mathbb R^{6}$$

**阻尼正规方程**（$\lambda=0.05$ 阻尼，避开奇异）解关节增量，高斯消元求解 $6\times6$：

$$\big(J^\top J+\lambda^2 I\big)\,\Delta\mathbf q=J^\top\mathbf e$$

**限步**：$\text{scale}=\min(1,\ \texttt{max\_step}/\max_i|\Delta q_i|)$，$\mathbf q\mathrel{+}=\text{scale}\cdot\Delta\mathbf q$。
收敛判据 $\lVert\mathbf e_p\rVert<10^{-5}\,\text{m}\ \wedge\ \lVert\mathbf e_r\rVert<10^{-5}\,\text{rad}$，且需在硬限位内。

### C.3 转移段时间积分（`evalTransit`）
沿 A→B 直线笛卡尔路径均匀采样 $N$ 点：位置线性插值、姿态 **slerp**

$$\mathbf q(t)=\frac{\sin((1{-}t)\Omega)}{\sin\Omega}\,\mathbf q_A+\frac{\sin(t\Omega)}{\sin\Omega}\,\mathbf q_B,\quad \Omega=\arccos|\mathbf q_A\!\cdot\!\mathbf q_B|$$

**基准笛卡尔段时间**（位置线速度与姿态角速度取瓶颈）：设总位移 $\Delta d=\lVert B-A\rVert$、
总姿态角 $\Delta\theta=2\arccos|\mathbf q_A\!\cdot\!\mathbf q_B|$，

$$\Delta t_\text{nom}=\max\!\Big(\frac{\Delta d}{N\,v},\ \frac{\Delta\theta}{N\,\omega}\Big),\qquad \omega=\texttt{omega\_per\_v\_lin}\cdot v$$

$\omega$ 随线速度自适应（实测比值 $\approx\pi$，与档位无关，`measure_tcp_omega` 标定）。后果：
一个 $180°$ 翻转 $\approx 1\text{m}$ 平移的时间代价。

**逐段对关节饱和取 max**（每点从连续 seed 做 IK，得 $\Delta\mathbf q$）：

$$\Delta t_k'=\max\!\Big(\Delta t_\text{nom},\ \max_i\frac{|\Delta q_{k,i}|}{v_{\max,i}}\Big),\qquad
\text{time\_cost}=\sum_k\Delta t_k'$$

同时累计 $\sum|\Delta q|$（平局次级项）与路径 $\max|J_6|$（限位校验）。任一采样 IK 失败 →
该转移不可行。

---

## D. 联合优化

统一最小化上面的**时间代价**，联合三个自由度：**放置顺序**（DAG 内重排）× **同形状抓取
分配**（从 board_state 候选池选物理块）× **腕部翻转** A(不翻)/B(翻 $180°$)。

### D.1 腕部翻转
吸盘对 $180°$ 对称，翻转 $\theta\to\theta+\pi$ 不改抓放落点，只改腕部转角——这是唯一能用
笛卡尔指令表达的自由度（圈数无法命令，固件按就近解）。对每个（放置槽, 抓取候选）两套翻转
各算代价取小（`evalPairJoint`）。

### D.2 限位与代价
评估两段大转移 cur→pick_hover（空载 $v_\text{transit}$）、pick_hover→place_hover（载料
$v_\text{loaded}$）。设路径 $\max|J_6|=M$：

$$\text{cost}=\text{time}_A+\text{time}_B+\underbrace{w_\text{tie}(\text{travel}_A+\text{travel}_B)}_{\text{平局项}}
+\underbrace{w_\text{soft}\,\max(0,\ M-\theta_\text{soft})}_{\text{软限位罚}}$$

**硬限位** $M>\texttt{wrist\_hard\_limit}$（到 $\pm2\pi$ 留余量）→ 绝对拒绝该方案；软限位越
界仍可用但秒尺度加罚（least-bad 有序）；平局用 $\sum|\Delta q|$ 偏好腕部少甩。

### D.3 顺序优化
两种模式**共用有界 beam 搜索**（宽度 `reorder_beam_width`，默认 12；$B{=}1$ 退化为原贪心）。
因 J6 可行性**路径相关**（某步能否可行取决于此前所有步的就近解落点），逐步贪心会近视地把
J6 绕到边界令后续无解，beam 保留前 $B$ 条最低累计代价前缀，让下游不可行/高代价能回改上游。
- **允许重排**（`allow_reorder`）：构造放置**依赖 DAG**——格 $(r,c)$ 之下 $(r{+}1,c)$ 是别的
  块则「下先放」，加边。每个 beam 节点**各自携带一份 DAG 进度**（入度/就绪前沿/已放置），
  每步只在自己的就绪前沿里展开（槽,块,翻转），放后更新入度解锁后继——即带代价、可回溯的
  拓扑排序。
- **保序**（`allow_reorder=false`，比赛可能有相邻等 DAG 外约束）：固定策略给的顺序，beam 仅在
  每槽的 (物理块, 翻转) 里展开。
- **前缀保险**：关节模式下若无「全 $n$ 槽」可行解，执行第一个不可行槽之前的最长合法前缀
  （保序前缀不破坏 DAG，可安全执行），并告警丢弃其余。
- **XY 回退**（IK 不可用）：代价退化为 $\sum(\lVert\text{cur}\!\to\!\text{pick}\rVert+\lVert\text{pick}\!\to\!\text{place}\rVert)$
  的贪心，事后用 `assignFlipsByYaw` 的 A/B 腕角 travel 启发式定翻转。

### D.4 多候选择优
`use_plan_candidates` 时，策略节点把若干同分布局发到 `/tetris_plan_candidates`，本节点逐个
求总代价（`parallel_candidate_eval` 多线程），选**总代价最小**者执行。

### D.5 起点关节与自检
起点 $\mathbf q_\text{cur}$ 取自 `/xarm/joint_states`（缺则标称种子）。启动用 FK/IK 对照
`base→link_tcp` TF **自检**（`ik_selfcheck_tol_rad`），失败回退 XY 直线代价安全网。

### D.6 为什么达不到「真·全局最优」（重要结论，勿再走回头路）

本问题 = **带优先约束（DAG）的最小代价序列 + 选块 + 翻转**，是 NP-hard 的带优先 TSP 变体。
更棘手的是**代价路径相关**：J6 是「就近解」，每步落点关节 $\mathbf q$ 由上一位形 seed 出来
（见 CLAUDE.md 腕部圈数 lore、`xarm6_kinematics.hpp` seeded-DLS IK），故某步的代价**乃至可行性**
取决于此前全部选择的历史。已实证三条路线都**绕不开这个根本障碍**：

| 方案 | 组合层 | 为何仍非全局最优 |
| --- | --- | --- |
| **有界 beam**（现用） | 近似 | 每步剪枝到前 $B$，可能丢掉「当前贵但下游更省」的前缀 |
| **状压 DP**（mask 键控 Held–Karp） | 伪精确 | 占优剪枝**失效**：`mask` 未含端点位形 $\mathbf q$、端点位置、块占用——同一 mask 不同历史被错误合并；可达 mask 数在松 DAG 下近指数，`N≤64` 是虚假安慰 |
| **状态离散图 + CP-SAT** | 图内精确 | 把决策塞进节点身份（选块/翻转/端点）修好了 DP 的坍缩，CP-SAT 能求**图模型**的精确最优——但节点 $\mathbf q_\text{end}$ 被量化成 IK(常量 `nominal_seed`) 的**历史无关定值**，与真实路径相关 $\mathbf q$ 不符，J6 绕圈敏感处即失真；且笛卡尔 `move_line` 按就近解挑圈，规范位形**无法命令**，量化在物理上补不齐 |

**根因**：状态含**连续的、路径相关的关节量** $\mathbf q$，无法无损离散折叠——任何离散化（DP 的
mask、图的固定 $\mathbf q_\text{end}$）都只得到**近似模型的最优**，不是真实机械臂的最优。要严格
最优只剩指数级 B&B，实时（50ms 级）做不到。**结论：实时真·全局最优在本问题上不可得，别再
尝试「上 DP / CP-SAT 求全局最优」——它们的最优性是模型内的，落到真机就是近似。**

### D.7 如何**接近**全局最优（可落地的改进方向，按性价比）

既然不能严格最优，目标改为「用可控算力把近似做到足够好、且不牺牲实时」：

1. **beam 后接 2-opt / or-opt 局部改善**（✅ 已实现，`refineOrderTwoOpt`，默认开）：在**不破坏
   DAG**的邻域内做 2-opt（反转一段子序列）、or-opt（整体挪 1~3 个相邻任务）交换，每次交换用真实
   路径相关代价重评（`evalPairJoint` 沿新历史重算，逐步重择翻转），只接受可行且更优的。**仅在
   重排模式（`allow_reorder=true`）+ 关节代价下运行**（保序模式不能重排）；抓取块保持 beam 所选
   不变，只搜顺序 + 翻转。用**前缀状态缓存**把每候选重放限制在首个变化位置之后，配 `two_opt_max_ms`
   预算（默认 40ms，accept-first、超时即返回当前最优）保证实时。DAG 合法性：候选顺序须满足所有
   `pos[parent]<pos[child]` 才评估。逼近局部最优，仍多项式、仍非全局最优。
2. **加宽 + 多样性 beam**：调大 `reorder_beam_width`（代价线性增长），并对前缀去重/保多样
   （避免 $B$ 条都挤在同一分支），降低「早剪掉最优前缀」概率。
3. **CP-SAT 当「离线上界/校验器」而非实时求解器**：修掉两个已知实现缺陷后离线跑，用来
   **量化 beam 与图模型最优的差距**，指导调参——但别上真机实时（50ms 证不到 OPTIMAL，
   代码却把 `FEASIBLE` 当成功，等于没有最优证书）。修法：
   - **翻转 bug**：`evalPairJoint` 只返回它自选的较优 flip，`pe.flip==nv.flip` 过滤会把另一个
     flip 节点的入边全删 → CP-SAT 无法自由选翻转（而翻转正是 J6 唯一杠杆）。预计算须**强制
     指定 `nv.flip`** 单独算代价，而非让 `evalPairJoint` 自己挑。
   - **求解预算**：要真最优须 `status==OPTIMAL`（非 `FEASIBLE`）且给足时间。
4. **降低路径相关性本身**（治本但涉及标定/策略）：若能约束每步落点远离 J6 软限位、让 IK
   分支稳定（$\mathbf q_\text{end}$ 近似历史无关），离散化误差就小，图/DP 模型才逼近真实——
   本质是把「连续状态」压回「可离散」。

**当前落地建议**：先做 (1) 2-opt 后处理（对现有 beam 直接加尾，风险最低、收益明确），把
(3) 的 CP-SAT 留作离线基准；(2)(4) 视实测差距再定。

---

## E. 接口与参数

### 输入 `/tetris_plan` 布局
`data[0]=total`，其后按 `stride=(size-1)/total` 切片，每任务：
`[shape, way, place_row×4, place_col×4, pick_u, pick_v, pick_angle(, geom_u, geom_v)(, 4×(row,col))]`。

### 输出 `lucky/MotionPlan`
`MotionTask[]`（顺序即执行顺序），每任务 4 个 base 位姿：`pick_pose` / `pick_hover_pose` /
`place_pose` / `place_hover_pose`。

### 关键参数

| 组 | 参数 | 默认 | 作用 |
| --- | --- | --- | --- |
| 抓取 | `pick_xy_source` / `use_true_pick_plane` / `use_depth_pick_z` | homography / true / true | XY 源 / 平面回退 / 深度 Z |
| 抓取 | `tcp_pick_offset_x/y/z` / `pick_yaw_offset_deg` | 0 | TCP 补偿 / yaw 偏置 |
| 放置 | `use_cell_max_place_z` / `place_release_z_margin` | true / 0.004 | 逐格取最高 / 离面余量 |
| 优化 | `optimize_order` / `allow_reorder` / `use_plan_candidates` | true / true / false | 顺序 / 重排 / 多候选 |
| 优化 | `reorder_beam_width` / `refine_two_opt` / `two_opt_max_ms` | 12 / true / 40 | beam 宽度 / 2-opt 开关 / 局部改善预算(ms) |
| 代价 | `transit_lin_speed_m_s` / `loaded_lin_speed_m_s` | 0.06 / 0.045 | 空载/负载线速度（须与控制器一致）|
| 代价 | `omega_per_v_lin_rad_per_m` | 3.14（lucky 3.23）| $\omega/v$ 比值，`measure_tcp_omega` 标定 |
| 代价 | `wrist_soft_limit_rad` / `wrist_hard_limit_rad` | 4.7 / 6.10 | J6 软/硬限位 |
| 代价 | `wrist_soft_penalty_s_per_rad` / `tie_break_weight` / `transit_j6_samples` | 见源码 | 越限罚 / 平局项 / 采样数 |

标定数据（`BOARD_POSE_BASE`、`BOARD_CENTERS_14x10_BOARD`、`PLACE_Z_MAP_14x10`、
`PICK_SURFACE_PLANE_BASE`、`PICK_HOMOGRAPHY`）从 `/tetris/*` 读，由 `tetris_config.yaml`
加载，**当作数据勿手改**（生成见 [calibrate.md](calibrate.md)）。

> 速度类参数须与控制器 `transit_speed_mm_s` / `loaded_transit_speed_mm_s` 对应，否则代价与
> 实际运动脱节；`omega_per_v_lin_rad_per_m` 是**比值**、与速度档无关，改速度不用动它。

---

## F. 运行与测试

```bash
# 感知→策略→路径链路（需臂提供 TF，绝不命令运动）
roslaunch lucky test_path.launch robot_ip:=192.168.1.216
rostopic echo /motion_cmds
```

生产 `lucky.launch` 端到端起感知/策略/路径/控制（`use_plan_candidates=true`、
`optimize_order=true`、`omega_per_v_lin_rad_per_m=3.23`）。配合控制器单块抓放见
`test_controller.launch`（**会驱动真机**，见 [control.md](control.md)）。

### F.1 验证 2-opt/or-opt 局部改善（`refineOrderTwoOpt`）

改动逻辑已审、编译通过，但**尚未在真机/仿真跑过**。接臂后用 `test_path.launch`（只发
`/motion_cmds`、绝不命令运动）跑一组 `allow_reorder=true` 的任务，看 `path_planner` 日志：

```bash
roslaunch lucky test_path.launch robot_ip:=<ip>
# 触发一次规划后，在 path_planner 输出里核对：
#   [PATH] optimized order+assign (beam=..) ... cost=<C_beam>
#   [PATH] 2-opt/or-opt refine: cost <C_beam> -> <C_ref> (<N> move(s), <t>/40 ms)
```

三项验收：
1. **代价单调不升**：`C_ref ≤ C_beam`（`moves=0` 时不打印该行 = 无改善，属正常）。
2. **实时**：`t ≤ two_opt_max_ms`（默认 40ms）；超预算应看到耗时贴着上限而非爆掉。
3. **DAG 未被破坏**：对输出序列逐任务查「某格正下方 $(r{+}1,c)$ 的块若也在计划里，其执行序号
   必须更靠前」。可 `rostopic echo /tetris_plan_opt`（调试 17-int 帧）比对顺序，或按
   `/motion_cmds` 里 `MotionTask` 的 place 格子人工核对相邻依赖。

排障：`refine_two_opt:=false` 可一键关掉局部改善、回到纯 beam 解做 A/B 对照；`two_opt_max_ms`
调大可换更多改善（离线/仿真时）。若 `C_ref > C_beam` 或依赖被破坏 → 是 bug，勿上真机。
