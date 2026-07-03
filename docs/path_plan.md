# 路径规划节点说明（path_planner_node）

本文档介绍 `src/tly/src/path_planner_node.cpp` 的职责与策略，以及配套测试 launch
`src/tly/launch/test_path.launch`。

`path_planner_node` 是流水线的**大脑**：它消费策略节点的 `/tetris_plan`（或同分候选集
`/tetris_plan_candidates`），把每个任务的「抓取像素 / 放置格子」**全部解算成 base 系可直接
执行的位姿**（含沿 board 法向的悬停），决定腕部 180° 翻转，并对整条抓放序列做**关节空间路程
（真实运动时间）优化**，最后输出 `tly::MotionPlan` 到 `/motion_cmds` 给控制节点。控制节点已是
**纯执行器**——所有坐标换算、标定加载、腕部决策都集中在本节点（见 `docs/control.md`）。

```
vision(/vision/board_state) ┐
strategy(/tetris_plan[_candidates]) ┼─► path_planner_node ─► /motion_cmds (MotionPlan) ─► controller
/xarm/joint_states, camera_info, depth, TF ┘
```

---

## 1. 接口契约

| 方向 | 话题 / 帧 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/tetris_plan` **或** `/tetris_plan_candidates` | `std_msgs/Int32MultiArray` | 策略方案；`use_plan_candidates` 二选一订阅 |
| 订阅 | `/vision/board_state` | `std_msgs/Int32MultiArray` | 抓取候选池（按形状分组），`optimize_order` 时用 |
| 订阅 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参，像素反投影 |
| 订阅 | `/xarm/joint_states` | `sensor_msgs/JointState` | 起点关节角 q（关节代价 + IK 自检）|
| 订阅 | `/xarm/xarm_states` | `xarm_msgs/RobotMsg` | 固件上报位姿，取初始工具朝向（`tool_tilt_from_firmware`）|
| 订阅 | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 抓取 Z 用的对齐深度（`use_depth_pick_z`）|
| 发布 | `/motion_cmds` | `tly/MotionPlan`（latched）| 解算好的抓放序列，控制节点消费 |
| 发布 | `/tetris_plan_opt` | `std_msgs/Int32MultiArray`（latched）| 重排+抓取分配后的 17-int 计划，**仅调试对照** |
| TF | `base ← camera`、`base ← eef` | — | 像素反投影、初始工具朝向、IK 自检 |

`BOARD_POSE_BASE` 以常量 `tf2::Transform`（`board_to_base_` / `base_to_board_`）加载，取代旧
`table_frame` TF；唯一需要的动态 TF 是 `camera → base`（相机随臂动，eye-on-hand）。

### 输入 `/tetris_plan` 布局

`data[0]=任务总数 total`，其后按 `stride = (size-1)/total` 切片。每任务字段（按 `stride`
自适应解析）：

```
[0] shape_type   形状 ID（0~6，见 CLAUDE.md 形状契约）
[1] way          放置朝向（0~3，对应 place_yaw = -way*90°）
[2] place_row*4  放置网格中心行（定点 ×4，/4 还原）
[3] place_col*4  放置网格中心列
[4] pick_u       抓取像素 u
[5] pick_v       抓取像素 v
[6] pick_angle   抓取角度（图像系，度）
[7,8]            （stride≥9）几何中心像素 geom_u/geom_v
[9..16]          （stride≥17）4 个目标格 (row,col)，放置 Z 逐格取最高
```

### 输出 `tly/MotionPlan`

`Header header` + `MotionTask[] tasks`，顺序即执行顺序。每个 `MotionTask` 含 `shape` /
`way`（仅记录）+ 四个 base 位姿：`pick_pose` / `pick_hover_pose` / `place_pose` /
`place_hover_pose`。字段细节见 `docs/control.md`。

---

## 2. 坐标解算（board 系 → base）

每个任务先在 board（table）局部系算出四位姿，再经 `board_to_base_` 换算到 base。

### 抓取点（`computeTablePoses` / `pixelToPickPointTable` / `computePickTable`）

- **XY**：`pick_xy_source=homography`（默认）用 `PICK_HOMOGRAPHY`（pixel → board-XY）+ 高度
  视差补偿；`=depth` 则用深度反投影。
- **Z**：`use_depth_pick_z` 用 RealSense 对齐深度逐块采样（`depth_sample_radius_px`）；深度无效
  时回退「真实抓取平面」`PICK_SURFACE_PLANE_BASE`（`use_true_pick_plane`），再回退水平 `PICK_Z`。
- **yaw**：`yawFromHomography()` 把图像角经单应性映射到 board 平面（像素方向经透视≠物理方向），
  叠加 `pick_yaw_offset`。
- **TCP 侧向补偿**：`tcp_pick_offset_x/y` 按 `pick_yaw` 旋转后加到 XY，`tcp_pick_offset_z` 加到 Z。

### 放置点（`placeZFromTargetCells` / `bilinearBoardCenter`）

- **XY**：`BOARD_CENTERS_14x10_BOARD` 双线性插值得目标格心；`tcp_place_offset_x/y` 按 `place_yaw`
  旋转后叠加。
- **Z**：逐格 `PLACE_Z_MAP_14x10`；`use_cell_max_place_z` 且任务带目标格时取这些格的**最高**
  place-Z（保证不压到凸起），再加 `place_release_z_margin` 与 `tcp_place_offset_z`。
- **yaw**：`place_yaw = -way * 90°`。
- **几何中心对齐**：若任务带 geom 像素，算 `pick - geom` 偏移，按 `place_yaw - pick_yaw` 旋转后
  补到放置 XY，使方块**几何中心**（而非吸取点）对齐网格。

### 工具下压朝向（roll/pitch）与悬停

- roll/pitch：`read_tool_tilt_from_initial_pose`（默认）时，每次收到 plan 取**臂此刻位姿**（即
  单应性标定位姿）的工具朝向换算到 board 系固定；来源 `tool_tilt_from_firmware`（固件
  `xarm_states.pose`，与 move_line 同运动学系，消除 URDF↔固件 ~1° 差）优先，缺失/过旧回退 TF。
  关掉则用 `fixed_roll_rad`/`fixed_pitch_rad`（默认 π,0）。yaw 始终每任务计算。
- 悬停：`hoverFromTable()` 把 pick/place 抬到 `HOVER_Z`（board +Z 即板面法向），至少高出目标
  10mm，得 `*_hover_pose`。

> 放置面（白板凸起顶面）比抓取面（发光板平面）高一个白板厚，二者基准不同（见 CLAUDE.md 叠放
> 说明）；本节点在 Z 解算里已分别处理。

---

## 3. 腕部翻转 + 关节空间路程优化

**关节代价 = 沿 `move_line` 直线路径积分的真实运动时间**（取代旧的 XY 直线距离）：

- 每段基准笛卡尔时间 = `max(Δd / v_lin, Δθ / ω)`（位置线速度与姿态角速度取瓶颈），再对关节饱和
  取 `max(Δt_nominal, maxᵢ|Δq| / v_max,i)`；用 xArm6 闭式 FK + seeded DLS IK（`tly/xarm6_kinematics.hpp`）
  把 TCP 位姿转成 6 关节角。
- ω 随线速度自适应：`ω = omega_per_v_lin_rad_per_m × v_lin`（实测比值≈π，与档位无关，用
  `measure_tcp_omega.launch` 复测更新，见 `docs/calibrate.md`）。后果：一个 180° 翻转 ≈ 1m 平移的
  时间代价。
- 转移段沿直线笛卡尔路径采样 J6（`transit_j6_samples`）：J6≈heading−J1，中途可越过端点值。

**联合优化**（统一最小化上面的时间代价）：放置顺序（DAG 内重排，`allow_reorder`）× 同形状抓取
分配（从 board_state 池选物理块）× 腕部 **A(不翻)/B(翻180°)** 翻转。翻转并入代价：两套各算取小；
**J6 软限位 `wrist_soft_limit_rad`** 越界按 `wrist_soft_penalty_s_per_rad` 加罚（仍可用，least-bad
有序），**硬限位 `wrist_hard_limit_rad`** 绝对拒发（到 ±2π 留余量）。平局用 `Σ|Δq|` 次级项
（`tie_break_weight`）偏好腕部少甩。

起点 q 取自 `/xarm/joint_states`；启动用 fk/ik 对照 `base→link_tcp` TF **自检**
（`ik_selfcheck_tol_rad`），失败则回退旧的 XY 直线代价作安全网（`use_joint_cost=false`）。

> 吸盘对 180° 对称，翻转不改变抓放落点，只改腕部转角——这是唯一能用笛卡尔指令表达的自由度
> （圈数无法命令，固件按就近解，见 CLAUDE.md 腕部踩坑）。

---

## 4. 多候选择优 · 顺序优化 · 抓取池

- **多候选**（`use_plan_candidates`）：策略节点把若干同分候选布局发到
  `/tetris_plan_candidates`；本节点逐个求关节代价（`parallel_candidate_eval` 节点内多线程），
  选**总代价最小**者执行。关掉则订阅单计划 `/tetris_plan`（旧行为）。
- **顺序优化**（`optimize_order`）：需 `/vision/board_state` 提供抓取候选池（按形状分组）。
  `allow_reorder=true` 在 DAG 内按代价重排放置顺序；`false` 保持策略给的顺序（比赛可能有相邻等
  DAG 外约束），仅为每个槽的指定形状选代价最小的物理块 + 翻转。

---

## 5. 关键参数（节点私有）

| 组 | 参数 | 默认 | 作用 |
| --- | --- | --- | --- |
| 话题 | `plan_topic` / `plan_candidates_topic` / `motion_topic` / `optimized_plan_topic` | 见源码 | 输入计划 / 候选 / MotionPlan 输出 / 调试计划 |
| 帧 | `base_frame` / `camera_frame` / `eef_frame` | `link_base` / `camera_color_optical_frame` / `link_tcp` | TF 帧名 |
| 抓取 | `use_pick_homography` / `pick_xy_source` | true / `homography` | 抓取 XY 来源 |
| 抓取 | `use_true_pick_plane` | true | 深度无效时用拟合平面取 Z |
| 抓取 | `use_depth_pick_z` / `depth_sample_radius_px` | true / 4 | RealSense 逐块抓取 Z |
| 抓取 | `pick_yaw_offset_deg` / `yaw_homography_probe_px` / `yaw_homography_v_sign` | 0 / 30 / +1 | 抓取 yaw 偏置 / 测角探针 |
| 抓取 | `tcp_pick_offset_x/y/z` | 0 | TCP 抓取侧向/深度补偿 |
| 放置 | `use_board_map` / `require_board_map` | true | 放置 XY 用 14×10 网格 |
| 放置 | `use_cell_max_place_z` / `require_place_z_map` | true | 放置 Z 取目标格最高 |
| 放置 | `place_release_z_margin` / `tcp_place_offset_x/y/z` | 0.004 / 0 | 放置离面余量 / TCP 补偿 |
| 朝向 | `read_tool_tilt_from_initial_pose` / `tool_tilt_from_firmware` | true / true | 下压 roll/pitch 来源 |
| 朝向 | `fixed_roll_rad` / `fixed_pitch_rad` | π / 0 | 关闭 tilt 读取时的固定 roll/pitch |
| 优化 | `optimize_order` / `allow_reorder` | true / true | 顺序优化 / 允许重排 |
| 优化 | `use_plan_candidates` / `parallel_candidate_eval` | false / true | 多候选择优 / 并行评估 |
| 代价 | `transit_lin_speed_m_s` / `loaded_lin_speed_m_s` | 0.06 / 0.045 | 空载/负载 TCP 线速度（须与控制器一致）|
| 代价 | `omega_per_v_lin_rad_per_m` | 3.14（tly 用 3.23）| ω/v_lin 比值，`measure_tcp_omega` 标定 |
| 代价 | `joint_max_vel_rad_s` / `wrist_soft_limit_rad` / `wrist_hard_limit_rad` | 3.14×6 / 4.7 / 6.10 | 关节限速 / J6 软·硬限位 |
| 代价 | `wrist_soft_penalty_s_per_rad` / `tie_break_weight_s_per_rad` / `transit_j6_samples` / `ik_selfcheck_tol_rad` | 见源码 | 越限罚 / 平局项 / J6 采样 / 自检阈值 |
| 安全 | `max_tasks_per_plan` | 0（不限）| 发布任务数上限（控制器另有 1 的执行闸）|

标定数据（`BOARD_POSE_BASE`、`BOARD_CENTERS_14x10_BOARD`、`PLACE_Z_MAP_14x10`、
`PICK_SURFACE_PLANE_BASE`、`PICK_HOMOGRAPHY`）从全局 `/tetris/*` 读取，由 `tetris_config.yaml`
加载，**当作数据、勿手改**（生成见 `docs/calibrate.md`）。

> 速度类参数 `transit_lin_speed_m_s` / `loaded_lin_speed_m_s` 是代价模型用的 TCP 线速度，须与
> 控制器 `transit_speed_mm_s` / `loaded_transit_speed_mm_s` 对应（否则代价与实际运动脱节）；而
> `omega_per_v_lin_rad_per_m` 是**比值**、与速度档无关，改速度不用动它。

---

## 6. 运行与测试

- **生产**：`tly.launch` 端到端起感知/策略/路径/控制；那里 `use_plan_candidates=true`、
  `optimize_order=true`，`omega_per_v_lin_rad_per_m=3.23`。
- **仅链路自测**：`test_path.launch` 起 **感知→策略→路径**（需臂提供 TF，但**绝不命令运动**），
  只发 `/motion_cmds` 供观察：

  ```bash
  roslaunch tly test_path.launch robot_ip:=192.168.1.216
  rostopic echo /motion_cmds
  ```

- **配合控制器单块抓放**：`test_controller.launch` 会同时起 path_planner + 控制器，手动喂
  `/tetris_plan` 触发一次（**会驱动真机**，见 `docs/control.md`）。
