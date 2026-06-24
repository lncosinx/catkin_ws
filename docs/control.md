# 控制节点策略说明（xarm_controller_node）

本文档介绍 `src/tly/src/xarm_controller_node.cpp` 的控制策略，以及配套测试
launch `src/tly/launch/test_controller.launch` 的用法。

控制节点是流水线的最后一环：它消费策略节点发布的 `/tetris_plan`，把每个任务
（一个方块的「抓取像素 + 放置网格」）转换成机器人 base 系下的具体位姿，再通过
xArm **原生服务**（不经 MoveIt）逐步执行抓取—搬运—放置。

---

## 1. 接口契约

| 方向 | 话题 / 服务 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/tetris_plan` | `std_msgs/Int32MultiArray` | 策略方案，消费一次；`count + 每任务 stride 个 int` |
| 订阅 | `/xarm/xarm_states` | `xarm_msgs/RobotMsg` | 读取当前 6D native 位姿（解绕 / yaw 预测起点）|
| 订阅 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 P，用于像素反投影 |
| 发布 | `/robot_status` | `std_msgs/Bool`（latched）| 忙 / 闲标志 |
| 调用 | `/xarm/set_mode`、`/xarm/set_state` | `xarm_msgs/SetInt16` | 进入原生位置模式（mode=0, state=0）|
| 调用 | `/xarm/move_line` | `xarm_msgs/Move` | 直线运动到目标 base 位姿 |
| 调用 | `/xarm/set_controller_dout` | `xarm_msgs/SetDigitalIO` | 吸盘开关（`suction_io_num`）|

`/tetris_plan` 数据布局：`data[0]=任务总数 total`，其后按 `stride = (size-1)/total`
切片。每任务字段（节点按 `stride` 自适应解析）：

```
[0] shape_type   形状 ID（0~6，见 CLAUDE.md 形状契约）
[1] way          放置朝向（0~3，对应 yaw = -way*90°）
[2] place_row*4  放置网格中心行（定点 ×4，故 /4 还原）
[3] place_col*4  放置网格中心列
[4] pick_u       抓取像素 u
[5] pick_v       抓取像素 v
[6] pick_angle   抓取角度（图像系，度）
[7,8]            （stride≥9）几何中心像素 geom_u/geom_v
[9..16]          （stride≥17）4 个目标格 (row,col)，用于逐格 place-Z 取最高
```

---

## 2. 坐标转换策略（base 原生）

整条流水线是 **base 原生** 的，不再依赖旧的 `table_frame` TF：

- `BOARD_POSE_BASE`（来自标定，存于 `tetris_config.yaml`）作为常量 `tf2::Transform`
  加载为 `board_to_base_` / `base_to_board_`，描述 board 系在 base 系的位姿。
- **唯一需要的动态 TF 是 `camera_color_optical_frame -> link_base`**（相机随臂动，
  eye-on-hand）。board↔base 用常量合成：`cam->board = base_to_board_ * cam->base`。

### 抓取点求解（pixel → board XY/Z）

`pixelToPickPointTable()` 分两步：

1. **3D 射线求交拿高度**：把像素反投影成相机系射线，与「真实抓取平面」
   `PICK_SURFACE_PLANE_BASE`（`use_true_pick_plane`，深度拟合得到的发光板平面）
   求交，得到带正确 `z` 的物理落点。无该平面时退化为水平 `PICK_Z` 平面。
2. **单应性修正 XY + 视差补偿**：若加载了 `PICK_HOMOGRAPHY`（pixel → board-XY），
   先做「高度视差（parallax）补偿」——把方块顶面像素按相机/方块高度比向光心
   `(cx,cy)` 拉回，等效把顶面「拍扁」到标定平面，再喂给单应性得到精确 XY。
   **`z` 始终保留第 1 步的下压高度，绝不被单应性覆盖。**

### 放置点求解

- XY：`BOARD_CENTERS_14x10_BOARD` 做双线性插值得到目标网格中心
  （`bilinearBoardCenter`），落到 board 局部坐标。
- Z：`placeZFromTargetCells()` 用逐格 `PLACE_Z_MAP_14x10`。当 `use_cell_max_place_z`
  打开且任务带 4 个目标格时，取这些格的 **最高** place-Z（保证方块不压到凸起），
  再加 `place_release_z_margin` 与波纹管补偿 `TCP_PLACE_OFFSET_Z`。

> 放置面（白板凸起顶面）比抓取面（发光板平面）高一个白板厚，二者基准不同，
> 见 CLAUDE.md 的叠放说明。

---

## 3. 朝向（yaw）策略

roll/pitch 固定为 `fixed_roll_rad`（默认 π）/ `fixed_pitch_rad`（默认 0），只有
yaw 随任务变化，且 **不连续累计、不解绕历史**。

- **抓取 yaw**：`yawFromHomography()` 把图像角度经单应性映射到 board 平面（因为
  像素方向经透视后并不等于物理方向），再叠加 `pick_yaw_offset`。
- **放置 yaw**：由 `way` 决定，`place_yaw = -way * 90°`。
- **180° 翻转择优**：`buildTask()` 用 `sim_yaw` 预测器（起点取机械臂当前真实
  yaw）评估「不翻转 / 翻转 180°」两套方案在解绕后的累计绝对角，选 **关节转动更小**
  的一套。由于吸盘对称，180° 翻转不改变抓放结果，但能避免接近关节限位 / 大回转。
- 几何中心偏移：若任务带 geom 像素，计算 `pick - geom` 偏移，并按
  `place_yaw - pick_yaw` 旋转后补到放置点，使方块「几何中心」而非「吸取点」对齐网格。

`moveLineNative()` 下发前对 roll/pitch/yaw 逐轴 `unwrapAngle`，相对当前 native
位姿就近解绕，避免整圈回绕。

---

## 4. 执行状态机

`controlLoop`（定时器，`control_period` 默认 50ms）驱动逐任务状态机，单个方案
最多执行 `max_tasks_per_plan` 个任务：

```
IDLE
  └─(收到 plan, 构建任务队列)→ TAKE_NEXT_TASK
TAKE_NEXT_TASK  取队首任务（空则 FINISH）
  → MOVE_TO_PICK_HOVER   直线到抓取上方 hover（transit 速度）
  → EXECUTE_PICK         下压(pick_down) → 吸盘 ON → 抬起(lift)
  → MOVE_TO_PLACE_HOVER  负载搬运到放置上方（loaded_transit 速度）
  → EXECUTE_PLACE        下压(place_down) → 可选抖动 → 吸盘 OFF → 抬起
  → TAKE_NEXT_TASK       回到取下一个任务
FINISH  发布 busy=false，回 IDLE
```

- **hover 高度**：`HOVER_Z`（board 系），并保证至少高出目标 50mm。
- **放置抖动**（`enable_place_shake`）：在放置面、吸盘仍 ON 时做对角线往复挤压
  `place_shake_cycles` 次（±`dx`/`dy`），回正后再断吸盘，帮助方块卡进网格凸起。
- **运动校验**：`verify_native_xyz_after_motion` 打开时，每段运动后对比 native
  XYZ 与目标，误差超 `native_xyz_tolerance_mm` 视为失败。
- **失败处理**：`failOrFinish()`。`stop_on_motion_failure=true`（默认）时清空队列、
  停机回 IDLE。注意若 **抓取下压失败吸盘不会开**，但 **放置阶段失败吸盘仍保持 ON**
  （需人工介入）。
- **急停**：`Ctrl+C` → `emergencyStop()` 调 `set_state=4`（STOP）。

---

## 5. 关键参数（节点私有）

| 参数 | 默认/来源 | 作用 |
| --- | --- | --- |
| `max_tasks_per_plan` | 1 | 单方案最多执行的任务数（安全闸）|
| `use_true_pick_plane` | true | 用拟合抓取平面取深度 |
| `use_pick_homography` | true | 启用 pixel→board-XY 单应性 + 视差补偿 |
| `use_board_map` / `require_board_map` | true | 放置 XY 用 14×10 网格中心 |
| `use_cell_max_place_z` / `require_place_z_map` | true | 放置 Z 取目标格最高 |
| `pick_yaw_offset_deg` | 0（test 中 90）| 抓取 yaw 偏置 |
| `yaw_homography_probe_px` / `yaw_homography_v_sign` | 30 / +1 | 单应性测角探针长度 / 图像 v 方向符号 |
| `tcp_pick_offset_z` / `tcp_place_offset_z` | `/tetris/TCP_*_OFFSET_Z` | 波纹管 Z 补偿，标定写入、launch 可覆盖 |
| `place_release_z_margin` | 0.004 | 放置离面余量 |
| `*_speed_mm_s` / `*_acc_mm_s2` | 见源码 | 各阶段 move_line 速度/加速度（native 单位）|
| `enable_place_shake` 及其子项 | false | 放置前抖动卡位 |
| `suction_io_num` / `*_wait` | 1 / 0.25,0.20 | 吸盘 IO 口与开关后等待 |

标定数据（`BOARD_POSE_BASE`、`BOARD_CENTERS_14x10_BOARD`、`PLACE_Z_MAP_14x10`、
`PICK_SURFACE_PLANE_BASE`、`PICK_HOMOGRAPHY`、`TCP_*_OFFSET_Z`）从全局
`/tetris/*` 读取，由 `tetris_config.yaml` 加载，**当作数据、勿手改**。

---

## 6. 用 test_controller.launch 单独测试

`test_controller.launch` 只起 **控制器所需** 的最小依赖，不起 vision/strategy/path：

1. xArm 原生驱动（`xarm6_server.launch`，`use_moveit=false`）+ `robot_state_publisher`
2. RealSense 相机（控制节点启动时等待 `camera_info`）
3. 手眼 TF（`easy_handeye/publish.launch`，eye-on-hand）+ 加载 `tetris_config.yaml`
4. 被测控制节点（`launch-prefix` 延迟 4s 启动，等驱动/相机就绪）；参数与生产
   `tly.launch` 一致，但 `max_tasks_per_plan=1` 仅执行一个任务

```bash
roslaunch tly test_controller.launch robot_ip:=192.168.1.228
```

然后手动喂一个方案触发一次抓放（格式见 `strategy_node` 输出）：

```bash
rostopic pub -1 /tetris_plan std_msgs/Int32MultiArray "..."
```

> ⚠️ 本 launch **会驱动真机**。运行前确认工作区无人、急停可达；默认仅执行 1 个任务。
> （注意 test 文件里把速度档上调到了较高值，按现场酌情调整。）
