# 控制节点说明（xarm_controller_node · 纯执行器）

本文档介绍 `src/lucky/src/xarm_controller_node.cpp` 的控制策略，以及配套测试
launch `src/lucky/launch/test_controller.launch` 的用法。

控制节点是流水线的最后一环，且已重构为**纯执行器（Pure Executor）**：它消费
`path_planner_node` 发布的 `/motion_cmds`（`lucky::MotionPlan`），其中每个任务已经是
**base 系、可直接 `move_line` 的四个位姿**（抓取/放置各一对「下压 + 悬停」）。控制节点
**不做任何坐标换算、不加载任何标定、不做腕部翻转决策**——像素→base 抓取（含深度 Z）、
格子→base 放置、腕部 yaw 翻转择优、沿 board 法向的悬停解算，全部由 `path_planner_node`
完成（见 `docs/path_plan.md` / `src/lucky/src/path_planner_node.cpp`）。控制节点只通过
xArm **原生服务**（不经 MoveIt）逐步执行抓取—搬运—放置。

> 历史说明：早期版本的控制节点自己做坐标转换、单应性/视差、yaw 与 180° 翻转择优。这些
> 现已全部上移到 `path_planner_node`，本节点只剩「执行 + 吸盘 + 抖动 + 到位校验」。

---

## 1. 接口契约

| 方向 | 话题 / 服务 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/motion_cmds` | `lucky/MotionPlan` | path_planner 解算后的抓放序列，消费一次 |
| 订阅 | `/xarm/xarm_states` | `xarm_msgs/RobotMsg` | 读当前 6D native 位姿：RPY 就近解绕 + 到位校验 |
| 订阅 | `/xarm_controller/continue` | `std_msgs/Empty` | Debug 单步模式的「继续」信号（`debug_continue_topic`）|
| 发布 | `/robot_status` | `std_msgs/Bool`（latched）| 忙 / 闲标志 |
| 调用 | `/xarm/set_mode`、`/xarm/set_state` | `xarm_msgs/SetInt16` | 进入原生位置模式（mode=0, state=0）；急停用 state=4 |
| 调用 | `/xarm/move_line` | `xarm_msgs/Move` | 直线运动到目标 base 位姿 |
| 调用 | `/xarm/set_controller_dout` | `xarm_msgs/SetDigitalIO` | 吸盘开关（`suction_io_num`，`io_service` 可改）|

**注意**：本节点已**不再订阅** `/camera/color/camera_info`——像素反投影等感知计算都在
`path_planner_node`。启动时会把 `/xarm/wait_for_finish` 设为 `xarm_wait_for_finish`
（默认 true，即 `move_line` 阻塞到运动完成）。

`/motion_cmds`（`lucky/MotionPlan`）数据布局：`Header header` + `MotionTask[] tasks`，
**顺序即执行顺序**。每个 `MotionTask`：

```
int32 shape                       形状 ID（0~6，仅用于日志/追溯，不参与执行）
int32 way                         放置朝向（0~3，同上仅记录）
geometry_msgs/Pose pick_pose          抓取下压目标 (base)
geometry_msgs/Pose pick_hover_pose    抓取上方悬停 (base, 沿 board 法向抬升)
geometry_msgs/Pose place_pose         放置下压目标 (base)
geometry_msgs/Pose place_hover_pose   放置上方悬停 (base, 沿 board 法向抬升)
```

上游 `/tetris_plan`（`strategy_node` 输出的 `std_msgs/Int32MultiArray`）由
`path_planner_node` 消费并解算成 `MotionPlan`；控制节点不直接读它。

---

## 2. 职责边界：控制节点做什么、不做什么

| | path_planner_node（上游） | xarm_controller_node（本节点） |
| --- | --- | --- |
| 坐标换算 | 像素→base 抓取、格子→base 放置 | ✗ 不做，直接用 plan 里的 base 位姿 |
| 抓取 Z | 深度/平面求高度 | ✗ |
| 放置 Z | 逐格 `PLACE_Z_MAP` 取最高 + 余量 | ✗ |
| 悬停位姿 | 沿 board 法向抬升算出 hover | ✗ 直接用 `*_hover_pose` |
| 腕部 yaw / 180° 翻转 | 择优决策写进位姿 | ✗ 只在下发时逐轴就近解绕 |
| 标定数据 | 加载 `tetris_config.yaml` | ✗ 完全不加载 |
| 执行 / 吸盘 / 抖动 / 到位校验 | ✗ | ✓ 全部在此 |

`MotionTask` 里的 `shape` / `way` 仅供本节点打日志、便于对账，不影响执行——四个位姿已是
最终目标。

> 放置面（白板凸起顶面）比抓取面（发光板平面）高一个白板厚，二者基准不同（见 CLAUDE.md
> 的叠放说明）——但这层差异已在 planner 的 Z 解算里处理完，控制节点无需感知。

### 下发前的姿态解绕（`moveLineNative`）

把 base 位姿（四元数）转成 native `[x_mm,y_mm,z_mm, r,p,y]` 后，先对 yaw 归一化到
`(-π,π]`，再**相对当前上报 native 位姿逐轴（roll/pitch/yaw）就近解绕**（`unwrapAngle`），
避免整圈回绕。J6 的圈数 / 是否翻转已由 planner 预先决策，固件按「就近解」执行（见 CLAUDE.md
腕部踩坑）——本节点的解绕只保证下发角度相对当前姿态连续，不改变 planner 的翻转选择。

---

## 3. 执行状态机

`controlLoop`（定时器，`control_period` 默认 50ms）驱动逐任务状态机，单个方案最多执行
`max_tasks_per_plan` 个任务：

```
IDLE
  └─(收到 MotionPlan, 入队前 max_tasks 个任务)→ TAKE_NEXT_TASK
TAKE_NEXT_TASK  取队首任务（空则 FINISH）
  → MOVE_TO_PICK_HOVER   直线到 pick_hover_pose（transit 速度）
  → EXECUTE_PICK         下压到 pick_pose(pick_down) → 吸盘 ON → 抬回 pick_hover(lift)
  → MOVE_TO_PLACE_HOVER  负载搬运到 place_hover_pose（loaded_transit 速度）
  → EXECUTE_PLACE        下压到 place_pose(place_down) → 可选抖动 → 吸盘 OFF → 抬回 place_hover
  → TAKE_NEXT_TASK       回到取下一个任务
FINISH  发布 busy=false，回 IDLE
```

- **悬停位姿**：直接用任务里的 `pick_hover_pose` / `place_hover_pose`（planner 已沿 board
  法向抬升算好），控制节点不再自算 hover 高度。
- **到位校验**（`verify_native_xyz_after_motion`）：每段运动后**轮询等待「运动后的新鲜上报」
  进入 XYZ 容差**（在 `native_verify_timeout_s` 内、按 `native_verify_poll_hz` 轮询），
  误差超 `native_xyz_tolerance_mm` 或超时拿不到新鲜状态即判失败。取代了固定 sleep。
- **放置抖动**（`enable_place_shake`）：在放置面、吸盘仍 ON 时做对角线往复挤压
  `place_shake_cycles` 次（base 系 ±`dx`/`dy`），回中后再断吸盘，帮助方块卡进网格凸起。
- **Debug 单步**（`debug_step`）：到达 `pick_hover`/`pick`/`place_hover`/`place` 每个路点后
  **暂停**，直到在 `debug_continue_topic`（默认 `/xarm_controller/continue`）发一条
  `std_msgs/Empty` 才继续。单块调试首选。
  ```bash
  rostopic pub -1 /xarm_controller/continue std_msgs/Empty "{}"
  ```
- **失败处理**：`failOrFinish()`。`stop_on_motion_failure=true`（默认）时清空队列、停机回
  IDLE；否则跳过当前任务取下一个。注意若 **抓取下压失败吸盘不会开**，但 **放置阶段失败
  吸盘仍保持 ON**（需人工介入）。
- **急停**：`Ctrl+C` → `emergencyStop()` 调 `set_state=4`（STOP）。

---

## 4. 关键参数（节点私有）

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `plan_topic` | `/motion_cmds` | 订阅的 MotionPlan 话题 |
| `max_tasks_per_plan` | 1 | 单方案最多执行的任务数（安全闸，`>0` 才截断）|
| `stop_on_motion_failure` | true | 失败即清队停机；false 则跳过继续 |
| `control_period` | 0.05 | 状态机定时器周期（s）|
| `xarm_wait_for_finish` | true | 启动时写 `/xarm/wait_for_finish`，move_line 阻塞到完成 |
| `verify_native_xyz_after_motion` | true | 每段运动后做 native XYZ 到位校验 |
| `native_xyz_tolerance_mm` | 6.0 | 到位判定 XYZ 容差（mm）|
| `native_verify_timeout_s` / `native_verify_poll_hz` | 1.0 / 50 | 校验轮询超时 / 频率 |
| `transit_*` / `loaded_transit_*` | 60/45 mm/s… | 空载 / 负载搬运的 move_line 速度、加速度 |
| `pick_down_*` / `place_down_*` / `lift_*` | 见源码 | 抓取下压 / 放置下压 / 抬起的速度、加速度 |
| `enable_place_shake` 及其子项 | false | 放置前抖动卡位（cycles/dx/dy/speed/acc）|
| `suction_io_num` | 1 | 吸盘数字 IO 口 |
| `suction_on_wait` / `suction_off_wait` | 0.25 / 0.20 | 吸盘开 / 关后等待（s）|
| `io_service` | `/xarm/set_controller_dout` | 吸盘 IO 服务名 |
| `debug_step` / `debug_continue_topic` | false / `/xarm_controller/continue` | 单步暂停 + 继续话题 |

本节点**不读任何 `/tetris/*` 标定参数**——所有标定由 `path_planner_node` 加载并已折进
`/motion_cmds` 的位姿里。速度类参数是 xArm native `move_line` 的 mm/s、mm/s²。

---

## 5. 用 test_controller.launch 单独测试

`test_controller.launch` 只起 **控制器所需** 的最小依赖（含 path_planner），不起
vision/strategy：

1. xArm 原生驱动（`xarm6_server.launch`，`use_moveit=false`）+ `robot_state_publisher`
2. RealSense 相机（`path_planner` 需要 `camera_info` 做像素反投影）
3. 手眼 TF（`easy_handeye/publish.launch`，eye-on-hand）+ 加载 `tetris_config.yaml`
4. **`path_planner_node`**：消费手动喂的 `/tetris_plan`，完成坐标/深度解算与腕部翻转，
   发布 `MotionPlan` 到 `/motion_cmds`（`optimize_order=false`，单任务手动测试不排序）
5. 被测 **控制节点**（纯执行器）：消费 `/motion_cmds`；测试里 `debug_step=true` 单步、
   `max_tasks_per_plan=1` 仅执行一个任务

```bash
roslaunch lucky test_controller.launch robot_ip:=192.168.1.216
```

然后手动喂一个方案（`/tetris_plan`，格式见 `strategy_node` 输出：count + 每任务 17 个 int），
经 path_planner 解算成 `/motion_cmds` 触发一次抓放：

```bash
rostopic pub -1 /tetris_plan std_msgs/Int32MultiArray "..."
```

单步模式下每到一个路点会暂停，用 `/xarm_controller/continue` 逐步放行：

```bash
rostopic pub -1 /xarm_controller/continue std_msgs/Empty "{}"
```

> ⚠️ 本 launch **会驱动真机**。运行前确认工作区无人、急停可达；默认仅执行 1 个任务。
> 注意 test 文件里把各段速度上调到了较高值（如 transit 500mm/s），按现场酌情调整。
