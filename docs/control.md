# 控制子系统技术文档（xarm_controller_node · 纯执行器）

控制节点是流水线最后一环，已重构为**纯执行器（Pure Executor）**：消费 path_planner 发布的
`/motion_cmds`（`lucky::MotionPlan`），其中每任务已是 **base 系、可直接 `move_line` 的四个
位姿**。控制节点**不做任何坐标换算、不加载标定、不做腕部翻转决策**，只通过 xArm **原生
服务**（不经 MoveIt）逐步执行抓—搬—放，并做姿态解绕、吸盘、抖动、到位校验。本文说明它对
每个位姿做哪些数据处理及其**数学公式**。

参考代码：
- 节点：[xarm_controller_node.cpp](../src/lucky/src/xarm_controller_node.cpp)
- 测试：[test_controller.launch](../src/lucky/launch/test_controller.launch)

---

## 0. 数据流总览

```
/motion_cmds (MotionTask[]：每任务 4 个 base 位姿)
   │
   ▼ [1] 状态机逐任务   IDLE→PICK_HOVER→PICK→PLACE_HOVER→PLACE→…→FINISH
   ▼ [2] 位姿→native    四元数→RPY，米→毫米
   ▼ [3] 逐轴就近解绕   相对当前上报位姿 unwrap，避免整圈回绕
   ▼ [4] move_line 下发  原生直线运动服务
   ▼ [5] 到位校验        轮询新鲜上报，XYZ 欧氏误差 ≤ 容差
   ▼ 吸盘 IO / 放置抖动 / busy 标志
```

### 职责边界

| | path_planner（上游） | xarm_controller（本节点） |
| --- | --- | --- |
| 坐标换算 / 抓放 Z / 悬停 / 腕部翻转 / 标定 | ✓ 全部 | ✗ 直接用 plan 位姿 |
| 姿态解绕 / 执行 / 吸盘 / 抖动 / 到位校验 | ✗ | ✓ 全部 |

`MotionTask` 里的 `shape` / `way` 仅供打日志对账，不参与执行。

---

## 1. 执行状态机

`controlLoop`（定时器 `control_period` 默认 50ms）驱动逐任务状态机，单方案最多执行
`max_tasks_per_plan` 个（默认 1，安全闸）：

```
IDLE ──(收到 MotionPlan)──► TAKE_NEXT_TASK
  → MOVE_TO_PICK_HOVER   直线到 pick_hover_pose（transit 速度）
  → EXECUTE_PICK         下压到 pick_pose → 吸盘 ON → 抬回 pick_hover
  → MOVE_TO_PLACE_HOVER  负载搬运到 place_hover_pose（loaded 速度）
  → EXECUTE_PLACE        下压到 place_pose → 可选抖动 → 吸盘 OFF → 抬回 place_hover
  → TAKE_NEXT_TASK       取下一个任务
FINISH  发布 busy=false，回 IDLE
```

- **悬停位姿**直接用任务里的 `*_hover_pose`（planner 已沿 board 法向抬升算好）。
- **失败处理** `failOrFinish`：`stop_on_motion_failure=true` 清队停机回 IDLE，否则跳过取下
  一个。注意抓取下压失败吸盘不会开，但**放置阶段失败吸盘仍 ON**（需人工介入）。
- **急停**：`Ctrl+C` → `emergencyStop()` 调 `set_state=4`（STOP）。
- **Debug 单步**（`debug_step`）：每个路点后暂停，等 `/xarm_controller/continue` 发
  `std_msgs/Empty` 才继续。

---

## 2. 位姿 → native 转换（`moveLineNative`）

xArm 原生 `move_line` 接受 `[x_mm, y_mm, z_mm, roll, pitch, yaw]`。从 base 四元数
$\mathbf q=(x,y,z,w)$ 转 RPY（Z-Y-X 内旋约定），位置米转毫米：

$$\text{tgt}=\big[\,1000\,p_x,\ 1000\,p_y,\ 1000\,p_z,\ r,\ p,\ y\,\big]$$

四元数到欧拉角（tf2 `getRPY`）：

$$r=\operatorname{atan2}\!\big(2(wx+yz),\ 1-2(x^2+y^2)\big)$$
$$p=\arcsin\!\big(2(wy-zx)\big)$$
$$y=\operatorname{atan2}\!\big(2(wz+xy),\ 1-2(y^2+z^2)\big)$$

---

## 3. 逐轴就近解绕（关键）

**问题**：欧拉角有 $2\pi$ 多值性，直接下发规范化角可能让固件走一整圈回绕。**方法**：先把
yaw 归一化到 $(-\pi,\pi]$，再对 roll/pitch/yaw **每轴相对当前上报 native 位姿就近解绕**，
使下发角与当前姿态连续。

**归一化**（`normalizeAngleRad`）把角模 $2\pi$ 折进 $(-\pi,\pi]$，即取唯一的

$$\operatorname{norm}(a)\equiv a\ (\mathrm{mod}\ 2\pi),\qquad \operatorname{norm}(a)\in(-\pi,\pi]$$

**就近解绕**（`unwrapAngle`）把 target 移到离 current 最近的等价角——先算差值 $d=t-c$，把
$d$ 模 $2\pi$ 折进 $[-\pi,\pi]$ 得 $\operatorname{wrap}(d)$，再

$$\operatorname{unwrap}(c,t)=c+\operatorname{wrap}(t-c),\qquad \operatorname{wrap}(t-c)\equiv t-c\ (\mathrm{mod}\ 2\pi)$$

其关键性质是 $\operatorname{unwrap}(c,t)\equiv t\ (\mathrm{mod}\ 2\pi)$ 且 $|\operatorname{unwrap}(c,t)-c|\le\pi$
（下发角与当前姿态的差不超过半圈，故绝不整圈回绕）。对 $r,p,y$ 三轴各做一次：
$\text{tgt}[3..5]\leftarrow\operatorname{unwrap}(\text{native}[3..5],\ \text{tgt}[3..5])$。

> J6 的圈数 / 是否翻转已由 planner 预先决策，固件按「就近解」执行（见 CLAUDE.md 腕部踩坑）；
> 本节点的解绕只保证下发角相对当前姿态连续，**不改变** planner 的翻转选择。

---

## 4. 到位校验（`verifyReachedXYZ`）

取代固定 sleep。每段运动后**轮询等待「运动后的新鲜上报」进入 XYZ 容差**：在
`native_verify_timeout_s` 内、按 `native_verify_poll_hz` 轮询，只认时间戳晚于进入时刻
$t_\text{enter}$ 的新鲜状态，计算与目标的**欧氏距离**（mm）：

$$\varepsilon=\sqrt{(c_x-\text{tgt}_x)^2+(c_y-\text{tgt}_y)^2+(c_z-\text{tgt}_z)^2}$$

$\varepsilon\le\texttt{native\_xyz\_tolerance\_mm}$（默认 6mm）判到位成功；超时仍超差或拿不到
新鲜状态即判失败（触发 `failOrFinish`）。

---

## 5. 放置抖动（`executePlaceShake`）

`enable_place_shake` 开启时，在放置面、吸盘仍 ON 时做**对角往复挤压**帮助方块卡进网格
凸起。每周期以放置中心 $\mathbf c$ 为基准，base 系沿对角 $\pm(\Delta x,\Delta y)$ 往返再回中：

$$\mathbf c+(\Delta x,\Delta y)\ \to\ \mathbf c-(\Delta x,\Delta y)\ \to\ \mathbf c$$

重复 `place_shake_cycles` 次，回中后再断吸盘。每段都是完整的 `moveLineNative`（含解绕 +
到位校验）。

---

## 6. 接口与参数

### 输入 `/motion_cmds`（`lucky/MotionPlan`）
`Header header` + `MotionTask[] tasks`（顺序即执行顺序）。每 `MotionTask`：
```
int32 shape / way                     仅日志，不参与执行
geometry_msgs/Pose pick_pose          抓取下压 (base)
geometry_msgs/Pose pick_hover_pose    抓取悬停 (base, 沿 board 法向)
geometry_msgs/Pose place_pose         放置下压 (base)
geometry_msgs/Pose place_hover_pose   放置悬停 (base, 沿 board 法向)
```

### 服务调用
- `/xarm/set_mode`+`/xarm/set_state`：进原生位置模式（mode=0,state=0），急停 state=4。
- `/xarm/move_line`：直线运动。
- `/xarm/set_controller_dout`：吸盘 IO（`suction_io_num`）。
- 启动写 `/xarm/wait_for_finish`（默认 true，`move_line` 阻塞到完成）。

### 关键参数

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `max_tasks_per_plan` | 1 | 单方案最多执行任务数（安全闸）|
| `stop_on_motion_failure` | true | 失败即清队停机 |
| `verify_native_xyz_after_motion` / `native_xyz_tolerance_mm` | true / 6.0 | 到位校验 / 容差 |
| `native_verify_timeout_s` / `native_verify_poll_hz` | 1.0 / 50 | 校验超时 / 频率 |
| `transit_*` / `loaded_transit_*` / `pick_down_*` / `place_down_*` / `lift_*` | 见源码 | 各段 move_line 速度/加速度（mm/s, mm/s²）|
| `enable_place_shake` 及子项 | false | 抖动卡位（cycles/dx/dy/speed/acc）|
| `suction_io_num` / `suction_on_wait` / `suction_off_wait` | 1 / 0.25 / 0.20 | 吸盘 IO 口 / 开关等待 |
| `debug_step` / `debug_continue_topic` | false / `/xarm_controller/continue` | 单步暂停 + 继续话题 |

本节点**不读任何 `/tetris/*` 标定**——全部由 path_planner 折进 `/motion_cmds` 位姿里。

---

## 7. 用 test_controller.launch 单独测试

`test_controller.launch` 起控制器所需最小依赖（含 path_planner，不起 vision/strategy）。

```bash
roslaunch lucky test_controller.launch robot_ip:=192.168.1.216
# 手动喂一个方案（17-int，经 path_planner 解算成 /motion_cmds 触发一次抓放）
rostopic pub -1 /tetris_plan std_msgs/Int32MultiArray "..."
# 单步模式逐点放行
rostopic pub -1 /xarm_controller/continue std_msgs/Empty "{}"
```

> ⚠️ 本 launch **会驱动真机**。运行前确认工作区无人、急停可达；默认仅执行 1 个任务。test
> 文件里各段速度上调到了较高值（如 transit 500mm/s），按现场酌情调整。
