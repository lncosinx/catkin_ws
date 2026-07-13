# 注意事项与踩坑汇总

本文汇总操作安全、机械臂运动学、标定、坐标系、代码结构等各方面的**注意事项**与**已踩过的坑**。
来源：`CLAUDE.md` + 历次真机排查记录。多数条目给出「现象 → 根因 → 做法」，遇到相关症状先来这里查。

---

## 1. 操作安全（上真机前必读）

- ⚠️ **示教器急停常备**：任何真机运行时，控制器（示教器）必须在手边，随时能按急停。
- ⚠️ **相机数据线务必设最大速度/加速度上限**，防止运动中把线折断（已发生过）。
- **电磁阀（吸盘）**：控制口 **CO0**，对应服务 `/xarm/set_controller_dout`，`io_num=1`。
- **容器内开 GUI**：宿主机终端先执行 `xhost +local:docker`。
- **看相机原始画面**：`realsense-viewer`。

---

## 2. 机械臂关节与运动学（最容易出事的一类）

### 2.1 启动 MoveIt / realMove_exec 前，J1/J4/J6 必须远离 ±360° 边界
凡包含 `xarm6_moveit_config/realMove_exec.launch` 的 launch（`calibrate_tool.launch`、旧
`test_pick.launch` 等），在打印 `Started controllers: xarm6_traj_controller, joint_state_controller`
那一刻，MoveIt 会把臂从 UF Studio 位姿模式切到 **SERVO 关节伺服模式**。
- **现象**：若某个 ±2π 关节（J1/J4/J6）此时绕在接近 ±360° 处，切模式瞬间会触发一次**未经规划的
  大幅摆动**。实测 J4=−358.4°（距硬限仅 1.6°）启动 → J4 甩约 180°、触发保护停止（state 5）。
- **根因**：固件切 SERVO 时对绕到边界的关节做「就近解」重解算、朝规范角解缠。**不是 ROS 层命令**
  （已核实 `xarm_hw` 初始 cmd=当前角、SDK 不归一化、URDF 限位 ±2π 不裁剪）。
- **做法**：启动前在 UF Studio 把 J1/J4/J6 摇到规范区间**中段**再 launch；若「一启动就乱动」，
  先查 `/xarm/joint_states` 有无关节贴近 ±2π。

### 2.2 腕部 yaw / J6：笛卡尔无法表达「圈数」，只能预防不能补救
xArm 笛卡尔接口（`move_line` / `set_position`，MoveIt Pilz LIN 同理）把命令 RPY 转旋转矩阵 `R`
做 IK，而 `R(yaw) ≡ R(yaw±2π)`——**"圈数/winding" 无法经笛卡尔指令表达**，固件按「就近解」自行决定
J6 落哪一圈。
- 任何想靠命令 `+2π·k` 绕圈来躲 J6 限位的做法都**无效**（实测在 UF Studio 把 yaw 从 −73° 命令到
  287°，差正好 360°，机械臂纹丝不动）。历史上 `path_planner` 的 J6 绕圈 DP 已因此撤除。
- **一旦 J6 越过约 ±180° 就再也无法用笛卡尔指令拉回**（就近解会往更远那圈走）→ 策略只能**预防**。
- 唯一能主动用的自由度是 **180° 翻转**（`flip`，真正改变 `R`，固件照做；吸盘对 180° 对称，
  pick/place 同步翻转后落点不变）。
- **谁在做择优（注意 CLAUDE.md 此段已过时）**：A（不翻）/B（翻 180°）的选择现在由
  **`path_planner_node`** 负责（不再是控制节点 `buildTask`）。目标是**关节空间路程最小**
  （各轴同时到达，多转的角会成为整段速度瓶颈）；J6 软/硬限位（`wrist_soft_limit_rad` /
  `wrist_hard_limit_rad`）作为惩罚/排除**约束**。详见 [path_plan.md](path_plan.md)。

### 2.3 J6 可能在「运输途中」越限（不只端点）
只在端点检查 J6 限位不够：直线 `move_line` 上 J1 非线性摆动（近底座快），J6≈heading−J1 会在**途中**
超过两端点。`path_planner` 已改为**沿运输段采样 J6**（`transit_j6_samples`，IK 连续 seed）取路径
`max|J6|`。若没有任何 flip/换块/重排能把运输段压进硬限位，`optimizePlan` 直接 **fail-stop**
（打印 `PLANNING FAILED` 不发布、不清错，让问题在调试期暴露）。

### 2.4 关节空间运动已被否决
显式用关节空间（`set_servo_angle` / `move_joint`）虽能确定性指定 J6 圈数，但**关节运动的 TCP 轨迹
不可预测、有撞机风险，已否决**。全程只用笛卡尔 `move_line`。

### 2.5 URDF 名义运动学 ≠ 固件出厂标定（~4.4mm / 1.2°）
- **现象**：verify 巡检对标过的凸起也偏 4~5mm，且**配置完全自洽**（centers==grid 0mm、pose 还原
  触点 ~1e-13）、代码无错。
- **根因**：同一关节角下 **ROS/URDF 正运动学 ≠ xArm 固件出厂逐台标定运动学**。本机实测固件
  `/xarm/xarm_states.pose` 与 ROS TF `link_base→link_tcp` 差 **~4.4mm XY + ~1.2°**。标定用 ROS TF
  记录、执行用 `move_line`（固件系）→ 记录源≠执行源，偏差进链路。
- **做法**：`calibrate_board.py` 已加 `~use_firmware_pose`（默认 true）：订阅 `/xarm/xarm_states`、
  用固件 TCP 记录触点。改此项后**必须重跑步骤 3 + 步骤 7**。下游无需改代码（controller 纯执行器；
  path_planner 放置走常量 `BOARD_POSE_BASE`、抓取 XY 被单应性覆盖）。

---

## 3. 标定坑

### 3.1 点击窗口被缩放会污染单应性（WINDOW_NORMAL）
标定/验证的手动点选 UI 用 `cv2.WINDOW_NORMAL`；当图像（1280×720）大于屏幕、窗口被缩小显示时，
OpenCV 鼠标回调返回的是**显示坐标**而非图像坐标。
- **为什么骗过所有验证**：`findHomography` 把这个全局缩放吸收进 `H`，步骤7 自检残差、`pixel_touch_check`
  都显示 RMS 低（点选 UI 同源缩放、误差互相抵消）。但**运行时视觉节点用程序检测的正确像素**，被污染的
  `H` 会产生**系统性、随位置变化的吸偏**——这是「有时吸不准/吸偏中心」的一大主因。
- **做法**：两处回调已改用 `cv2.getWindowImageRect` 把显示坐标还原成图像坐标；**必须重跑步骤7 重标
  `PICK_HOMOGRAPHY`**（旧值已污染）。验证：步骤7 打印的 (x,y) 应与 realsense-viewer 同点像素一致。

### 3.2 手眼 3D 投影在板面偏 1~2cm，平面单应性才准
用 eye-on-hand 手眼把已知 board 格心 3D 投影回图像，红点偏离凸起最大 **1~2cm**；改用 4 角点击拟合的
**平面单应性**后红点精准落在凸起上。
- **结论**：手眼这条路在板面是 10~20mm 量级误差（是抓取单应性 ~2.2mm 的 5~10 倍）。任何走手眼变换
  的方法（含「像素→射线→手眼×FK→z=0 平面求交」的无深度法）都继承这 1~2cm，去掉深度也救不回。
- **做法**：**不要**建议用手眼 3D 投影 / 射线-平面法替代单应性做 pixel→robot。抓取侧、放置侧都以单应性
  为准。详见 [calibrate.md](calibrate.md)。

### 3.3 TCP 侧向残差被 ~180° 腕部翻转放大
- **现象**：新臂标定后 verify 每格**恒定**偏几 mm、同方向；config 完美、`move_line` 精确复现、板未动。
- **根因**：UF Studio 的 **TCP 侧向(X/Y)有 ~1.6mm 标定残差**。verify 工具 yaw≈180°，比步骤3 触碰
  姿态(yaw≈0)翻了约 180°，残差 `e` 随腕部旋转 → 尖端偏 ~2·e。肉眼看 J6 轴心分辨不到 ~1mm，
  但 `move_line` 抓得到。生产抓/放 yaw 随任务变（`way` 的 0/90/180/270° + flip）→ 偏移**方向乱变**，
  无法用常量补，必须把 TCP 修准。
- **做法（比肉眼灵敏）**：手动触凸起(yaw=0)读 pose → `move_line` 到【同位置 + yaw 翻 180°】→ 量尖端
  位移 `d` → 解 `d=(R_verify−R_touch)·e_tool` 得侧向残差 → 加进 UF Studio TCP，重测收敛（180° 比
  90° 灵敏 2×）。改 TCP 后**必须**：① 同步改各 launch 的 `link6→link_tcp` 静态 TF；② 重跑步骤 3+7。
  注意 `link_eef`=法兰 ≠ `link_tcp`=吸嘴尖（差 68mm），别混。

### 3.4 抓取/放置下扎的 roll/pitch 要匹配初始位姿倾角，别写死 (π,0)
抓取单应性在**初始/观测位姿**采点，记录的是该姿态下吸嘴尖的 board-XY，含与工具朝向相关的 TCP 投影残差。
执行若用 board 系 (π,0) 而标定姿态≠(π,0)，残差不抵消 → 恒定横向抓偏。
- **现行机制**：`path_planner` 每次收到 plan 时**现读初始位姿**（`updateToolTiltFromInitialPose`）取
  roll/pitch，不再读存档 `PICK_TOOL_RPY_BOARD`（默认 `read_tool_tilt_from_initial_pose=true`，缺 TF
  回退 π,0）。前提：处理 plan 时臂确在初始位姿（生产单发满足；`test_controller` 手动喂 plan 故设 false）。
- **量级**：本机初始点工具轴离白板法向仅 **2.08°**，杠杆 67mm → 抓取横向**至多 ~2.44mm**，是次要项，
  别指望它根治吸偏。

### 3.5 改了 board 坐标系，必须重跑 pick_affine
`calibrate_board` 重定坐标系后，抓取单应性 `PICK_HOMOGRAPHY` 也失效，**必须重跑 `pick_affine`**
（`affine.launch`）。同理凡涉及固件 pose/TCP 的改动，都要重跑步骤 3 + 步骤 7。详见 [calibrate.md](calibrate.md)。

> ⚠️ **待办（clean cut）**：base 化重构后，旧标定失效，须在真机重跑 `calibrate_board` → `pick_affine`，
> 再用 `test_controller.launch`（限 1 任务）验证放置朝向——X 轴自动对齐尚未硬件验证。

---

## 4. 抓取高度 / 深度 / 视差

### 4.1 pick Z = 固定 `PICK_Z`；视差只改 X/Y
- 控制/路径里的视差补偿**只改 X/Y，从不动 Z**，不可能造成「抓取偏高」这种纯高度误差。
- 当前 `tetris_config.yaml` **没有** `PICK_SURFACE_PLANE_BASE`，pick Z 被钉死为常量 `PICK_Z`（≈0.01，
  board 系）+ 波纹管下压（`tcp_pick_offset_z`）。前提：散料**单层平铺**、不叠层。
- 「有些块偏高」是逐块厚度/坐落差异 + 单一 `PICK_Z` 盖不住，不是视差或手眼。

### 4.2 RealSense 逐块深度已关（±8mm 太不准）
`path_planner` 的 `use_depth_pick_z` **已于 2026-06-28 关闭**：RealSense 逐块深度实测有时差到 ~8mm，
比单一常量还糟；关掉后 path 节点连深度话题都不订阅。视觉节点 `lucky.launch` 里的 `use_depth_pick_z=true`
是 **debug-only**（只发 `/vision/pick_depth_debug`，从不写 `board_state`）。**别再建议靠深度修逐块高度。**

---

## 5. 坐标系与物理叠放

### 5.1 白板叠在发光板上；占用/place-Z 基准用逐格凸起顶面
- **物理叠放**：**白板（14×10 放置网格，带 140 个 2~3mm 凸起）叠在发光板（背光板，抓取/散料源区）上面**，
  放置面比抓取面高出一个白板厚。
- **z=0 在发光板**：标定步骤 2 拟的是发光板平面，`board_frame` 的 z=0 在发光板上；白板表面/凸起在
  board 系 z 为正（≈白板厚）。
- **判定「某格是否摆了方块」/放置高度**必须以**逐格白板凸起顶面**（`BOARD_BUMP_HEIGHT_MAP_14x10` /
  `PLACE_Z_MAP_14x10`）为基准，**不能**用发光板平面 `BOARD_SURFACE_Z`（否则空白板已高出一个厚度、
  每格都会误判为「有块」）。详见 [calibrate.md](calibrate.md)。

### 5.2 base 原生；`table_frame` 已删
流水线已全部 base 原生：旧 `table_frame` TF 与 `table_tf_broadcaster.py` 已删，坐标系由白板网格派生、
存为常量 `BOARD_POSE_BASE`。控制/路径节点把它当常量 `tf2::Transform` 加载，唯一动态 TF 是 `camera→base`。
视觉的 `table_frame` 参数只用于 debug（默认 `link_base`），`board_state` 从不依赖它。

> `way`/放置朝向约定脆弱：放置 yaw = `−way*90°`；`calibrate_board` 会把坐标系 X 轴贴合到最接近旧系的
> 网格轴以保约定。搞错这个符号/轴 → 放置朝向翻 90°/180°、方块错位。

### 5.3 方块是彩色的
放置/抓取的多联骨牌是**彩色**的，不是黑色（`CLAUDE.md` 及部分旧描述里的「黑色」不准确）。写文档/注释
一律用「彩色」。

---

## 6. 代码结构（改动前必查）

- **只有部分 `.cpp` 被编译**：按 `CMakeLists.txt`，只有 `strategy_node`、`xarm_controller_node`、
  `vision_processor_node_cpp`、`vision_processor_node_dexined`、`path_planner_node` 会被构建。
  `src/vision_processor_node_cuda.cpp`、`src/xarm_controller_node_pliz.cpp` **不在构建里**（参考用旧变体）。
  改它们对运行无影响——**认定某个 `.cpp` 是活的之前，先查 `CMakeLists.txt`**。
- **`scripts/vision_processor_node.py`** 是同契约的 Python 重实现，仅供 `affine.launch` 标定使用，
  不进生产（`lucky.launch` 用 C++ 节点）。
- **两套相机曝光/色彩档要配对**：DexiNed 用 `config/camera_config.yaml`、经典 `_cpp` 用
  `config/camer_config_1.yaml`；launch 硬编码加载前者，切换视觉实现时相机档要一并换。详见
  [vision.md](vision.md) §5.0。
- **配置文件当数据、别手改**：`config/tetris_config.yaml` 由标定工具生成/覆盖；同名 `.bak_before_*`
  是时点备份，供参考/回滚。

---

> 更底层的机制与最新架构以 `CLAUDE.md`、[path_plan.md](path_plan.md)、[control.md](control.md)、
> [calibrate.md](calibrate.md)、[vision.md](vision.md) 为准；本文只做「注意事项 + 踩坑」的集中索引。
