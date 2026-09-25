# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## Claude 交互与语言约定（最高优先级）

针对 Claude 内部 Agent 在处理多语言上下文时可能产生的“跨语言幻觉漂移” Bug，特制定以下硬性交互约束：

1. 回复语言限制：Claude 在与用户交互、生成命令行输出、输出解释或撰写日志时，必须且只能使用简体中文（Simplified Chinese）或英文（English）。
2. 绝对禁用语言：在任何情况下，严禁使用韩文（Korean）或日文（Japanese）进行回复。如果检测到自身输出中包含韩文字符，请立即擦除并重写。
3. 代码与注释规范：代码本身（变量、类名、函数名）必须使用英文；新增或修改的代码注释必须与项目既有风格保持一致（即使用简体中文）。

## 项目概览

ROS 1 (Noetic) catkin 工作区，用于一台 xArm6 机械臂：用 RealSense 相机识别
背光板上的彩色俄罗斯方块，再用真空吸盘把它们抓取并放入固定的
14x10 网格，目标是放进尽可能多的方块（一个精确覆盖装箱问题）。

物理叠放关系（容易踩坑，务必记清）：**白板（14x10 放置网格，带 140 个 2~3mm 凸起）
叠在发光板（背光板，抓取/散料源区）上面**，所以放置面比抓取面高出一个白板厚。
标定里 `board_frame` 的 z=0 平面在**发光板**上（步骤2拟的是发光板平面，取法向+z=0
基准，不是白板），白板表面/凸起在 board 系 z 为正（约等于白板厚）。判定"白板某格上
是否摆了方块"必须以**逐格白板凸起顶面**（`BOARD_BUMP_HEIGHT_MAP_14x10`，步骤3采）
为基准，不能用发光板平面 `BOARD_SURFACE_Z`。

所有活跃开发都在 [src/lucky](src/lucky)。`src/` 下其余内容（`xarm_ros`、`realsense-ros`、
`vision_opencv`、`easy_handeye`）都是引入的第三方依赖——[.gitignore](.gitignore) 已把
它们以及 `OpenCV_Source/`、`build/`、`devel/` 排除在本仓库历史之外。一般不需要改它们，
当作已安装的包对待。根目录的 [docs/](docs/) 为各个部分的介绍（入口 [docs/readme.md](docs/readme.md)）。


## 构建 / 运行

```bash
source /opt/ros/noetic/setup.bash
catkin_make            # 在 /root/catkin_ws 下执行 —— 唯一在用的构建流程
source devel/setup.bash
```

[src/lucky/CMakeLists.txt](src/lucky/CMakeLists.txt) 全局强制 `-std=c++14` 和 `-O3`——`-O3` 是专门为
`strategy_node` 的 DLX 搜索（计算密集）加的，改构建配置时别去掉。`lucky` 本身没有
lint/测试套件（[src/lucky/test/test_tetris_solver.cpp](src/lucky/test/test_tetris_solver.cpp) 未接入 CMake，需手动编译）。

对真机跑完整流水线：

```bash
roslaunch lucky lucky.launch robot_ip:=192.168.1.216   # launch 默认 IP
rostopic pub -1 /ready std_msgs/Bool "{data: true}"    # 控制节点默认 require_ready=true，需放行
```

按节点划分的 `test_*.launch`（只起某个节点需要的东西，省得跑整个
[lucky.launch](src/lucky/launch/lucky.launch) 流水线）。带视觉的默认用 DexiNed 版，可用
`vision_node:=vision_processor_node_cpp` 切到经典实现。
- [test_vision.launch](src/lucky/launch/test_vision.launch) —— **仅视觉**：相机 + 视觉节点。无臂/TF/
  策略。查看 `/vision/board_state`、`/vision/debug_image`、`/vision/pick_depth_debug`。
- [test_strategy.launch](src/lucky/launch/test_strategy.launch) —— **仅策略**，无硬件。发一个假的
  `/vision/board_state` 驱动它；看 `/tetris_plan` 与 `/tetris_plan_candidates`。
- [test_path.launch](src/lucky/launch/test_path.launch) —— **感知→策略→路径**链路。需要臂提供 TF 与
  `joint_states`（`robot_ip:=`），但没有控制节点、绝不命令运动。发布 `/motion_cmds`；看 `[PATH]` 日志。
- [test_controller.launch](src/lucky/launch/test_controller.launch) —— **path_planner + 控制器** + 臂 + 相机；
  手动喂 `/tetris_plan`，经 path_planner 解算成 `/motion_cmds`。**会驱动真机**（限 1 个任务）；
  控制节点默认 `require_ready=true`，还需发 `/ready`。原生驱动，非 MoveIt。
- [test_pick.launch](src/lucky/launch/test_pick.launch) —— 旧的基于 MoveIt 的单块集成测试（视觉 +
  `xarm_controller_node` + [single_block_test.py](src/lucky/scripts/single_block_test.py)）。**已与现架构脱节、不可用**：
  它不起 `path_planner_node`，而控制节点只消费 `/motion_cmds`，[single_block_test.py](src/lucky/scripts/single_block_test.py) 发的
  `/tetris_plan` 无人解算；其中的 MoveIt/速度比例参数控制节点也已不读。

[src/lucky/launch/](src/lucky/launch/) 下其它 launch：
- [lucky.launch](src/lucky/launch/lucky.launch) —— 生产运行。仅用 xArm 原生驱动（明确*不用* MoveIt/Pilz）。
  端到端起相机、手眼 TF、视觉、策略、`path_planner_node`、控制器。
- 标定/诊断工具（见下文与 [docs/calibrate.md](docs/calibrate.md)）：
  [xarm_calibration_setup.launch](src/lucky/launch/xarm_calibration_setup.launch)、
  [xarm_calibration_setup_moveit.launch](src/lucky/launch/xarm_calibration_setup_moveit.launch)、
  [calibrate_xarm.launch](src/lucky/launch/calibrate_xarm.launch)、
  [calibrate_tool.launch](src/lucky/launch/calibrate_tool.launch)、
  [verify_camera.launch](src/lucky/launch/verify_camera.launch)、
  [pixel_touch_check.launch](src/lucky/launch/pixel_touch_check.launch)、
  [measure_tcp_omega.launch](src/lucky/launch/measure_tcp_omega.launch)。

## 节点流水线 (src/lucky)

四个自定义节点通过固定的话题契约串联：
`/vision/board_state` → `/tetris_plan` + `/tetris_plan_candidates` → `/motion_cmds` → 真机。

**1. 视觉：`vision_processor_node_dexined` / `vision_processor_node_cpp`**
（分别由 [vision_processor_node_dexined.cpp](src/lucky/src/vision_processor_node_dexined.cpp) 与
[vision_processor_node.cpp](src/lucky/src/vision_processor_node.cpp) 编译；两者话题/消息契约一致，
launch 用 `vision_node:=` 选择，生产默认 DexiNed 版）
- 订阅彩色图与相机内参；在亮板上分割方块（Otsu/手动阈值 + HSV 饱和度彩色前景 + 发光板
  掩膜），提取轮廓，并按步进旋转的 7 个多联骨牌模板逐一分类（`template_angle_step` /
  `template_refine_step`）。
- DexiNed 版另用 ONNX 边缘检测网络（[module/dexined.onnx](src/lucky/module/dexined.onnx)，强制 CUDA 后端，失败回退经典
  流程）+ Lab 颜色边界切割分开贴碰块，并有输入帧 EMA（`temporal_alpha`）与同帧 NMS。
- 跨帧跟踪检测（`track_history_len`、`stable_min_frames`、`stable_max_px_std`…），
  方块稳定后才发布。
- 发布：
  - `/vision/board_state`（`std_msgs/Int32MultiArray`）：7 个形状库存 + 140 个
    棋盘占用栅格（`BOARD_ROWS` x `BOARD_COLS` = 14x10，视觉端置 0）+ `num_blocks` + 每个检测
    方块一个扁平的 `[shape, pick_u, pick_v, angle, geom_u, geom_v]` 六元组。`strategy_node`
    要求 `data.size() >= 147`（7+140）后再读其余部分。
  - `/vision/tracked_blocks_table`（`geometry_msgs/PoseArray`）、`/vision/pick_depth_debug`
    （`Float32MultiArray`），以及 `/vision/debug_*` 和 `/vision/preprocess/*` 下的调试图话题。
  - 不提供任何服务。

**2. `strategy_node`**（[strategy_node.cpp](src/lucky/src/strategy_node.cpp)，求解引擎在
[tetris_solver.hpp](src/lucky/include/lucky/tetris_solver.hpp)）
- 等 `/vision/board_state`，直到库存在 `expected_total_blocks`（默认 35）稳定
  `inventory_stable_required_frames` 帧；若只差 1 个，有兜底接受
  `min_usable_total_blocks`（代码默认 34，[lucky.launch](src/lucky/launch/lucky.launch) 设 35）。
- 把放置当作精确覆盖问题求解，用手写的 Dancing Links（DLX）引擎在
  `BASE_SHAPES`（7 个类俄罗斯方块）及其 4 种旋转上，带重启地搜索最高分覆盖，并收集
  最多 `num_strategy_candidates`（默认 8）套同分布局。进阶模式（`advanced_mode`）改用
  束搜索按 `shape_sequence` 求解。
- 发布（latched）：`/tetris_plan` 为首选方案（17-int/任务），`/tetris_plan_candidates` 为
  全部同分候选 `[K, len0, plan0, len1, plan1, ...]`。

**3. `path_planner_node`**（[path_planner_node.cpp](src/lucky/src/path_planner_node.cpp)，运动学在
[xarm6_kinematics.hpp](src/lucky/include/lucky/xarm6_kinematics.hpp)）
- 消费 `/tetris_plan_candidates`（`use_plan_candidates=true`，否则 `/tetris_plan`）与
  `/vision/board_state`（同形状抓取候选池），加载 `/tetris/*` 标定，把像素抓取点（单应性 XY +
  抓取平面 Z + 高度视差补偿）与目标格（格心双线性 + 逐格最高放置 Z）全部解算成 **base 系**位姿。
- 读 `/xarm/joint_states`，用 xArm6 FK/IK 沿 `move_line` 直线路径积分关节时间代价，联合优化
  放置顺序（DAG 内，`allow_reorder`）× 同形状抓取分配 × 腕部 180° 翻转，并做 J6 限位约束与
  "保余量"势垒 + 自动升挡重解；多候选时逐个评估取总代价最小者。
- 发布 `/motion_cmds`（`lucky/MotionPlan`，见 [MotionPlan.msg](src/lucky/msg/MotionPlan.msg) /
  [MotionTask.msg](src/lucky/msg/MotionTask.msg)，每任务 4 个 base 位姿）与调试用 `/tetris_plan_opt`。

**4. `xarm_controller_node`**（[xarm_controller_node.cpp](src/lucky/src/xarm_controller_node.cpp)）
- **纯执行器**：只消费 `/motion_cmds`，**不加载任何标定、不做坐标换算或腕部决策**。
- 仅通过 xArm 的*原生*服务驱动臂——`/xarm/set_mode`、`/xarm/set_state`、
  `/xarm/move_line`，加上吸盘的数字 IO 服务 `/xarm/set_controller_dout`（`suction_io_num`）——绝不用 MoveIt。
- 逐任务状态机：`IDLE` → `WAIT_FOR_READY`（`require_ready=true` 时等 `/ready` 发 `true`）→
  `TAKE_NEXT_TASK` → `MOVE_TO_PICK_HOVER` → `EXECUTE_PICK` → `MOVE_TO_PLACE_HOVER` →
  `EXECUTE_PLACE` → … → `FINISH`，上限 `max_tasks_per_plan`（代码默认 1，[lucky.launch](src/lucky/launch/lucky.launch) 设 34）。
- 发布 `/robot_status`（`std_msgs/Bool`，latched）作为忙/闲标志。

腕部 yaw / J6 处理（**重要踩坑，务必记清**）：xArm 笛卡尔接口（`move_line`/原生
`set_position`，以及 MoveIt Pilz LIN 同理）把命令的 RPY 转成旋转矩阵 `R` 做 IK，
而 `R(yaw) ≡ R(yaw±2π)`——**"圈数 / winding" 无法经笛卡尔指令表达**，固件按
**"就近解"**（从当前关节最近的 IK 分支）自行决定 J6 落哪一圈。实测佐证：在 UF Studio
把 yaw 从 -73° 命令到 287°（差正好 360°），机械臂纹丝不动。两条推论：(1) 任何想靠
命令带绕圈的 `+2π·k` 来躲 J6 限位的做法都是**无效**的（历史上 `path_planner` 里的
J6 绕圈 DP `assignYaws` 因此被撤除）；(2) **一旦 J6 越过约 ±180° 就再也无法用笛卡尔
指令拉回**（就近解会朝更远那一圈走），所以策略只能"预防"不能"补救"。能动的唯一自由度
是 **180° 翻转**（`flip`，真正改变了 `R`，固件会照做；吸盘对 180° 对称，pick/place
**同步**翻转后落点不变）。翻转决策现在由 **`path_planner_node`** 负责（`evalPairJoint` 对每个
（放置槽, 抓取块）在 A 不翻 / B 翻 180° 间择优）：代价是沿 `move_line` 路径积分的**关节时间**
（xArm 各轴**同时到达**，多转的 yaw 会成为整段运动的速度瓶颈），J6 限位作约束——
`wrist_soft_limit_rad`（代码默认 4.7≈269°，[lucky.launch](src/lucky/launch/lucky.launch) 设 3.7）越界秒尺度加罚、
`wrist_hard_limit_rad`（代码默认 6.10，[lucky.launch](src/lucky/launch/lucky.launch) 设 6.14）越界直接排除；另有
`wrist_center_*` 二次势垒让 |J6| 超出约 ±180° 时加罚、保住头寸以放满全部方块。注意这里
**不是**"把 J6 往 0 居中"——留白带内不加势，居中会制造大量多余转角。控制节点的逐轴
`unwrapAngle` 只保证下发角与当前姿态连续，不改翻转选择。显式用关节空间
（`set_servo_angle`/`move_joint`）控制 J6 虽能确定性指定圈数，但**关节运动的 TCP 轨迹
不可预测、有撞机风险，已否决**。

坐标系：`board_frame` 完全由白板网格标定派生（原点=网格原点，X≈行轴，Z=板面法向），
以常量 `BOARD_POSE_BASE`（`{origin, rpy}`）存于 [tetris_config.yaml](src/lucky/config/tetris_config.yaml)。
旧的 `table_frame` TF 与 `table_tf_broadcaster.py` 已删除（base 化重构）——`path_planner_node`
把 `BOARD_POSE_BASE` 作为常量 `tf2::Transform` 加载，所有抓放计算都在 `link_base` 系完成，
唯一需要的动态 TF 是 `camera->base`；控制节点不需要任何 TF。视觉的 `table_frame` 参数现在只用于
debug（默认 `link_base`），`board_state` 从不依赖它。

## 标定数据与形状契约

[tetris_config.yaml](src/lucky/config/tetris_config.yaml)（`tetris:` 键下的全部）保存所有标定状态：
`BOARD_POSE_BASE`（board_frame 在 base 系的位姿——原点=网格原点，Z=板面法向
`BOARD_SURFACE_NORMAL_BASE`，因为 base-Z 并不垂直于桌面）、14x10 棋盘网格的
采样/原点/步长（board 局部坐标：`BOARD_SAMPLES_BOARD`、`BOARD_ORIGIN_BOARD`、
`BOARD_CENTERS_14x10_BOARD`…）、逐格放置高度 `PLACE_Z_MAP_14x10`、抓取平面
`PICK_SURFACE_PLANE_BASE`、抓取单应性（`PICK_HOMOGRAPHY`，`pixel -> board-XY`）、
波纹管补偿 `TCP_PICK_OFFSET_Z`/`TCP_PLACE_OFFSET_Z`、以及棋盘"凸起高度"模型
（`BOARD_BUMP_HEIGHT_MODEL` 等）。此文件由标定工具生成/覆盖——当作数据，别手改。
手眼结果不在此文件里，由 `easy_handeye` 存到其自身目录。

形状 ID 是 [strategy_node.cpp](src/lucky/src/strategy_node.cpp)（`BASE_SHAPES` 在
[tetris_solver.hpp](src/lucky/include/lucky/tetris_solver.hpp)）、两个 C++ 视觉节点、
[vision_processor_node.py](src/lucky/scripts/vision_processor_node.py)、
[single_block_test.py](src/lucky/scripts/single_block_test.py) 共享的契约：`0` 一字、`1` 方块（田）、
`2` T、`3` L_left、`4` L_right、`5` Z_left、`6` Z_right。改这个枚举需要同步更新所有这些。

标定流程（详见 [docs/calibrate.md](docs/calibrate.md)）：
- **末端 TCP 标定（第一步）**：吸嘴恒竖直朝下、只有 yaw 旋转，故只标吸嘴尖相对法兰轴的 XY 偏心
  （Z 不可观测且前后抵消，沿用 68mm）。UF Studio 手动 180° 对点法算修正量，写入 UF Studio TCP 与各 launch 的
  `link6→link_tcp` 静态 TF（两处须一致）。**TCP 一变，手眼 + 白板全 7 步都要重跑。**
- [xarm_calibration_setup.launch](src/lucky/launch/xarm_calibration_setup.launch)（原生驱动，手动 freehand）或
  [xarm_calibration_setup_moveit.launch](src/lucky/launch/xarm_calibration_setup_moveit.launch)（MoveIt 自动采样）
  → [calibrate_xarm.launch](src/lucky/launch/calibrate_xarm.launch) —— 以 ChArUco 板
  （[charuco_tracker.py](src/lucky/scripts/charuco_tracker.py) 检测）做 eye-on-hand 手眼标定，产出
  `xarm6_realsense_calibration_eye_on_hand`，供 `easy_handeye/publish.launch` 运行时广播。
- [calibrate_tool.launch](src/lucky/launch/calibrate_tool.launch) → [calibrate_board.py](src/lucky/scripts/calibrate_board.py)
  —— 交互式白板标定，**7 步**（可用 `~steps` 私有参数只跑子集，launch 未暴露该 arg，默认交互选择）：
  1 拍照起点位姿；2 深度拟合发光板平面（Z 轴 + z=0 基准）；3 在 base 系采棋盘网格并**由网格派生
  `board_frame`**（产出 `BOARD_POSE_BASE`、board 局部几何、凸起高度），X 轴贴合现有坐标系以保持策略的
  `way` 约定；4 方块厚度；5 逐格放置 Z；6 可选波纹管 Z 补偿（`TCP_PICK_OFFSET_Z`/`TCP_PLACE_OFFSET_Z`，
  由 `path_planner_node` 读取，但会被节点私有参数 `tcp_pick_offset_z`/`tcp_place_offset_z` 覆盖——
  [lucky.launch](src/lucky/launch/lucky.launch) 当前显式设了这两项）；7 抓取单应性 `PICK_HOMOGRAPHY`。**重跑步骤 3（改坐标系）后必须一并
  重跑步骤 7。** 另有专项模式 `edit_cells` / `verify_cells` / `depth_offset_check`。
- 诊断/验证：[verify_camera_intrinsics.py](src/lucky/scripts/verify_camera_intrinsics.py)（相机内参，
  [verify_camera.launch](src/lucky/launch/verify_camera.launch)）、
  [pixel_touch_check.py](src/lucky/scripts/pixel_touch_check.py)（抓偏来源诊断）、
  [measure_tcp_omega.py](src/lucky/scripts/measure_tcp_omega.py)（`omega_per_v_lin_rad_per_m` 测量）。

## 注意事项

- 按 [CMakeLists.txt](src/lucky/CMakeLists.txt)，编译 5 个可执行文件：`strategy_node`、`xarm_controller_node`、
  `vision_processor_node_cpp`、`vision_processor_node_dexined`、`path_planner_node`；安装的 Python 脚本只有
  [vision_processor_node.py](src/lucky/scripts/vision_processor_node.py) 与
  [single_block_test.py](src/lucky/scripts/single_block_test.py)（标定/诊断脚本直接以源码运行）。
- [vision_processor_node.py](src/lucky/scripts/vision_processor_node.py) 是同一节点/话题契约的 Python
  重实现（与 `strategy_node` 保持形状 ID 兼容），**当前没有任何 launch 使用它**；生产用 C++ 节点。
- 两个视觉节点各配一套 RealSense 相机档：DexiNed 用 [camera_config.yaml](src/lucky/config/camera_config.yaml)
  （lucky/test_vision launch 硬编码加载），经典 `_cpp` 用 [camera_config_1.yaml](src/lucky/config/camera_config_1.yaml)；
  切换视觉实现时相机档要一并换。
- **速度同步点**：控制节点的 `transit_speed_mm_s` / `loaded_transit_speed_mm_s` 必须与 path_planner 的
  `transit_lin_speed_m_s` / `loaded_lin_speed_m_s`（÷1000）一致，否则关节代价模型失真。
- **启动 MoveIt/realMove_exec 前，±360° 关节（J1/J4/J6）必须远离 ±360° 边界**
  （**重要踩坑**）：凡包含 `xarm6_moveit_config/realMove_exec.launch` 的 launch
  （[calibrate_tool.launch](src/lucky/launch/calibrate_tool.launch)、
  [pixel_touch_check.launch](src/lucky/launch/pixel_touch_check.launch)、
  [xarm_calibration_setup_moveit.launch](src/lucky/launch/xarm_calibration_setup_moveit.launch)、旧的
  [test_pick.launch](src/lucky/launch/test_pick.launch)），在打印
  `Started controllers: xarm6_traj_controller, joint_state_controller` 那一刻，
  MoveIt 会把臂从 UF Studio 位姿模式切到 **SERVO 关节伺服模式**
  （`xarm_driver.cpp` `set_mode(SERVO)+set_state(START)`）。若某个 ±2π 关节此时
  绕在接近 ±360° 处（实测 J4=-358.4°，距 -360° 硬限仅 1.6°），**固件会在切模式
  瞬间对这个绕到边界的关节做"就近解"重解算、朝规范角解缠**，触发一次未经 MoveIt
  规划的大幅摆动（实测 J4 甩约 180° 后触发保护停止 state 5，停在解缠行程中点
  ≈-178°）。这不是 ROS 层命令的（已核实 `xarm_hw` 初始 `position_cmds_=当前角`、
  SDK `set_servo_angle_j` 不归一化、URDF J4 限位 ±2π 不裁剪），而是固件行为，与上文
  腕部"圈数/就近解" lore 同源。**规避**：启动前在 UF Studio 把 J1/J4/J6 摇到中段
  （规范区间正中、远离 ±360°）再 launch；关节停在规范区间内启动则不跳。
