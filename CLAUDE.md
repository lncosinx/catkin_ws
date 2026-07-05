# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在本仓库中工作时提供指引。

## Claude 交互与语言约定（最高优先级）

针对 Claude 内部 Agent 在处理多语言上下文时可能产生的“跨语言幻觉漂移” Bug，特制定以下硬性交互约束：

1. 回复语言限制：Claude 在与用户交互、生成命令行输出、输出解释或撰写日志时，必须且只能使用简体中文（Simplified Chinese）或英文（English）。
2. 绝对禁用语言：在任何情况下，严禁使用韩文（Korean）或日文（Japanese）进行回复。如果检测到自身输出中包含韩文字符，请立即擦除并重写。
3. 代码与注释规范：代码本身（变量、类名、函数名）必须使用英文；新增或修改的代码注释必须与项目既有风格保持一致（即使用简体中文）。

## 项目概览

ROS 1 (Noetic) catkin 工作区，用于一台 xArm6 机械臂：用 RealSense 相机识别
背光板上的黑色多联骨牌（"俄罗斯方块"），再用真空吸盘把它们抓取并放入固定的
14x10 网格，目标是放进尽可能多的方块（一个精确覆盖装箱问题）。

物理叠放关系（容易踩坑，务必记清）：**白板（14x10 放置网格，带 140 个 2~3mm 凸起）
叠在发光板（背光板，抓取/散料源区）上面**，所以放置面比抓取面高出一个白板厚。
标定里 `board_frame` 的 z=0 平面在**发光板**上（步骤2拟的是发光板平面，取法向+z=0
基准，不是白板），白板表面/凸起在 board 系 z 为正（约等于白板厚）。判定"白板某格上
是否摆了方块"必须以**逐格白板凸起顶面**（`BOARD_BUMP_HEIGHT_MAP_14x10`，步骤3采）
为基准，不能用发光板平面 `BOARD_SURFACE_Z`。

所有活跃开发都在 `src/tly`。`src/` 下其余内容（`xarm_ros`、`realsense-ros`、
`vision_opencv`、`easy_handeye`）都是引入的第三方依赖——`.gitignore` 已把
`src/xarm_ros`、`src/realsense-ros`、`src/easy_handeye`、`OpenCV_Source/`、
`build/`、`devel/` 排除在本仓库历史之外。一般不需要改它们，当作已安装的包对待。
根目录的 `plan.md` / `test_schedule.md` 是当前维护的规划/验证文档；旧的 `docs/`
已过时，别依赖。

## 构建 / 运行

```bash
source /opt/ros/noetic/setup.bash
catkin_make            # 在 /root/catkin_ws 下执行 —— 唯一在用的构建流程
source devel/setup.bash
```

`src/tly/CMakeLists.txt` 全局强制 `-std=c++14` 和 `-O3`——`-O3` 是专门为
`strategy_node` 的 DLX 搜索（计算密集）加的，改构建配置时别去掉。`tly` 本身没有
lint/测试套件。

对真机跑完整流水线：

```bash
roslaunch tly tly.launch robot_ip:=192.168.1.228
```

按节点划分的 `test_*.launch`（只起某个节点需要的东西，省得跑整个 `tly.launch`
流水线）。所有带视觉的都可用 `vision_node:=vision_processor_node_dexined` 切换
视觉实现。
- `test_vision.launch` —— **仅视觉**：相机 + `image_proc` + 视觉节点。无臂/TF/
  策略。查看 `/vision/board_state`、`/vision/debug_image`、`/vision/pick_depth_debug`。
- `test_strategy.launch` —— **仅策略**，无硬件。发一个假的 `/vision/board_state`
  驱动它；看 `/tetris_plan`。
- `test_path.launch` —— **感知→策略→路径**链路。需要臂提供 TF（`robot_ip:=`），
  但绝不命令运动。发布 `/motion_cmds`；对比 `[PATH][TASK]` 的 `z_plane`/`z_depth` 日志。
- `test_controller.launch` —— **仅控制器** + 臂 + 相机。手动喂 `/tetris_plan`；
  **会驱动真机**（限 1 个任务）。原生驱动，非 MoveIt。
- `test_pick.launch` —— 较旧的基于 MoveIt 的集成测试：视觉 + `xarm_controller_node`
  + `single_block_test.py`，一次抓放循环。

`src/tly/launch/` 下其它 launch：
- `tly.launch` —— 生产运行。仅用 xArm 原生驱动（明确*不用* MoveIt/Pilz）。端到端
  起相机、手眼 TF、视觉、控制器、策略、`path_planner_node`。
- `affine.launch` / `calibrate_tool.launch` / `calibrate_xarm.launch` /
  `xarm_calibration_setup.launch` —— 标定工具，见下文。

## 节点流水线 (src/tly)

三个自定义节点通过固定的话题/服务契约通信：

**1. `vision_processor_node` / `vision_processor_node_cpp`**
（由 `src/vision_processor_node.cpp` 编译）
- 订阅校正后的彩色图（`image_proc` 输出）与相机内参；在亮板上分割暗色方块
  （Otsu/手动阈值 + 发光板掩膜），提取轮廓，并按步进旋转的 7 个多联骨牌模板
  逐一分类（`template_angle_step` / `template_refine_step`）。
- 可选地用在 CUDA 上跑的 DexiNed ONNX 模型细化方块边缘（`module/dexined.onnx`，
  由 `use_dexined` 开关）——这是替代旧经典边缘检测的神经网络边缘检测器。
- 跨帧跟踪检测（`track_history_len`、`stable_min_frames`、`stable_max_px_std`…），
  方块稳定后才发布。
- 发布：
  - `/vision/board_state`（`std_msgs/Int32MultiArray`）：7 个形状库存 + 140 个
    棋盘占用栅格（`BOARD_ROWS` x `BOARD_COLS` = 14x10）+ `num_blocks` + 每个检测
    方块一个扁平的 `[shape, u, v, angle]` 元组。`strategy_node` 要求
    `data.size() >= 147`（7+140）后再读其余部分。
  - `/vision/tracked_blocks_table`（`geometry_msgs/PoseArray`）以及 `/vision/debug_*`
    和 `/vision/preprocess/*` 下的调试图话题。
  - 服务 `/vision/get_precise_pose`（`tly/GetPrecisePose`）：给定 `target_shape_type`，
    返回细化的 `dx`/`dy`/`angle`。注意：本仓库当前没有节点调用此服务——它仅供
    未来/手动使用。

**2. `strategy_node`**（`src/strategy_node.cpp`）
- 等 `/vision/board_state`，直到库存在 `expected_total_blocks`（默认 35）稳定
  `inventory_stable_required_frames` 帧；若稳定帧数不够，有兜底接受
  `min_usable_total_blocks`（默认 34-35）。
- 把放置当作精确覆盖问题求解，用手写的 Dancing Links（DLX）引擎在
  `BASE_SHAPES`（7 个类俄罗斯方块）及其 4 种旋转上，带重启地搜索最高分覆盖
  （`max_search_nodes` / `max_restarts`）。
- 在 `/tetris_plan`（`std_msgs/Int32MultiArray`，latched）上发布一次方案。

**3. `xarm_controller_node`**（`src/xarm_controller_node.cpp`）
- 仅通过 xArm 的*原生*服务驱动臂——`/xarm/set_mode`、`/xarm/set_state`、
  `/xarm/move_line`，加上吸盘的数字 IO 服务（`suction_io_num`）——绝不用 MoveIt。
- 消费 `/tetris_plan` 一次，跑逐任务状态机（`IDLE` → `TAKE_NEXT_TASK` →
  `MOVE_TO_PICK_HOVER` → `EXECUTE_PICK` → `MOVE_TO_PLACE_HOVER` → `EXECUTE_PLACE`
  → `FINISH`），上限 `max_tasks_per_plan`。
- 用 `tetris_config.yaml` 里的标定数据（抓取面平面、棋盘映射、抓取单应性）结合
  easy_handeye 的 eye-on-hand TF，把像素检测转成机器人 base 坐标。整条流水线是
  **base 原生**的：它把 `BOARD_POSE_BASE`（board_frame 在 base 系的位姿）作为常量
  `tf2::Transform` 加载，所有抓放计算都在 `link_base` 系完成；唯一需要的动态 TF 是
  `camera->base`。已无 `table_frame` TF。
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
**同步**翻转后落点不变）。控制节点 `buildTask` 对每个任务在 A（不翻）/B（翻 180°）两套
方案里选择，目标是**腕部转角最小**——因为 xArm 各轴**同时到达**，多转的 yaw 会成为
整段运动的速度瓶颈（例：只需 +1° 却反向转 179° 就会拖慢全程）；J6 限位仅作**约束**
（`wrist_soft_limit_rad`，默认 ≈315°，硬限 ±2π），越软限位的方案才被排除。注意这里
**不是**"把 J6 往 0 居中"——居中虽不撞限位但会制造大量多余转角。显式用关节空间
（`set_servo_angle`/`move_joint`）控制 J6 虽能确定性指定圈数，但**关节运动的 TCP 轨迹
不可预测、有撞机风险，已否决**。

坐标系：`board_frame` 完全由白板网格标定派生（原点=网格原点，X≈行轴，Z=板面法向），
以常量 `BOARD_POSE_BASE`（`{origin, rpy}`）存于 `tetris_config.yaml`。旧的
`table_frame` TF 与 `table_tf_broadcaster.py` 已删除（base 化重构）——
`path_planner_node` 和 `pick_affine_calibration_tool.py` 同样加载 `BOARD_POSE_BASE`；
视觉的 `table_frame` 参数现在只用于 debug（默认 `link_base`），`board_state` 从不
依赖它。

## 标定数据与形状契约

`config/tetris_config.yaml`（`tetris:` 键下的全部）保存所有标定状态：手眼坐标系、
`BOARD_POSE_BASE`（board_frame 在 base 系的位姿——原点=网格原点，Z=白板平面法向
`BOARD_SURFACE_NORMAL_BASE`，因为 base-Z 并不垂直于桌面）、14x10 棋盘网格的
采样/原点/步长（board 局部坐标：`BOARD_SAMPLES_BOARD`、`BOARD_ORIGIN_BOARD`、
`BOARD_CENTERS_14x10_BOARD`…）、抓取单应性（`PICK_HOMOGRAPHY`，`pixel -> board-XY`）、
以及一个棋盘"凸起高度"模型（`BOARD_BUMP_HEIGHT_MODEL`、`USE_BUMP_HEIGHT_FOR_PLACE`），
用于在方块坐落高度略有不同时修正 2.5D 高度视差。此文件由下述标定工具生成/覆盖——
当作数据，别手改。同名的 `.bak_before_*` 是过去标定运行的时点备份，留作参考/回滚。

形状 ID 是 `strategy_node.cpp`、`vision_processor_node.cpp`/`.py`、
`single_block_test.py` 共享的契约——见 `BASE_SHAPES`：`0` 一字、`1` 方块、`2` T、
`3` L_left、`4` L_right、`5` Z_left、`6` Z_right。改这个枚举需要同步更新所有这些。

标定流程：
- `xarm_calibration_setup.launch` + `calibrate_xarm.launch` —— 通过 `easy_handeye`
  做 ArUco 标记的 eye-on-hand 标定，产出 `xarm6_realsense_calibration_eye_on_hand`
  数据，供 `tly.launch` 里的 `easy_handeye/publish.launch` 使用。
- `calibrate_tool.launch` → `scripts/calibrate_board.py` —— 交互式白板标定
  （6 步；可用 `~steps` 只跑子集）。步骤 3 在 base 系采棋盘网格并**由网格派生
  `board_frame`**（不再手动选原点/X），使放置朝向自动跟随棋盘网格；产出
  `BOARD_POSE_BASE`、board 局部几何、逐格 place-Z 图、凸起高度，以及可选的波纹管 Z
  补偿（步骤 6 → `TCP_PICK_OFFSET_Z`/`TCP_PLACE_OFFSET_Z`，由控制/路径节点自动读取）。
  X 轴方向会贴合到现有坐标系以保持策略的 `way` 约定；**改坐标系后必须一并重跑
  `pick_affine`。** 抓取 Z 来自 RealSense 深度，而非平面拟合。
- `affine.launch` → `scripts/pick_affine_calibration_tool.py`（配合 Python 版
  `vision_processor_node.py`）—— 标定抓取侧单应性（`pixel -> board-XY`）。记录
  `base<-eef` 并经加载的 `BOARD_POSE_BASE` 换算；每当 `calibrate_board` 重定坐标系
  都必须重跑。
- `scripts/hsv_tuner.py`、`scripts/test_hsv.py`、`scripts/test_aruco.py` ——
  独立的手动调试工具，未接入任何 launch。

## 注意事项

- 按 `CMakeLists.txt`，只有 `strategy_node`、`xarm_controller_node` 和
  `vision_processor_node_cpp`（由 `src/vision_processor_node.cpp` 编译）会被编译。
  `src/vision_processor_node_cuda.cpp` 和 `src/xarm_controller_node_pliz.cpp`
  **不在**构建里——是留作参考的旧变体。改它们对运行无影响；在认定某个 `.cpp`
  是活的之前，先查 `CMakeLists.txt`。
- `scripts/vision_processor_node.py` 是同一节点/话题/服务契约的 Python 重实现
  （与 `strategy_node.cpp` 保持形状 ID 兼容，见其模块 docstring），仅供
  `affine.launch` 使用。生产（`tly.launch`）用 C++ 节点。
- **启动 MoveIt/realMove_exec 前，±360° 关节（J1/J4/J6）必须远离 ±360° 边界**
  （**重要踩坑**）：凡包含 `xarm6_moveit_config/realMove_exec.launch` 的 launch
  （如 `calibrate_tool.launch`、旧的 `test_pick.launch`），在打印
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
