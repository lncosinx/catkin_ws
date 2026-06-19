# 重构计划 (plan.md)

xArm6 俄罗斯方块抓取项目的代码重构方案。目标是把现有「散落在
`vision_processor_node` / `strategy_node` / `xarm_controller_node` 三个大节点
里、职责互相渗透」的代码，按照 **标定 / 视觉 / 策略 / 路径 / 控制** 五大类
重新切分，并补齐进阶任务所需的能力。

> 说明：图像里方块本身是**有颜色**的（颜色与形状一一绑定）。当前通过调相机
> 参数把整幅画面压成黑白——白色是发光板、黑色是放在板上的方块，便于/root/catkin_ws/src/tly/src/xarm_controller_node.cpp分割。
> 但是还可以通过调整相机参数，将画面变回彩色，方便/root/catkin_ws/src/tly/src/vision_processor_node_dexined.cpp分割。
> 现有源码注释里「black blocks」的说法不准确，重构时一并订正。

---

## 0. 现状梳理（重构前）

| 现有文件 | 角色 | 重构后去向 |
|---|---|---|
| `src/vision_processor_node.cpp` | 当前是 **DexiNed + 模板匹配**（最新一版把边缘检测换成了神经网络） | 改回**经典轮廓+模板匹配**版，作为 `vision_processor_node.cpp` |
| `src/vision_processor_node_cuda.cpp` | 遗留 CUDA 版，不参与编译 | 承接 DexiNed 代码并改名 `vision_processor_node_dexined.cpp` |
| `src/strategy_node.cpp` | DLX 精确覆盖 + **放置顺序拓扑排序** + 像素打包 | 仅保留「放哪些块、放在哪些格」；顺序规划移交「路径」 |
| `src/xarm_controller_node.cpp` | **像素→base 坐标解算** + 运动执行 + 状态机 | 坐标解算移交「路径」，仅保留运动执行 |
| `src/xarm_controller_node_pliz.cpp` | Pilz/PTP 版，不参与编译 | **删除**（PTP 路径让波纹管吸嘴螺旋、方块受重力旋转） |
| `scripts/calibration_tool.py` | 同时做①桌面/吸取面平面拟合 ②白板格心+放置 Z | **拆分**：丢弃①，保留②为「白板标定」 |
| `scripts/pick_affine_calibration_tool.py` | 单应性矩阵标定 | 保留为「单应性标定」 |
| `scripts/table_tf_broadcaster.py` | 发布 `table_frame` | 视「待定决策 D1」决定保留/删除 |
| `srv/GetPrecisePose.srv` + `handle_precise` | 无人调用的历史遗留服务 | **删除** |

关键事实（来自现有代码，重构时要继承/搬运）：

- **抓取点 Z 现状**：`xarm_controller_node.cpp::pixelToTruePickPlaneTablePoint`
  用 `PICK_SURFACE_PLANE_BASE`（平面拟合）做射线求交得到 Z，再用
  `pixelToPickPointTable` 做 2.5D 视差压缩 + 单应性求 XY。**没有任何节点读取
  RealSense 深度**（`/camera/aligned_depth_to_color/*` 当前未被订阅）。
- **放置点**：`bilinearBoardCenter` 用 `BOARD_CENTERS_14x10_TABLE` 双线性插值
  得到 XY，Z 来自 `PLACE_Z_MAP_14x10` / `BOARD_BUMP_HEIGHT_MODEL`。
- **顺序规划现状**：`strategy_node.cpp` 已经建了「上下相邻块」的有向图并做拓扑
  排序 + 优先队列（按最低行、最左列），这正是「跨行连接」约束的雏形。
- **打分现状**：满行 +10，满行且 `unique_colors>=4` 再 +10——「每行四种不同
  颜色」已经在打分里。

---

## 1. 目标架构总览

```
[相机/RealSense RGB-D]
        │ color + aligned_depth + camera_info
        ▼
┌─────────────┐  /vision/board_state(+Z)   ┌─────────────┐
│  视觉 Vision │ ─────────────────────────▶ │ 策略 Strategy│
│ (轮廓/Dexi)  │                            │   (DLX)     │
└─────────────┘                            └─────────────┘
        │ 每块: shape, 像素uv, 角度, 深度Z        │ /tetris_plan
        │                                        │ (放哪些块/哪些格/形状序列)
        └──────────────┬─────────────────────────┘
                       ▼
                ┌─────────────┐
                │  路径 Path   │  像素→base 解算 + 建图 + 最短路径排序
                └─────────────┘
                       │ /motion_cmds (有序的 pick/place base 位姿队列)
                       ▼
                ┌─────────────┐
                │  控制 Control│  move_line(LIN) + 吸盘IO + 状态机
                └─────────────┘
                       │ /robot_status
                       └────────────────▶（回灌给上游做忙/闲门控）
```

五大类各自只做一件事：

1. **标定 Calibration** —— 离线生成 `tetris_config.yaml`，运行期不参与。
2. **视觉 Vision** —— 识别方块（形状/位置/角度），并给出每个抓取点的深度 Z。
3. **策略 Strategy** —— 解精确覆盖，决定「放哪些块、放在哪些格、对应形状」。
4. **路径 Path** —— 把像素+格子解算成 base 系实际位姿，建图求最短抓放顺序。
5. **控制 Control** —— 纯执行：收到有序位姿队列，逐条 LIN 运动 + 吸放。

---

## 2. 标定 Calibration

**目标：从 4~5 类标定缩减为 3 类。**

保留：
1. **手眼标定**（eye-on-hand，**ChArUco** + easy_handeye）——
   `xarm_calibration_setup.launch` + `calibrate_xarm.launch`，产出
   `camera_color_optical_frame` 在机械臂 TF 树中的位姿。**不动。**
   （实测 ChArUco 明显优于单 ArUco，详见 §10。）
2. **单应性标定**——`pick_affine_calibration_tool.py`（`affine.launch`），
   产出 `tetris/PICK_HOMOGRAPHY`（像素 → 平面 XY）。
3. **白板标定**——从 `calibration_tool.py` 中拆出「白板格心 + 放置高度」部分，
   产出 `BOARD_CENTERS_*`、`PLACE_Z_MAP_*`、`BOARD_BUMP_HEIGHT_*`。

删除：
- **桌面坐标系标定**——即 `calibration_tool.py` 里**独立的**平面拟合段
  （`TABLE_SURFACE_PLANE_BASE` / `PICK_SURFACE_PLANE_BASE`）。
  抓取面 Z 改由视觉深度直接给出，这一步不再需要。

> **重要约束（实测）：base-Z 不垂直于桌面。** 在 UFactory Studio 里让 TCP 沿
> base-X 直线运动（Studio 显示 Z 不变）时，机械臂物理上越来越靠近桌面；沿
> base-Y 同样下沉、且偏差量不同。说明 **base 的 XY 平面相对桌面有倾斜**（roll/
> pitch 两个分量都非零）。因此**绝不能把 base-Z 当作竖直方向**做下压/放置，
> 否则吸盘不垂直于板面、且抓放高度会随 XY 漂移。
>
> **结论（D1 定稿）**：仍需要一个「板面对齐」的竖直参考，但**不再单独标定桌面
> 坐标系**——直接从**白板标定**的平面拟合（`BOARD_SURFACE_PLANE` /
> `BOARD_GRID_FIT_*`，本来就保留）里取板面法向，作为抓取下压与放置的竖直方向
> 和工具姿态参考。这样既满足「只留 3 类标定」，又自然吸收了倾斜。
> （**实现细化**：板面法向与放置面 Z 由 `calibrate_board.py` 用**单帧对齐深度 +
> RANSAC** 拟合得到，不再触点采集；并用一段已知走位做**差分验证深度尺度**——
> 多帧×几千像素治随机噪声，走位差分治系统偏置。详见 §2 动作项。）
> `table_frame` 以何种形式存在（继续发 TF，还是只存一个法向量在 base 系里用）
> 属实现细节，二者等价；单应性与白板格心相应统一到所选参考系。

动作项：
- [x] 把 `calibration_tool.py` 拆成 `calibrate_board.py`（只做白板：格心 +
      `PLACE_Z_MAP` + `BUMP_HEIGHT` + 板面平面）。
- [x] **板面平面改由单帧对齐深度采集**（替代触点采集）：静止机位多帧逐像素中值
      + ROI（交互框选 / 参数 / 持久化复用）+ 深度范围门 + RANSAC 主平面 → 法向
      (base，定 table_frame 竖直) 与放置面 Z（同一物理白板，转 table 系）。
- [x] 新增**走位差分验证 / 尺度**步：手动挪一小段二次采集，差分估深度尺度、两次
      法向一致性、base-Z↔法向夹角（差分抵消常值偏置）；结果存 `BOARD_DEPTH_VERIFY`。
- [ ] 板面法向 → 抓取/放置的工具姿态（垂直板面，而非 base-Z）。**随路径/控制做。**
- [x] `calibrate_tool.launch` 更新到 `calibrate_board.py` + 深度/ROI 新参数
      （`board_capture_frames` / `board_roi` / `board_roi_force_interactive` /
      `board_roi_z` / `board_ransac_thresh_m` / `board_verify_scale`）；`affine.launch` 待核。
- [ ] `tetris_config.yaml` 移除 `TABLE_SURFACE_PLANE_BASE` /
      `PICK_SURFACE_PLANE_BASE` / `PICK_SURFACE_*` 系列键（真机重跑标定时整文件
      重写自然移除；保留 `.bak`）。

---

## 3. 视觉 Vision

**目标：拆成两个可切换的实现，并补齐深度 Z 输出。**

### 3.1 两个节点
- `vision_processor_node.cpp` —— **回退到经典轮廓提取 + 模板匹配**。优点是
  稳、可解释；缺点是方块贴太近时分不开。
- `vision_processor_node_dexined.cpp` —— 把当前 `vision_processor_node.cpp`
  里的 **DexiNed ONNX（CUDA）** 边缘检测搬到这里（基于遗留的
  `vision_processor_node_cuda.cpp` 文件名改造）。解决贴近粘连问题，**尚未实测**。
- 两者**话题/消息契约完全一致**，靠 `tly.launch` 的 `type:=` 切换；不要分叉契约。

### 3.2 新增：每个抓取点的深度 Z
因为删掉了桌面/吸取面平面拟合，抓取 Z 不再来自平面，改由 RealSense RGB-D：
- 订阅 `/camera/aligned_depth_to_color/image_raw`（depth 已对齐到 color，
  需在 `rs_camera.launch` 保持 `align_depth:=true`，当前已开）。
- 对每个方块抓取像素 (u,v)，在小邻域内取深度中值（抗噪/抗孔洞），得到相机系
  Z_cam，再结合 `camera_info` 反投影 + 手眼 TF 得到 base 系抓取点（XYZ）。
- **两种解算都实现，用参数切换（D2 定稿，提升鲁棒性）**：
  - `pick_xy_source: homography`（默认）——只用深度给 Z、XY 仍走单应性，
    避免深度横向噪声污染 XY 精度。
  - `pick_xy_source: depth`——全用深度反投影出 XYZ，单应性失效/未标定时的兜底。
  - 深度无效（黑色/反光/孔洞）时回退：邻域扩大重采 → 再不行用「板面常数 Z」。

### 3.3 输出契约变更
当前 `/vision/board_state`（`Int32MultiArray`）每块是 4 或 6 个 int
（`shape,u,v,angle[,geom_u,geom_v]`），**无 Z**。需要把 Z 带出来：
- 方案 A（最小改动）：扩展 `board_state` 每块 stride，追加 `z_mm`（整数毫米）。
  `strategy_node` 现有解析已按 stride 自适应，向后兼容成本低。
- 方案 B（更干净，**采用**）：新增 `geometry_msgs/PoseArray` 或自定义消息承载
  base 系抓取位姿（XYZ+角度+shape），`board_state` 只留库存+占用栅格供策略用。
  让「路径」直接拿到带 Z 的物理点，库存/栅格继续走 `board_state`。

动作项：
- [ ] 抽出公共部分（分割→轮廓→模板匹配→跟踪稳定化）为共享头/库，避免两节点
      重复维护跟踪与模板逻辑。
- [ ] 实现深度采样模块（邻域中值 + 有效性校验 + 反投影）。
- [ ] 订正「black blocks」等不准确注释。
- [ ] 删除 `GetPrecisePose` 服务及其 handler。

---

## 4. 策略 Strategy

**目标：保留 DLX 精确覆盖求解，新增「进阶任务：可指定每步形状序列」。**

保留：
- `BASE_SHAPES` 七种形状 + 旋转、DLX 引擎、多 `PlanConfig`/子库存搜索、
  打分（满行 +10、满行且 ≥4 色再 +10）。

两种工作模式：
- **普通模式（求解器）**：保持现状目标——自由选块/选格，最大化满行分。
- **进阶模式（带附加硬约束的求解器，D4 修订）**：在普通模式的求解之上，外部
  下发一个**有序形状序列**作为**额外的硬约束**——第 i 步放置的方块必须是序列
  里的第 i 个形状（颜色随形状绑定）。策略**仍然是求解**：在该约束下搜索每个
  方块的目标格/旋转/位置，使连接性（硬）成立并最大化满行+四色分；**不是**只对
  一个给定完整解做合法性校验。
  - 关键耦合：形状序列规定了**放置顺序**，而连接性约束也作用在放置顺序上——
    求解时要保证「按序列顺序逐个放置、且每一步落子都连接合法」。
  - 序列来源：话题或参数（如 `/strategy/shape_sequence`）。颜色↔形状绑定，
    指定形状即指定颜色，无需额外颜色识别。

偏序约束（D3 定稿）——策略输出**偏序**，路径在偏序下求最短：
1. **跨行连接 = 硬约束**：每个方块放下时必须与已放置结构在相邻行相连，否则该
   次放置无效。当前 `strategy_node.cpp` 已用「上下相邻块」有向图 + 拓扑排序近似
   实现，需**显式化为放置合法性偏序**（前驱块必须先放）。**永远生效。**
2. **从左到右 = 软约束（带开关）**：默认希望同层方块从左到右依次放置，但可由
   开关 `enforce_left_to_right` 关闭。软约束不进硬偏序，而是作为**路径目标里的
   惩罚项**（开关开 → 偏离左到右加惩罚；关 → 不惩罚），由路径在求最短时权衡。
3. **每行尽量四种不同颜色**：已在打分里（`unique_colors>=4`），保留。

职责边界：连接约束（硬偏序）的正确性留在**策略**；行程优化与软约束权衡留在
**路径**。策略产出「放哪些块/哪些格/形状 + 硬偏序 + 软约束标记」，路径在硬偏序
下求最短（软约束作惩罚）。

输出契约（`/tetris_plan`）调整为：
- 每个动作：`shape, way, 目标4格(r,c)×4`，外加 `must_after`（前驱块 id 列表，
  表达**硬**连接偏序）。**不再**在这里塞像素 uv —— 像素抓取点由「路径」从视觉
  侧直接获取，不经过策略中转（现状是策略把视觉像素打包进 plan，重构后解耦）。

动作项：
- [ ] 抽离 DLX 引擎为独立可单测的模块（纯算法，无 ROS）。
- [ ] 输出每块的硬连接前驱（偏序），替代节点内的「最终拓扑排序输出」。
- [ ] 模式开关：普通=自由求解；进阶=吃外部形状序列做落位+校验。
- [ ] `enforce_left_to_right` 开关（软约束，传递给路径作惩罚）。

---

## 5. 路径 Path（新模块）

**目标：承接「坐标解算 + 顺序规划」，这是从控制/策略里剥出来的新职责。**

输入：
- 视觉的每块带 Z 抓取点（像素 uv + 角度 + 深度 Z，或已是 base 系位姿）。
- 策略的放置计划（每块 shape/目标格/连接偏序）。

处理：
1. **抓取点解算**：把视觉像素解算成 base 系实际抓取位姿——搬运现有
   `xarm_controller_node.cpp` 的 `pixelToNormalizedRay` / 单应性 XY / 2.5D 视差
   补偿；Z 改用视觉深度（不再用平面拟合）。工具姿态用**板面法向**（非 base-Z）。
2. **放置点解算**：把目标格 (r,c) 用 `BOARD_CENTERS_*` 双线性插值 +
   `PLACE_Z_MAP_*`/bump height 解算成 base 系放置位姿——搬运现有
   `bilinearBoardCenter` 等。姿态同样用板面法向。
3. **建图 + 最短路径（在偏序下）**：以「当前 TCP 位置 → 抓取点 →（载块）
   放置点 → 下一个抓取点 …」为节点/边建图。
   - **硬约束**：必须满足策略给的连接偏序（前驱块先放）——受约束 TSP / 拓扑序
     内的最短路（带优先级最近邻 + 2-opt 改良起步即可）。
   - **软约束**：`enforce_left_to_right` 开时，把「偏离同层左→右顺序」作为目标
     函数里的惩罚项，与行程长度一起加权最小化；关时不计惩罚。

输出：
- `/motion_cmds`：**有序**的运动指令队列，每条已是 base 系位姿（pick hover /
  pick / place hover / place）+ 吸/放标志。控制端不再做任何坐标换算。

动作项：
- [ ] 新建 `src/path_planner_node.cpp` + 对应 launch 接线。
- [ ] 把控制节点里所有 `pixel*Table` / `*BoardCenter` / 单应性 / 视差代码迁入。
- [ ] 实现受偏序约束的最短路径排序（先正确，再优化）。
- [ ] 定义 `/motion_cmds` 消息（建议自定义 msg：位姿序列 + 动作类型枚举）。

---

## 6. 控制 Control

**目标：退化为纯执行器。**

保留并精简 `xarm_controller_node.cpp`：
- 只保留 xArm **原生服务**驱动：`/xarm/set_mode`、`/xarm/set_state`、
  `/xarm/move_line`、吸盘 `set_controller_dout`，以及状态机
  （`IDLE → TAKE_NEXT_TASK → MOVE_TO_PICK_HOVER → EXECUTE_PICK →
   MOVE_TO_PLACE_HOVER → EXECUTE_PLACE → FINISH`）。
- **全程 LIN（move_line）**，不用 PTP/Pilz——PTP 规划的路径让波纹管吸嘴出现
  螺旋姿态，方块在重力下旋转。删除 `xarm_controller_node_pliz.cpp`。
- 删除所有坐标换算（已迁入「路径」）：移除 `pixelToPickPointTable`、
  `bilinearBoardCenter`、单应性/视差/平面相关成员与参数加载。
- 输入从 `/tetris_plan` 改为 `/motion_cmds`（已是 base 位姿，直接执行）。
- 保留 `/robot_status`（忙/闲，latched）回灌做门控；保留放置前抖动、吸放等待、
  轨迹跳变保护等执行期安全参数。

动作项：
- [ ] 裁剪控制节点到「队列消费 + LIN + IO + 安全保护」。
- [ ] 适配新输入消息 `/motion_cmds`。

---

## 7. 数据流与接口契约（重构后）

| 话题/服务 | 类型 | 生产者 → 消费者 | 说明 |
|---|---|---|---|
| `/vision/board_state` | `Int32MultiArray` | 视觉 → 策略 | 7 库存 + 140 占用栅格（策略需要的离散信息） |
| `/vision/pick_targets`（新） | `PoseArray` 或自定义 | 视觉 → 路径 | 每块 base 系抓取位姿（含深度 Z）+ shape + 角度 |
| `/strategy/shape_sequence`（新，仅进阶） | 自定义/`Int32MultiArray` | 外部 → 策略 | 有序形状序列，作为求解的附加硬约束（第 i 步放指定形状） |
| `/tetris_plan` | 自定义/`Int32MultiArray` | 策略 → 路径 | 放哪些块/哪些格/形状 + 硬连接偏序（`must_after`）+ 软约束标记 |
| `/motion_cmds`（新） | 自定义 | 路径 → 控制 | 有序 base 位姿队列 + pick/place/吸放标志 |
| `/robot_status` | `Bool`(latched) | 控制 → 上游 | 忙/闲门控 |

> 契约是跨节点强约束，改任意一个都要同步更新所有收发端。形状 ID 仍是全局契约
> （0 一字 / 1 田 / 2 T / 3 L左 / 4 L右 / 5 Z左 / 6 Z右）。

---

## 8. 已定决策（已确认）

- **D1 竖直参考**：实测 base-Z **不**垂直于桌面（沿 base-X/Y 平移会下沉，且
  X、Y 偏差不同 → base 平面相对桌面有倾斜）。**不删除「板面对齐的竖直参考」**，
  但**取消独立的桌面坐标系标定**：板面法向改由**白板标定**的平面拟合得到，用于
  抓取下压与放置的竖直方向/工具姿态。`table_frame` 以 TF 还是法向量形式存在为
  实现细节。详见 §2。
- **D2 深度 Z 用法 + 输出消息**：**两种解算都做、参数切换**
  （`pick_xy_source = homography | depth`）以提升鲁棒性；输出采用**方案 B**
  （新增带 Z 的抓取位姿消息），`board_state` 仅留库存/栅格。详见 §3。
- **D3 偏序与最短路径**：策略输出偏序、路径在偏序下求最短。偏序含两类约束——
  **跨行连接=硬约束**（始终生效，进硬偏序）；**从左到右=软约束**（开关
  `enforce_left_to_right`，作为路径目标的惩罚项）。详见 §4、§5。
- **D4 进阶任务语义（修订）**：外部下发**有序形状序列**，作为在普通求解之上
  **附加的硬约束**（第 i 步必须放指定形状）；系统**仍需求解**放置（格/旋转/
  位置）以满足连接性并最大化得分——**不是**只验证一个给定完整解。详见 §4。

---

## 9. 分阶段迁移（进度跟踪）

- [x] **1 清理死代码**：删 `xarm_controller_node_pliz.cpp`、`GetPrecisePose`
  服务及调用面；`CMakeLists.txt` 已校。
- [x] **2 视觉拆分**：DexiNed → `vision_processor_node_dexined.cpp`，
  `vision_processor_node.cpp` 回退经典法；契约一致，`tly.launch` `vision_node:=` 切换。
- [x] **3 深度 Z**：视觉订阅对齐深度，`DepthSampler` 采样抓取点 Z（含 D415
  rect→raw 修正），log + `/vision/pick_depth_debug` 调试话题。⚠️ 数值待真机验证。
- [x] **4 标定缩减**：`calibration_tool.py` → `calibrate_board.py`；板面法向 + 放置面 Z
  改由**单帧对齐深度 + ROI + RANSAC**（替代触点），新增走位差分验证/尺度步；法向定
  table_frame 竖直（D1）。⚠️ 需真机重跑标定验证（命令见 [test_schedule.md](test_schedule.md) 第 3 项）。
- [~] **5 路径模块（5a 已做，5b 待真机）**：
  - [x] **5a** 新建 `path_planner_node`（附加式）：坐标解算 + 深度 Z，发布
    `/motion_cmds`；控制器未动、仍吃 `/tetris_plan`。⚠️ `/motion_cmds` 待真机对照。
  - [ ] **5b** 控制器切吃 `/motion_cmds`、删坐标解算（含 yaw 0/180° 翻转归属）。**需真机。**
- [x] **6 策略解耦 + 进阶求解器**：
  - [x] DLX 抽成 `tetris_solver.hpp`（可离线单测）。
  - [x] 进阶 `solveSequence`（beam search）+ 规则③(下方支撑)/规则④(<3 形状不计分)，
    离线测试全过。
  - [x] `strategy_node` 接入 `advanced_mode`/`shape_sequence`，打包进现有
    `/tetris_plan`（不改契约），roslaunch 冒烟通过。
  - [ ] **输出契约解耦**（策略不再打像素/不做最终排序）随 5b 一起做。**需真机。**
- [ ] **7 最短路径**：path 里实现受偏序约束的排序替换直通顺序。**依赖 5b**（要改
  `/tetris_plan`，而控制器届时已挪到 `/motion_cmds`）。

每个节点可用 `test_vision/strategy/path/controller.launch` 单独回归对应链路。

---

## 10. 风险与注意

- **base 倾斜（已实测）**：base-Z 不垂直于桌面。竖直方向/工具姿态务必用白板
  标定得到的板面法向；不要用 base-Z。另：用户判断这可能是机械臂本体的
  运动学误差（底座已固定）——若误差是位置相关的非平面形变，单纯平面法向修不
  净，但**密集的逐格 `PLACE_Z_MAP` + 逐像素深度 Z 是在真实位置采样/测量的**，
  能经验性吸收倾斜与轻度运动学误差；必要时加密白板采样点。
- **手眼旋转质量门控深度法向（已踩坑）**：深度反投影、板面法向、`align_tool` 对齐
  都经 `base←camera` 手眼 TF，精度上限 = 手眼**旋转**标定精度。实测旧的**单 ArUco**
  手眼旋转偏约 **15°**，导致深度法向/对齐/深度版白板标定全部偏 15°；换 **ChArUco**
  标定板（`charuco_tracker.py`，11×8 格）后降到 **~1.4°**，与**触点法向**（纯运动学、
  不经相机，是地面真值）一致。→ **手眼优先用 ChArUco，不用单 ArUco**；深度法向采信
  前先对照触点法向，或用 `align_tool.py mode:=diag`（多姿态测法向漂移）验收。
  附带澄清：桌面相对 base-Z 的**真实**倾斜只有 ~1.4°（之前直觉的"大倾斜"里那 15°
  其实是手眼误差，非真实倾斜）。
- **相机深度滤波**：RealSense 起相机统一开 `filters=spatial,temporal,hole_filling`
  （`tly.launch` 与 `align_tool`/`verify_camera`/`calibrate_tool` 均已加，可用
  `depth_filters:=` 覆盖）；背光发光板半透/反光，无滤波时深度孔洞多，反投影/深度 Z
  受影响。
- **深度噪声**：RealSense 深度在黑色/反光/边缘处易出孔洞；抓取点取邻域中值并
  做有效性兜底（无效时回退到板面常数 Z）。
- **契约迁移期**：第 5~6 步会同时改多个收发端，务必一次性对齐 msg 定义。
- **连接约束 ↔ 最短路径耦合**：最短不能破坏放置合法性；务必在硬偏序下优化，
  软约束（左到右）只作惩罚项。
- **`-O3` 编译**：DLX 与（新增的）路径搜索都是算力敏感，保留 `CMakeLists.txt`
  里的 `-O3`。

---

## 11. 真机验证清单（已编译/离线验证，待真机确认）

下列改动我（助手）只做了编译 / 离线测试，**无法实跑真机**，需在硬件上确认。
逐项可执行命令与通过判据见 [test_schedule.md](test_schedule.md)。

- [ ] **深度 Z（步骤3）**：`roslaunch tly tly.launch`，看 `[DEPTH] pick Z` 日志或
  `rostopic echo /vision/pick_depth_debug`，确认 Z ≈ 相机到积木顶面距离（~0.7m 量级）；
  并核对 `/camera/color/camera_info` 的 `D`：若非 0，则 rect→raw 映射应让
  `(rect)->raw` 有几像素偏移、Z 才准。
- [ ] **标定缩减（步骤4，深度版）**：重跑 `roslaunch tly calibrate_tool.launch`（跑
  `calibrate_board.py`）。首次交互框选 ROI（空板、看白板平坦区），确认：拟合内点率/
  残差(mm级)合理、走位验证 `深度尺度比≈1` / `两次法向夹角≈0` / `base-Z↔法向夹角`
  (实测倾斜值)、新键 `BOARD_SURFACE_NORMAL_BASE` / `BOARD_SURFACE_PLANE` /
  `BOARD_DEPTH_ROI` / `BOARD_DEPTH_VERIFY` 及 `table_tf` / `BOARD_CENTERS` /
  `PLACE_Z_MAP` 合理。注意：重跑整文件重写 `tetris_config.yaml`，旧
  `PICK_SURFACE_PLANE_BASE` 消失、旧控制器抓取 Z 回退 flat `PICK_Z`（与迁移方向一致）。
- [ ] **路径节点 5a**：`roslaunch tly test_path.launch`（机械臂只供 TF、不运动），
  `rostopic echo /motion_cmds`；对照 `[PATH][TASK]` 与 `[CTRL][TASK]` 日志——XY 应
  与控制器一致，`z_depth` vs `z_plane` 看深度相对平面的差异。**这是 5b 切换前的前置验证。**
- [ ] **进阶模式（步骤6）**：`roslaunch tly tly.launch advanced_mode:=true`，
  `shape_sequence` 按现场抽签填；确认 `/tetris_plan` 的放置顺序/落子合法、首块落最底行。
- [ ] **5b 切换后**：控制器吃 `/motion_cmds` 后，先慢速、`max_tasks_per_plan` 设小值
  跑单块，确认抓放位姿与原 `/tetris_plan` 路径一致再放开。
- **LIN 全程**：贴近障碍/边缘格的直线下压要留够 hover 余量，避免直线段撞邻块。
```
