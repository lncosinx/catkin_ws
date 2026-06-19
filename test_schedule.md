# test_schedule.md — 验证排期（已实现但未确认）

对照 [plan.md](plan.md) §9 迁移进度与 §11 真机清单，列出**已实现但还没确认**的
改动，给出逐项测试命令与通过判据。本次会话新增的「深度板面标定」也并入第 3 项
（plan.md 里 §9-4 / §11 还写的是旧触点流程，尚未同步）。

**不在本排期内**（尚未实现，见 plan.md §9 待办）：5b 控制器切 `/motion_cmds`、
策略输出契约解耦、§7 最短路径。

---

## 状态总览

| # | 改动 | plan.md | 当前状态 | 需硬件 |
|---|---|---|---|---|
| 0 | 进阶求解器（离线） | §9-6 | ✅ **已确认**（本次 `ALL TESTS PASSED`） | 否 |
| 1 | 视觉拆分（经典/DexiNed 切换） | §9-2 | ⏳ 待真机冒烟 | 相机 |
| 2 | 抓取点深度 Z | §9-3 / §11 | ⏳ 数值待真机验证 | 相机 |
| 3 | 板面标定（**深度重构**） | §9-4 / §11 | ⏳ 待真机重跑 | 臂+相机 |
| 4 | 路径节点 5a（`/motion_cmds`） | §9-5a / §11 | ⏳ 待真机对照 | 臂(仅TF)+相机 |
| 5 | 进阶模式真机 | §9-6 / §11 | ⏳ 待真机 | 相机 |

---

## 前置

```bash
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws && catkin_make            # 改了 launch/py 不需重编，改了 C++ 节点要
source devel/setup.bash
```

硬件项均需 RealSense（`rs_camera.launch align_depth:=true`，已默认开）。涉及臂的项
需 `robot_ip:=192.168.1.228`。

---

## 0. 进阶求解器（离线，无硬件）— ✅ 已确认

```bash
cd /root/catkin_ws/src/tly
g++ -std=c++14 -O2 -I include test/test_tetris_solver.cpp -o /tmp/tst && /tmp/tst
```
**判据**：末行 `ALL TESTS PASSED`；每个用例 `verify: OK`（首块触底、连接/支撑成立、
计分自洽、规则④ <3 形状不计分门生效）。本次已跑通。改动 [tetris_solver.hpp](src/tly/include/tly/tetris_solver.hpp)
或 [test_tetris_solver.cpp](src/tly/test/test_tetris_solver.cpp) 后回归此项即可。

---

## 1. 视觉拆分（步骤2）

经典轮廓+模板 与 DexiNed 两实现共用契约，靠 `vision_node:=` 切换。

```bash
# 经典版
roslaunch tly test_vision.launch
# DexiNed 版
roslaunch tly test_vision.launch vision_node:=vision_processor_node_dexined
```
观察：
```bash
rostopic echo -n1 /vision/board_state          # data.size() >= 147 (7库存+140栅格+…)
rqt_image_view /vision/debug_image
```
**判据**：两实现都能稳定输出 `board_state`；`debug_image` 里方块分割/形状/角度标注
正确；贴近方块时 DexiNed 应比经典版更能分开（plan.md §3.1 备注 DexiNed「尚未实测」，
这是首次确认点）。

---

## 2. 抓取点深度 Z（步骤3）

```bash
roslaunch tly tly.launch robot_ip:=192.168.1.228
# 或仅视觉链路：roslaunch tly test_vision.launch
rostopic echo /vision/pick_depth_debug          # 深度采样调试 (Float32MultiArray)
```
观察 `[DEPTH] pick Z` 日志 + `/vision/pick_depth_debug`。
**判据**：
- 每块 Z ≈ 相机到积木顶面距离（**~0.7m 量级**），不随机跳变。
- 黑色/反光/孔洞处有邻域中值兜底，无效时回退板面常数 Z（看日志有无回退告警）。
- 核对内参畸变：`rostopic echo -n1 /camera/color/camera_info` 的 `D`——若非 0，
  确认 rect→raw 修正让 `(rect)->raw` 有几像素偏移，Z 才准（plan.md §11）。

---

## 3. 板面标定 — 深度重构（步骤4 + 本次会话）★ 重点

本次把 [calibrate_board.py](src/tly/scripts/calibrate_board.py) 的 Step 2/Step 6
由**触点采集**改成**单机位对齐深度 + ROI + RANSAC**；新增**走位差分验证/尺度**步。
launch 见 [calibrate_tool.launch](src/tly/launch/calibrate_tool.launch)。

```bash
# 首次（弹窗交互框选 ROI；空板、相机看白板平坦区）
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.228
# 复用上次 ROI（不弹窗）：保持 board_roi 为空且 force_interactive=false 即自动读 config 里 BOARD_DEPTH_ROI
# 强制重画 ROI：
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.228 board_roi_force_interactive:=true
# 直接给定 ROI（headless）：
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.228 board_roi:="u0,v0,w,h"
```
新参数（[calibrate_tool.launch](src/tly/launch/calibrate_tool.launch)）：
`board_capture_frames`(50) / `board_roi` / `board_roi_force_interactive` /
`board_roi_z`([0.10,1.50]) / `board_ransac_thresh_m`(0.004) / `board_verify_scale`。

**判据**：
1. **ROI 交互**：弹深度伪彩预览能拖框；画完写入 `tetris_config.yaml: tetris.BOARD_DEPTH_ROI`；
   第二次跑不弹窗、日志显示 `ROI=…（来源:persisted）`。
2. **平面拟合**：日志 `内点/总点` 内点率高（>70%）、`residual RMS/max` 在 **mm 级**；
   ROI 要框在平坦区，内点率低或残差大说明框到凸起/杂物。
3. **走位验证**（手动挪 2~5cm 再采）：
   - `深度尺度比 ≈ 1.0`（偏离 → 深度尺度有问题，Step 2 绝对 Z 也别全信，回查手眼/深度）。
   - `两次法向夹角 ≈ 0°`（大 → TF 或深度不自洽）。
   - `base-Z 与板面法向夹角` 应是个**明显非零**的小角（正是 plan.md §2/§10 实测的倾斜，
     记下这个实测值）。
4. **放置面 Z**：日志 `放置面 z(table) 中值` + `残差 std`（mm 级）。
5. **输出核对**：`tetris_config.yaml` 新键 `BOARD_SURFACE_NORMAL_BASE`、`BOARD_SURFACE_PLANE`、
   `BOARD_DEPTH_ROI`、`BOARD_DEPTH_VERIFY` 合理；若有 `.bak` 旧触点标定，法向应与旧值接近。
6. **回归依赖**：重跑会整文件重写 `tetris_config.yaml`，旧 `PICK_SURFACE_PLANE_BASE`
   消失属预期（plan.md §11）；之后跑第 4 项确认 `/motion_cmds` 仍正常。

---

## 4. 路径节点 5a（步骤5a）

[path_planner_node.cpp](src/tly/src/path_planner_node.cpp) 解算并发布 `/motion_cmds`
（`tly::MotionPlan`），控制器**未动**、仍吃 `/tetris_plan`。这是 5b 切换前的前置对照。

```bash
roslaunch tly test_path.launch robot_ip:=192.168.1.228     # 臂只供 TF，不运动
rostopic echo /motion_cmds
```
**判据**：对照 `[PATH][TASK]` 与 `[CTRL][TASK]` 日志——
- pick/place 的 **XY 应与控制器一致**。
- `z_depth`（深度）vs `z_plane`（平面回退）差异符合预期；默认 `pick_xy_source=homography`
  （[tly.launch](src/tly/launch/tly.launch) L245），XY 走单应性、只有 Z 用深度。
- 切 `pick_xy_source:=depth` 应改为全深度反投影 XYZ，可作兜底对照。

---

## 5. 进阶模式真机（步骤6）

```bash
# 仅策略链路（无硬件也能跑，喂假 board_state）
roslaunch tly test_strategy.launch advanced_mode:=true
# 全链路真机
roslaunch tly tly.launch robot_ip:=192.168.1.228 advanced_mode:=true
rostopic echo /tetris_plan
```
`shape_sequence` 默认 `[0,1,2,3,4,5,6]`，真机按现场抽签改
（[test_strategy.launch](src/tly/launch/test_strategy.launch) L29 / [tly.launch](src/tly/launch/tly.launch) L271）。
**判据**：`/tetris_plan` 的放置**第 i 步形状 == 序列第 i 个**；首块落**最底行**；
每步连接/下方支撑合法；`[ADVANCED]` 日志 `placed/score` 与离线一致。

---

## 附：本次未跑/未实现

- **5b / 输出契约解耦 / §7 最短路径**：尚未实现，等 5b 落地后再排期（plan.md §9-5b/§9-7）。
- 5b 切换后单块慢跑验证、LIN 全程避撞——见 plan.md §11 末两条，到时并入本表。
