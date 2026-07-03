# 标定指南

本文档介绍 `tly` 工作区的全部标定流程：**手眼标定**（相机↔机械臂）、**白板标定**
（放置网格几何 + 放置高度 + 抓取单应性）、以及若干**验证/对齐辅助工具**。

所有标定结果最终都写入 `src/tly/config/tetris_config.yaml`（手眼结果除外，它由
`easy_handeye` 存到自己的目录，由 `publish.launch` 在运行时广播）。这些文件当作数据，
不要手改。

> 物理叠放（务必记清）：**白板（14×10 放置网格，带 140 个 2~3mm 凸起）叠在发光板
> （背光板，抓取/散料源区）上面**。`board_frame` 的 z=0 平面在**发光板**上；白板表面/
> 凸起在 board 系 z 为正。判定某格是否摆了方块要以**逐格白板凸起顶面**为基准，不能用
> 发光板平面。

---

## 标定总览与顺序

| 阶段 | 工具 launch | 产物 | 何时重跑 |
|------|-------------|------|----------|
| 1. 相机内参验证（可选自检） | `verify_camera.launch` | 仅打印/校验，不写盘 | 换相机或怀疑内参时 |
| 2. 手眼标定 | `xarm_calibration_setup_moveit.launch`（自动采样，不推荐）或 `xarm_calibration_setup.launch`（手动 freehand） | `xarm6_realsense_calibration_eye_on_hand`（base↔camera） | 换相机/重装相机支架/移动相机后 |
| 3. 白板标定（含抓取单应性） | `calibrate_tool.launch` → `calibrate_board.py`（7 步） | `BOARD_POSE_BASE`、网格几何、放置 Z 图、`PICK_HOMOGRAPHY` 等 | 手眼变了、白板/发光板移动后 |
| 验证 · 抓偏诊断 | `pixel_touch_check.launch` → `pixel_touch_check.py` | 仅诊断（两条预测 vs 触点误差 + URDF↔固件系统差），只存 `/tmp` | 怀疑抓偏/吸偏、想定位偏差来源时 |
| 验证 · 角速度标定 | `measure_tcp_omega.launch` → `measure_tcp_omega.py` | `ω/v_lin` 比值 → `path_planner` 的 `omega_per_v_lin_rad_per_m` | 改了 TCP 或关节加速度/jerk 后 |

**依赖链（顺序不能乱）**：手眼标定是一切的基础——手眼一变，白板标定（尤其依赖
`camera→base` TF 的步骤 2、抓取单应性）全部失配，必须重跑。白板标定内部步骤之间也有
依赖：重跑步骤 2 会让 3/4/5/6 失配；重跑步骤 3（网格+坐标系）会让 4/5/6 失配。

---

## 0. 相机内参验证 —— `verify_camera.launch`

只起 RealSense + `verify_camera_intrinsics.py`，**不需要机械臂**。用于在标定前确认相机
内参/深度尺度可信。不写任何配置。

```bash
roslaunch tly verify_camera.launch mode:=report
```

参数：
- `mode`：`report` / `points` / `aruco` 的逗号组合（默认 `report,points`）。
  - `report`：打印 `/camera/color/camera_info` 内参摘要。
  - `points`：在图上点两点，配合 `known_distance_m`（尺子量的真实距离, m）校验深度尺度。
  - `aruco`：检测打印的 ArUco，配合 `marker_length_m`（marker 实际边长, m）校验。
- `capture_frames`：多帧平均的帧数（默认 30）。

深度对齐到彩色、滤波 `spatial,temporal,hole_filling`——与后续手眼/白板标定保持一致。

---

## 1. 手眼标定（Eye-on-Hand）

相机固连在末端（eye-on-hand），用 `easy_handeye` 标定 **base ↔ camera** 变换。标定靶为
**ChArUco 整板**（calib.io：列 11 × 行 8，方格 15mm，标记 11mm，`DICT_4X4_50`），由
`charuco_tracker.py` 检测并广播 `camera_color_optical_frame → charuco_board_frame`。

关键约定：
- 标定末端用**真实吸盘尖端 `link_tcp`**（不是法兰盘 `link6`）。`link_tcp` 由静态 TF
  `args="0.00075 0.0004 0.068 0 0 0 link6 link_tcp"` 挂在 `link6` 下（单位米；此偏移经
  `pixel_touch_check` 侧向误差诊断精修，各标定/诊断栈保持一致）。
- `namespace_prefix=xarm6_realsense_calibration`，结果存为
  `xarm6_realsense_calibration_eye_on_hand`，供 `tly.launch`/各标定栈里的
  `easy_handeye/publish.launch` 运行时广播。
- 标定阶段用 MoveIt 没问题；**生产抓放才必须走原生 `move_line`**（避免规划出曲线把吸住
  的方块转向）。MoveIt 规划器用 **OMPL**，不用 Pilz（Pilz 偶发关节加速度突增）。

有两种采样方式：

### 1a. 自动采样（不推荐）—— `xarm_calibration_setup_moveit.launch`

一键起 **xArm6 MoveIt 真机栈（OMPL）+ RealSense + ChArUco 检测 + easy_handeye 自动采样**。
easy_handeye 通过 MoveIt `move_group`（SRDF 组名 `xarm6`）自动遍历多姿态采样。

```bash
roslaunch tly xarm_calibration_setup_moveit.launch robot_ip:=192.168.1.216
```

常用参数：
- `robot_velocity_scaling` / `robot_acceleration_scaling`：关节限速比例 0~1，**先用 0.1
  慢速**，确认安全再调快。
- `rotation_delta_degrees`（默认 25）/ `translation_delta_meters`（默认 0.1）：采样运动
  范围。转角越大旋转越多样、手眼越准，但太大会丢标记/碰撞。
- ChArUco 板参数 `squares_x/y`、`square_length`、`marker_length`、`dictionary`：换板才改。

流程：在 easy_handeye 的 rviz/GUI 里点 “Next/Take sample” 让臂自动走位并采样（一般 ≥15
个姿态），采够后点 “Compute” 计算并 “Save”。

### 1b. 手动 freehand 采样 —— `xarm_calibration_setup.launch`

纯 **xArm 原生驱动**（`xarm6_server`，无 MoveIt），手动拖动/示教臂到不同姿态，每个姿态
手动 “Take sample”。适合不想起 MoveIt、或想完全手控的场合。

```bash
roslaunch tly xarm_calibration_setup.launch robot_ip:=192.168.1.216
```

它内部 `include` 了 `calibrate_xarm.launch`（默认 `freehand_robot_movement:=true`）。

### 底层：`calibrate_xarm.launch`

上面两个 setup 都通过它调用 `easy_handeye/calibrate.launch`，封装了固定坐标系约定：
`robot_base_frame=link_base`、`robot_effector_frame=link_tcp`、
`tracking_base_frame=camera_link`、`tracking_marker_frame=charuco_board_frame`。

- `freehand_robot_movement`：`true`=手动（本栈默认，无需 MoveIt）；`false`=easy_handeye
  自动采样，**需要 MoveIt `move_group`**。注意：纯原生驱动栈没起 `move_group`，此时设
  `false` 会让 `robot.py` 因缺 `get_planning_scene` 而崩——所以自动采样必须走
  `xarm_calibration_setup_moveit.launch`。
- `move_group`：自动采样的规划组名，xArm6 是 `xarm6`（不是 easy_handeye 默认的
  `manipulator`）。
- `rotation_delta_degrees` / `translation_delta_meters` / 速度加速度：同上。

> 一般不直接单独跑 `calibrate_xarm.launch`，而是通过两个 setup launch 之一。

---

## 2. 白板标定 —— `calibrate_tool.launch`

起 **MoveIt 真机栈 + RealSense + 手眼 TF + image_proc + `calibrate_board.py`**，交互式
完成放置侧的全部几何/高度/抓取单应性标定。手眼 TF（`base ← camera_color_optical_frame`）
是必需的——步骤 2 深度拟板面要把法向从相机系换到 base 系。

```bash
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216
```

### 七个步骤（`STEP_TITLES`）

1. **拍照起点位姿 + 相机内参**：记录拍照起点。
2. **深度拟合白板平面**：单机位、深度多帧中值 + ROI + RANSAC 拟板面 → 板面法向定义
   `board_frame` 的 Z 轴（base 系）。这是 z=0 的**发光板**平面基准。
3. **白板凸起网格 + 由网格派生 `board_frame`**：在 base 系采棋盘网格凸起（A=9 / B=25 /
   C=140 点），仿射拟合网格 + 高度图，**由网格派生 `BOARD_POSE_BASE`**（原点=网格原点，
   X≈行轴，Z=板面法向），放置朝向自动跟随网格。X 轴贴合现有坐标系以保持策略的 `way`
   约定。
4. **方块厚度**：触方块顶面 + 触旁边裸板面，沿法向取差（同波纹管状态，恒正、免疫绝对
   零点）→ `block_thickness`。
5. **放置 Z**：分批在白板上摆方块，按格心采深度**方块顶面** → 每格放置 Z（自动含板面
   翘曲 + 凸起 + 多格架桥）。裸板面（步骤 2 平面）仅作“有无方块”覆盖判据基准。
6. **波纹管长度补偿（可选）**：拆/装波纹管两次触同一点，两次 `link_tcp` 差 →
   `TCP_PICK_OFFSET_Z` / `TCP_PLACE_OFFSET_Z`，由控制/路径节点自动读取。
7. **抓取单应性**：在整流彩色图上点选发光板黑点，标定 `pixel → board-XY` 的
   `PICK_HOMOGRAPHY`。

### 部分标定（只跑某些步骤）

用 ROS 参数 `~steps` 选择，未选中的步骤从现有 `tetris_config.yaml` 读取；不给 `~steps`
会交互式提示。每完成一步即落盘该步参数（增量保存，后续崩溃不丢前面结果）。

```bash
# 例：只重跑步骤 4、5
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216 steps:=4,5
# 支持 '4' / '3,4,5' / '2-6' / 'all'
```

> 依赖提醒：重跑步骤 2 → 3/4/5/6 失配；重跑步骤 3（网格+坐标系）→ 4/5/6 失配，且必须
> 重跑步骤 7（抓取单应性）。**任何重定坐标系都要一并重跑 `pick_affine`/步骤 7。**

### 常用可调参数

白板标定模式与高度插值：
- `board_mode`：`A`=9 点 / `B`=25 点（默认）/ `C`=140 点。白板翘曲明显建议 `C`。
- `height_interpolation`：`idw`（默认，C 模式建议）/ `quadratic`。
- `place_z_margin`：放置高度额外余量（m，默认 0.001，防刮蹭）。

ROI / 平面拟合（相机被迫降低后调）：
- `board_roi`：空=首次交互框选并存盘复用；`board_roi_force_interactive:=true` 强制重选。
- `board_roi_use_color`：`true`=彩色底图框选（更易看清板面），`false`=深度伪彩。
- `board_roi_z`（默认 `[0.10, 1.50]`）：深度范围门（m）。
- `board_ransac_thresh_m`（默认 0.004）：平面内点阈值。
- `board_min_inliers_frac`（默认 0.3）：主平面最小占比安全闸——宁可调小 ROI 也别轻易降。

步骤 5 放置 Z（方块顶面深度）：
- `place_empty_baseline`（默认 true）：先采空白板深度作逐格基准（纯深度对深度，免疫触点
  零点偏差）。
- `place_corner_clicks`（默认 true，**首选**）：步骤 5 开始现场点白板四角，当场拟合投影
  单应性（当前机位 + 白板本身，最准，免手眼/抓取单应性外推）。
- `place_use_homography`（点角关闭时的次选）：`true`=用 `PICK_HOMOGRAPHY`（抓取区标的，
  用到白板是外推、可能偏）；`false`=手眼 3D 投影。
- `place_cover_min_m`（0.005）/ `place_top_z_span_m`（0.040）：有方块 ⟺ 顶面高出该格凸起
  落在 `[cover_min, top_z_span]` 绝对窗口内（不依赖方块厚；上限只为排除飞点/叠放）。
- `place_show_overlay`（默认 true）：每批弹彩色覆盖调试图（绿=新增/青=既有/红=未覆盖）
  并存 `place_overlay_save_dir`（默认 `/tmp`）。

专项子模式（与正常 7 步互斥，跑完即退）：
- `depth_offset_check:=true`：只触几点空发光板，与步骤 2 深度平面比偏移，存
  `DEPTH_TOUCH_Z_OFFSET` 后退出（`check_points` 控制点数）。
- `edit_cells:=true`：**单独微调白板放置格心**。交互选 `(col,row)`，用吸嘴逐格触碰，
  只覆盖 `BOARD_CENTERS_14x10_BOARD` 的 XY（存 `BOARD_CELL_OVERRIDES_BOARD`），**不重标
  整块网格**。只改 XY——格的 z（凸起高度）、放置深度 `PLACE_Z_MAP` 不变。需已有步骤 2/3
  的坐标系与网格；改完即退出。适合个别格放偏、又不想重跑整块网格标定的场合。

  ```bash
  # 进入逐格微调模式（不跑 7 步标定）
  roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216 edit_cells:=true
  ```

  > 注意：`edit_cells` 写的是相对现有网格/坐标系的逐格覆盖；一旦重跑步骤 3（重派生网格+
  > 坐标系），这些逐格覆盖会被清空，需要重新微调。

- `verify_cells:=true`：**白板格心巡检（验证标定，不写配置）**。驱动机械臂依次悬停到各格
  中心【上方设定高度】，肉眼核对标定 XY 是否对准——**走与生产控制器同一条路径**（原生
  `/xarm/move_line` + `BOARD_POSE_BASE`），是"标定完上真机前"最直接的对准自检。需已有步骤
  2/3 结果。⚠️ 会自动驱动真机。

  ```bash
  # 巡检全部格心（悬停不下扎）
  roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216 verify_cells:=true
  ```

  参数：
  - `verify_cells_spec`（默认 `all`）：选巡检哪些格——`all` / `r<行>` / `c<列>` /
    `'col,row;...'`。
  - `verify_height`（默认 0.05，米）：每格上方悬停高度；巡检中输入 `h` 可临时改，回车走下
    一格。
  - `verify_z_ref`（默认 `bump`）：悬停高度的基准 Z——`bump`=格心凸起顶面，`place`=放置面
    `PLACE_Z_MAP`。
  - `verify_speed`（60）/ `verify_acc`（300）/ `verify_travel_clearance`（0.08，格间过渡抬升
    高度）。

---

## 3. 抓偏来源诊断 —— `pixel_touch_check.launch`

对比"像素 → base 预测"与"实际吸嘴尖触点"，定位抓偏/吸偏到底出在**感知**还是
**单应性/视差/执行**。一键起 **MoveIt 真机栈 + 相机 + 手眼 TF + `pixel_touch_check.py`**
（TF 链与 `calibrate_tool.launch` 保持一致）。**只诊断，不写 `tetris_config.yaml`**——结果
存到 `/tmp`。

它同时跑两条预测路径，各落在不同 base 系、各用一把同系触点尺子比对：
- **深度+手眼**：像素→深度反投影→相机系→`TF(base←camera)`（URDF/TF 系，感知粗核对）。
- **单应性+视差**：复刻 `xarm_controller` 的抓取 XY（`PICK_HOMOGRAPHY` + `BOARD_POSE_BASE`）；
  这套标定由 `calibrate_board` 用**固件位姿**记录、`move_line` 也走固件系，故预测落在
  **固件系**（生产真正用的路径）。

流程：彩色图上手点 N 个物理点 → 两条各算一个 base 预测 → 手动移吸嘴尖(`link_tcp`)到同一
点回车，同时记 ROS TF 触点（URDF 系，比深度+手眼）与固件 `/xarm/xarm_states.pose` 触点
（固件系，比单应性）→ 打印/保存两条误差 + URDF↔固件系统偏差。

```bash
roslaunch tly pixel_touch_check.launch robot_ip:=192.168.1.216
```

参数：
- `num_points`（默认 4）：手点物理点数。
- `compare_homography`（默认 true）：除"深度+手眼"外，再走"单应性(+视差)"路径对照。
- `use_parallax`（默认 true）：单应性路径是否叠加视差修正（与控制器一致）。
- `use_firmware_pose`（默认 true）：单应性对照的触点用固件 `xarm_states.pose` 记录（与
  `calibrate_board`/`move_line` 同系，消除 URDF↔固件运动学 ~4-5mm 系统差）；`false` 退回
  ROS TF。"深度+手眼"那条恒用 TF。
- `capture_frames`（50）/ `sample_win_half`（4）/ `sample_min_valid`（8）：深度多帧中值采样。
- `save_path`：诊断结果保存路径（空=`/tmp` 下默认名）。

---

## 4. TCP 姿态角速度标定 —— `measure_tcp_omega.launch`

测 `move_line` 的 TCP 姿态角速度 ω，换算成 `path_planner` 代价直接吃的**比值**
`ω/v_lin`（`omega_per_v_lin_rad_per_m`，rad/m），即"多少 rad 转动 ≈ 1m 平移"的时间当量。
一键起 **xArm 原生驱动 + `measure_tcp_omega.py`**，原地纯转姿态、多角度计时，用**斜率法**
消掉加减速斜坡与固定延迟后求 ω。**只测量、打印建议值，不写任何配置**——把结果手填进
`tly.launch` 的 `omega_per_v_lin_rad_per_m`（当前 3.23）。

原理：`move_line` 默认立刻返回（`/xarm/wait_for_finish=false`），故靠固件上报的
`/xarm/xarm_states` state 边沿（1 RUNNING→2 SLEEPING）+ `header.stamp` 计时，抗客户端排队
抖动；对多个角度做 `ω ≈ (θ2−θ1)/(T2−T1)`。ω 随 `mvvelo` 线性变，**比值与档位无关**（理论
固件映射 ≈π）——换个 `mvvelo` 复测比值应稳定即确认。

```bash
roslaunch tly measure_tcp_omega.launch robot_ip:=192.168.1.216 mvvelo:=200
# 实测 ω≈0.645@200mm/s → 比值≈3.23≈π
```

> ⚠️ **真机脚本，会驱动腕部原地旋转**：运行前在 UF Studio 把 J6/J4/J1 摇到中段，避免大角度
> 旋转把腕部推过 ±180°/限位；确认工作区无人、急停可达。

参数：
- `mvvelo`（默认 200，mm/s）：测生产实际档位；比值与它无关，任一档位测得的比值都可直接用。
- `mvacc`（默认 1000，mm/s²）。
- `axis`（默认 `yaw`，可 `roll`/`pitch`）：转哪个 RPY 分量（Δθ 用四元数测地夹角算，与选哪
  轴无关）。
- `angles_deg`（默认 `[20,60,100,140]`）：多角度采样点，斜率法至少要 2 个有效角。
- `repeats`（默认 3）：每角度重复次数取均值。

> 重测时机：改了 TCP 或关节的加速度/加加速度(jerk)后，加减速斜坡/角速度上限可能变，重测本项。

---

## 快速参考

```bash
# 0) 相机内参自检（可选）
roslaunch tly verify_camera.launch mode:=report,points known_distance_m:=0.30

# 1) 手眼标定（自动采样，推荐）
roslaunch tly xarm_calibration_setup_moveit.launch robot_ip:=192.168.1.216
#    或手动 freehand：
roslaunch tly xarm_calibration_setup.launch robot_ip:=192.168.1.216

# 2) 白板标定（全 7 步）
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216
#    只重跑部分：
roslaunch tly calibrate_tool.launch robot_ip:=192.168.1.216 steps:=4,5

# 3) 抓偏来源诊断（感知 vs 单应性/视差/执行）
roslaunch tly pixel_touch_check.launch robot_ip:=192.168.1.216

# 4) TCP 姿态角速度标定 → path_planner 的 omega_per_v_lin_rad_per_m
roslaunch tly measure_tcp_omega.launch robot_ip:=192.168.1.216 mvvelo:=200
```
