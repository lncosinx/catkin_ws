# 视觉模块说明

本文档介绍 `tly` 的视觉感知子系统：它在背光板上识别黑色/彩色的多联骨牌
（俄罗斯方块），分类形状、估计朝向、确定抓取点，并把库存与逐块信息发布给
策略节点。

参考文件：
- 启动：[test_vision.launch](../src/tly/launch/test_vision.launch)
- 经典实现：[vision_processor_node.cpp](../src/tly/src/vision_processor_node.cpp)
- DexiNed 实现：[vision_processor_node_dexined.cpp](../src/tly/src/vision_processor_node_dexined.cpp)

---

## 1. 两套实现，一份契约

视觉节点有两个可互换的 C++ 实现，**话题 / 消息 / 服务契约完全一致**，只在
“如何从图像里分割出方块前景”这一步不同：

| 节点（`type=`）                  | 分割方式                                | 何时用 |
|----------------------------------|-----------------------------------------|--------|
| `vision_processor_node_cpp`      | 经典：灰度阈值（OTSU/自适应/手动）+ 形态学 | 纯黑块、对比度好、无 GPU |
| `vision_processor_node_dexined`  | DexiNed ONNX（CUDA）边缘 + Lab 颜色边界切割 | 彩色块、贴碰块多、有 GPU（生产默认） |

在 launch 里用 `vision_node:=...` 切换：

```bash
roslaunch tly test_vision.launch vision_node:=vision_processor_node_dexined
```

形状 ID 是跨节点共享的硬契约（见 `SHAPE_NAMES` / `BASE_SHAPES`）：
`0` 一字、`1` 田、`2` T、`3` L_left、`4` L_right、`5` Z_left、`6` Z_right。
改这个枚举必须同步 `strategy_node.cpp`、Python 视觉节点等所有使用方。

---

## 2. 数据流水线

两个实现的整体管线相同，差别仅在步骤 2 的“前景分割”：

```
彩色图(image_raw/rect) ──▶ [1 预处理] ──▶ [2 前景分割] ──▶ [3 轮廓提取]
                                                                  │
                       ┌──────────────────────────────────────────┘
                       ▼
   [4 模板匹配分类] ──▶ [5 抓取点+朝向] ──▶ [6 跨帧跟踪/稳定] ──▶ [7 发布]
                                              │
                          对齐深度采样抓取点 Z ┘（仅 debug 通路）
```

### 步骤 1 — 预处理
- 可选缩放 `scale_percent`，按 `roi_*` 裁出 ROI（避开画面边缘干扰）。
- 转灰度 + 高斯模糊（`blur_kernel`）。
- **发光板掩码**（`use_lightboard_mask`）：对灰度做 OTSU 取最大连通域，得到背光板
  区域；后续前景与之相交，挡掉板外的反光/手/边框。`lightboard_*` 控制形态学与
  最小面积比。
- DexiNed 版还可选 **输入帧时间平均**（`temporal_alpha`，EMA）：静止料堆下抑制
  传感器噪声与背光闪烁，越小越稳但越滞后，0=关闭。

### 步骤 2 — 前景分割（两版分歧点）

**经典版**（`vision_processor_node.cpp`，`process_foreground`）：
1. 灰度阈值取暗色前景：`threshold_mode` = `otsu` / `adaptive` / 手动
   (`manual_dark_threshold`)。
2. 可选 **饱和度前景**（`use_saturation_foreground`）：用 HSV 的 S/V 抓彩色块
   （含偏亮的红/黄），并到暗阈值结果上，避免灰度暗阈值漏掉彩色块。
3. 与发光板掩码相交 → 形态学闭/开（`close_kernel` / `open_kernel`）→ Canny 出边缘。

**DexiNed 版**（`vision_processor_node_dexined.cpp`）：
1. 初始化时把 `dexined.onnx` 强制加载到 **CUDA** 后端（失败则回退经典分支）。
2. 每帧把 ROI（pad 到 16 的倍数、扣 ImageNet 均值）送 `dexined_net_.forward()`，
   得边缘热力图并归一化到 0–255。
3. 前景 = 灰度 OTSU 暗块 ∪ HSV 高饱和彩色块。
4. **切割贴碰块**的核心是 **Lab 颜色边界**：对 Lab 的 a、b 通道做形态学梯度取
   `max` 作为切割线（`color_edge_thresh`）。块内均匀色无 a/b 梯度→不内切；眩光只
   改亮度 L→对 a/b 免疫；异色块边界色度突变→切开。
5. 可选 `use_dexined_for_split` 把高阈值 DexiNed 强边或上来，专补**同色**贴碰块
   （颜色无边界处）。
6. `color_edge_min_area` 按连通域面积滤掉掉漆/划痕产生的零碎短边，避免把单块切碎。
7. 前景减去（加粗后的）颜色边界 → 得到彼此分开的前景掩码。
   `debug_edges` 显示的就是这条颜色边界。

> 注意：DexiNed 版的 `dexined_thresh` 现在只参与“同色切割”与 debug，主切割已改走
> 颜色边界；纯黑白竞赛场景可设 `use_dexined_for_split:=false` 走纯颜色（最稳）。

### 步骤 3 — 轮廓提取与过滤
对前景掩码 `findContours`，按面积（`min_area` / `max_area`）、bbox 填充率
（`min/max_contour_fill_ratio`）、最小宽高过滤掉噪声与粘连大 blob。

### 步骤 4 — 模板匹配分类（`TemplateBank`）
- 启动时为 7 个形状 × 360° 预渲染二值模板（`template_size`，
  `template_angle_step` 粗扫 + `template_refine_step` 细化）。
- 每个候选轮廓归一化后与模板库匹配，综合多项打分：硬/软 IoU、Chamfer 距离、
  4×4 与 8×8 网格占用签名；L 形额外用逐格占用模式分（`l_cell_pattern_score`）。
- 返回 `(shape_id, angle, score)`；`score < min_template_iou` 的候选丢弃。
- 匹配前可用 `poly_approx_eps_ratio` 对轮廓做多边形近似，拉直分割留下的波纹边，
  提高分类准度（DexiNed 版）。

### 步骤 5 — 抓取点与朝向
- **抓取点**（`pick_point_mode`）：`centroid` 用质心；`distance` 用距离变换的内点
  （最大内切圆心，远离边缘，吸盘更稳）；`hybrid` 仅对 `distance_pick_shapes`
  （默认 L_left/L_right）用距离变换、其余用质心。
- **朝向**：模板角给出基础朝向；可选 `use_contour_axis_yaw` 用轮廓主轴
  （minAreaRect / Hough 直线）细化，并按形状做**有向**判定（T/L/Z 各有专门的
  `directed_*_axis_angle` 函数定出 0–360° 的唯一方向）。
- `publish_angle_in_table_frame` 控制把图像角投影成桌面/board 系偏航再发布（需要
  相机→`table_frame` 的 TF；该 TF 已弃用，默认 `link_base`，仅 debug/PoseArray 用，
  `board_state` 不依赖）。

### 步骤 5.5 — 同帧 NMS（仅 DexiNed 版）
分割过切可能让一个块出多个框。按分数降序贪心，质心距小于
`nms_center_dist_px` 的视为同块、只留高分。

### 步骤 6 — 跨帧跟踪与稳定（`BlockTrack`）
- 用像素/桌面坐标 + 形状把检测关联到历史轨迹（`track_match_gate_px/m`），
  保留 `track_history_len` 帧历史。
- 形状按周期对角度取循环均值（田 90°、一字/Z 180°、其余 360°）。
- 仅当轨迹**稳定**才发布（`publish_only_stable`）：帧数 ≥ `stable_min_frames`、
  位置抖动 ≤ `stable_max_px_std`、角度抖动 ≤ `stable_max_angle_std_deg`。
- `stable_publish_max_missed` 允许已稳定的边缘块偶尔漏检仍保持发布，稳住库存计数；
  `max_missed_frames` 后彻底删除轨迹。

### 步骤 7 — 深度抓取 Z（debug 通路）
`use_depth_pick_z` 开启时，订阅对齐深度
（`/camera/aligned_depth_to_color/image_raw`），在抓取点采样相机系 Z（米），
半径 `depth_sample_radius_px`。检测像素是 rect 坐标、对齐深度在 raw 彩色坐标，
采样前用 `rectified_to_raw` 映射回去（`depth_sample_in_raw_color`）。
目前仅打 log + 发 `/vision/pick_depth_debug`，**尚未写入 board_state**。

---

## 3. 输出话题与契约

| 话题 | 类型 | 内容 |
|------|------|------|
| `/vision/board_state` | `Int32MultiArray` | **主输出**，见下 |
| `/vision/tracked_blocks_table` | `PoseArray` | 稳定块在 `table_frame` 的位姿（debug） |
| `/vision/debug_image` | `Image` | 叠加轮廓/抓取点/标签的可视化 |
| `/vision/debug_foreground` | `Image` | 前景掩码 |
| `/vision/debug_edges` | `Image` | 边缘 / 颜色边界（DexiNed 版） |
| `/vision/preprocess/*` | `Image` | rgb / gray / gaussian_blur / otsu_binary / morphology |
| `/vision/pick_depth_debug` | `Float32MultiArray` | 每块 `[shape, u, v, z_m, valid]` |

`/vision/board_state` 布局（`strategy_node` 要求 `size() >= 147` 再读其余）：
```
[0..6]    7 个形状的库存计数
[7..146]  140 格棋盘占用栅格（14×10，视觉端当前置 0，占位）
[147]     num_blocks
[148..]   每块 6 元组: [shape_id, pick_u, pick_v, angle_deg, geom_u, geom_v]
```

> 当前没有节点调用视觉服务 `/vision/get_precise_pose`，它仅供未来/手动使用。

---

## 4. 单独运行视觉（test_vision.launch）

`test_vision.launch` **只起视觉链路**：RealSense 相机 + 视觉节点，不含臂 / TF /
策略 / 控制，便于单独调参。

```bash
roslaunch tly test_vision.launch                       # 默认 DexiNed
roslaunch tly test_vision.launch vision_node:=vision_processor_node_cpp
# 命令行可覆盖调参，对照 debug 图边看边调：
roslaunch tly test_vision.launch color_edge_thresh:=16 min_template_iou:=0.6
```

launch 要点：
- 带 `align_depth:=true` 起相机，供抓取点 Z 采样。
- 启动 6s 后用 `dynparam` 加载 `config/camera_config.yaml` 的固定曝光/白平衡，
  避免每次手调、消除自动曝光带来的检测抖动。
- 默认 `image_topic=/camera/color/image_raw` + `assume_image_rectified:=true`
  （免单独跑 `image_proc`）。

常用调参对照（边看对应 debug 话题边调）：
- 块没切开（粘连）→ 调小 `color_edge_thresh`（12~16）；块被切碎 → 调大（28~35）。
- 掉漆零碎边还在切 → 调大 `color_edge_min_area`。
- 粘连大 blob 误分类 → 调高 `min_template_iou`。
- 彩色块漏检 → 确认 `use_saturation_foreground:=true`。
- 静止料堆抖动/背光闪 → 调小 `temporal_alpha`（更稳更滞后）。

---

## 5. 相机参数：实时调整 / 保存 / 加载

视觉检测对**曝光与白平衡**很敏感：开自动曝光时背光板亮度会浮动，检测会抖。所以本
项目把相机参数固定下来存进 [config/camera_config.yaml](../src/tly/config/camera_config.yaml)，
`test_vision.launch` 启动时自动加载。相机参数的 dynamic_reconfigure 命名空间是
**`/camera/rgb_camera`**（深度/红外在 `stereo_module` 等其它分支，别选错）。

### 5.1 实时调整（rqt_reconfigure）

先起带相机的 launch（如 `test_vision.launch`），再开 GUI：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

- 左侧树展开 `camera` → 选 **`rgb_camera`**，右侧拖滑块/改数值即**实时生效**，
  配合 `/vision/debug_*` 图边看边调。
- 关键项（对应 yaml）：
  - `enable_auto_exposure` / `exposure` —— **务必先关自动曝光**再手调 `exposure`，
    否则背光亮度浮动让检测抖。
  - `enable_auto_white_balance` / `white_balance` —— 关自动白平衡、固定色温
    （彩色块识别尤其重要）。
  - `gain`、`gamma`、`brightness`、`contrast`、`saturation`、`sharpness`、
    `power_line_frequency`（防工频闪，国内取 `3`=50Hz）。

### 5.2 保存参数

调满意后用 `dynparam dump` 导出当前命名空间的全部参数，**覆盖**仓库里的 yaml：

```bash
rosrun dynamic_reconfigure dynparam dump /camera/rgb_camera \
  $(rospack find tly)/config/camera_config.yaml
```

> 统一用 `dynparam dump` 保存，别用 rqt_reconfigure 自带的 “Save” 按钮——两者格式
> 略有差异，而 launch 是用 `dynparam load` 读取的。`dump` 出来的就是
> `camera_config.yaml` 那种结构。

### 5.3 加载参数

手动加载到运行中的相机：

```bash
rosrun dynamic_reconfigure dynparam load /camera/rgb_camera \
  $(rospack find tly)/config/camera_config.yaml
```

而 `test_vision.launch` / `tly.launch` 里**已自动加载**——相机起来 6 秒后执行：

```xml
<node pkg="dynamic_reconfigure" type="dynparam" name="load_rgb_cfg"
      args="load /camera/rgb_camera $(find tly)/config/camera_config.yaml"
      launch-prefix="bash -c 'sleep 6; exec $0 $@'" />
```

`sleep 6` 是等相机的 dynamic_reconfigure server 注册完再灌参，避免抢跑加载失败。
所以正常跑这些 launch 时无需手动 load，开机即恢复到调好的固定曝光/白平衡。

---

## 6. 注意事项

- 只有 `vision_processor_node_cpp`（由 `vision_processor_node.cpp` 编译）与
  `vision_processor_node_dexined` 在 `CMakeLists.txt` 里被构建。生产
  （`tly.launch`）默认用 DexiNed 版。
- `scripts/vision_processor_node.py` 是同契约的 Python 重实现，仅供
  `affine.launch` 标定使用，不进生产。
- `board_state` 的 140 格棋盘占用由视觉端置 0 占位；“某格是否已摆块”的真正判定在
  下游用逐格白板凸起高度完成，不由视觉负责。
