# xArm6 俄罗斯方块自动装箱系统

> ⚠️ **使用时请一定确保控制器（示教器）在身边，以便及时按下急停按钮！！！**
> ⚠️ **使用时请一定确保控制器（示教器）在身边，以便及时按下急停按钮！！！**
> ⚠️ **使用时请一定确保控制器（示教器）在身边，以便及时按下急停按钮！！！**

## 项目简介

一个 **ROS 1 (Noetic)** catkin 工作区，驱动一台 **xArm6** 机械臂完成一个精确覆盖装箱任务：
用 **RealSense** 相机识别背光板上的彩色多联骨牌（"俄罗斯方块"），再用**真空吸盘**把它们
抓取并放入固定的 **14×10** 网格，目标是放进尽可能多的方块。

物理叠放关系（务必记清）：**白板（14×10 放置网格，带 140 个 2~3mm 凸起）叠在发光板
（背光板，抓取/散料源区）上面**。标定里 `board_frame` 的 z=0 平面在**发光板**上，白板
表面/凸起在 board 系 z 为正。详见 [标定指南](calibrate.md)。

活跃开发都在 `src/tly`；`src/` 下其余（`xarm_ros`、`realsense-ros`、`vision_opencv`、
`easy_handeye`）是引入的第三方依赖，当作已安装的包对待。

## 硬件与环境

| 项 | 说明 |
|---|---|
| 机械臂 | UFACTORY xArm6（末端真空吸盘，数字 IO 控制电磁阀，CO0 / `io_num=1`）|
| 相机 | Intel RealSense（eye-on-hand，装在末端）|
| 工作区 | 背光板（抓取源区）+ 叠在其上的 14×10 白板（放置网格）|
| 系统 | Ubuntu 20.04 / ROS Noetic / CUDA 11.8 + cuDNN 8（容器内）|

## 快速开始

### 1. 准备环境（Docker）

- **Linux**：用本仓库的 [Dockerfile](Dockerfile) 构建环境镜像，再绑定挂载工作区：
  ```bash
  docker build -f docs/Dockerfile -t xarm_tly:latest .
  docker run -it --gpus all --net=host --privileged \
      --device=/dev/bus/usb \
      -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
      -v $PWD:/root/catkin_ws xarm_tly:latest
  ```
- **Windows / WSL2（不推荐）**：见 [docker_setup.md](docker_setup.md)（导入 `.tar` 镜像 + devcontainer）
  与 [wsl_usb.md](wsl_usb.md)（用 usbipd-win 挂载 USB，网络设 **nat** 不要 mirror）。
- 第三方源码包 `xarm_ros` / `realsense-ros` / `easy_handeye` 已被 `.gitignore` 排除，
  **必须存在于挂载进来的 `src/` 里**（镜像不含它们）。
```bash
cd catkin_ws
git clone -b ros1-legacy https://github.com/realsenseai/realsense-ros.git 
git clone https://github.com/xArm-Developer/xarm_ros.git
git clone https://github.com/IFL-CAMP/easy_handeye.git
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
```

### 2. 下载 DexiNed 边缘检测模型

生产默认视觉节点是 DexiNed（`vision_processor_node_dexined`），需要 ONNX 模型：

- 下载：<https://huggingface.co/opencv/edge_detection_dexined>
- 放到：`src/tly/module/dexined.onnx`

### 3. 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make            # 在 /root/catkin_ws 下执行 —— 唯一在用的构建流程
source devel/setup.bash
```

> `src/tly/CMakeLists.txt` 强制 `-std=c++14` 和 `-O3`（`-O3` 是为 `strategy_node`
> 的 DLX 搜索加的，别去掉）。

### 4. 允许容器访问 X11（在宿主机终端）

```bash
xhost +local:docker
```

### 5. 标定

**首次上真机前必须标定**（手眼 + 白板 + 抓取单应性）。整套流程见
[标定指南](calibrate.md)。标定结果写入 `src/tly/config/tetris_config.yaml`（当作数据，别手改）。

### 6. 运行完整流水线

```bash
roslaunch tly tly.launch robot_ip:=<机械臂IP>     # 默认 192.168.1.216
```

切换经典视觉实现（非神经网络边缘检测，无需 onnx 模型）：

```bash
roslaunch tly tly.launch robot_ip:=<机械臂IP> vision_node:=vision_processor_node_cpp
```

> 两个视觉节点各配一套 RealSense 曝光/色彩档（`dynparam` 灌进 `/camera/rgb_camera`）：
> DexiNed 用 `config/camera_config.yaml`（launch 默认加载），经典 `_cpp` 用
> `config/camer_config_1.yaml`（高对比、去饱和）。切到 `_cpp` 时需把 launch 里
> `load_rgb_cfg` 加载的相机档一并换成后者，详见 [vision.md](vision.md) §5.0。

## 节点流水线

四个自定义节点通过固定的话题契约串联：

| 顺序 | 节点 | 职责 | 文档 |
|---|---|---|---|
| 1 | `vision_processor_node_dexined` / `_cpp` | 分割/分类背光板上的方块，发布库存 + 占用栅格 + 逐块像素 | [vision.md](vision.md) |
| 2 | `strategy_node` | 把 14×10 放置当精确覆盖问题，用 DLX 搜最高分覆盖，发候选布局 | [strategy.md](strategy.md) |
| 3 | `path_planner_node` | 像素→base 抓取(含深度)、格子→base 放置、腕部翻转择优、关节路程优化 | [path_plan.md](path_plan.md) |
| 4 | `xarm_controller_node` | 纯执行器：用 xArm **原生服务**（非 MoveIt）逐任务执行抓—搬—放 | [control.md](control.md) |

`/vision/board_state` → `/tetris_plan(_candidates)` → `/motion_cmds` → 真机。

## 分节点测试 launch

只起某个节点需要的东西，省得跑整条 `tly.launch`：

| launch | 作用 | 是否驱动真机 |
|---|---|---|
| `test_vision.launch` | 仅视觉（相机 + image_proc + 视觉节点）| 否 |
| `test_strategy.launch` | 仅策略（喂假 `board_state`，看 `/tetris_plan`）| 否 |
| `test_path.launch` | 感知→策略→路径链路（需臂提供 TF，不命令运动）| 否 |
| `test_controller.launch` | 仅控制器 + 臂 + 相机（手动喂 `/tetris_plan`）| **是（限 1 任务）** |
| `test_pick.launch` | 较旧的 MoveIt 集成测试，一次抓放循环 | **是** |

## 常用命令

```bash
# 相机内参：输出里的 K 矩阵 = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
rostopic echo /camera/color/camera_info

# 末端真实位姿（base ← eef）
rosrun tf tf_echo link_base link_eef

# 手眼 TF 广播（eye-on-hand）
roslaunch easy_handeye publish.launch eye_on_hand:=true namespace_prefix:=xarm6_realsense_calibration

# 单独起各组件（调试用）
roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.216
roslaunch realsense2_camera rs_camera.launch align_depth:=true
rosrun tly vision_processor_node_dexined
rosrun tly strategy_node
rosrun tly path_planner_node
rosrun tly xarm_controller_node
```

## 文档索引

| 文档 | 内容 |
|---|---|
| [calibrate.md](calibrate.md) | 手眼 + 白板 + 抓取单应性标定全流程 |
| [vision.md](vision.md) | 视觉感知子系统（分割、分类、抓取点）|
| [strategy.md](strategy.md) | 策略节点（精确覆盖 / DLX 求解）|
| [path_plan.md](path_plan.md) | 路径规划节点（坐标解算、腕部翻转、路程优化）|
| [control.md](control.md) | 控制节点（纯执行器，xArm 原生服务）|
| [docker_setup.md](docker_setup.md) | Windows/WSL2 下导入镜像与 devcontainer |
| [wsl_usb.md](wsl_usb.md) | WSL2 用 usbipd-win 挂载 USB 设备 |
| [precautions.md](precautions.md) | **注意事项与踩坑汇总**（安全、关节/运动学、标定、坐标系、代码结构）|

## 可选：CUDA 加速的 OpenCV（进阶）

默认镜像用 apt 版 OpenCV 4.2（**无 CUDA**），DexiNed 经 `cv2.dnn` 走 CPU，可用但较慢。
若要用 GPU 加速 DexiNed，需从源码编译带 CUDA/cuDNN 的 OpenCV（+ contrib）：

- 源码：<https://github.com/opencv/opencv/releases> 与
  <https://github.com/opencv/opencv_contrib/tags>
- 解压后编译（`CUDA_ARCH_BIN` 按你的 GPU 计算能力调整）：

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D WITH_CUBLAS=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D ENABLE_FAST_MATH=1 \
      -D CUDA_FAST_MATH=1 \
      -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
      -D BUILD_opencv_world=ON \
      -D CUDA_ARCH_BIN=8.6,8.9 \
      ../opencv
```

## 安全注意事项

- **示教器急停常备**（见文首）。
- 相机数据线务必设**最大速度/加速度上限**，防止折断（见 [precautions.md](precautions.md)）。
- 上真机前确认已完成标定，且 J1/J4/J6 未绕在 ±360° 边界附近（详见 `CLAUDE.md`）。
