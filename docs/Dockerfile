# =============================================================================
# xArm6 俄罗斯方块工作区 —— 环境镜像 (environment image)
#
# 用法约定（与 .devcontainer/devcontainer.json 一致）：
#   本镜像只提供"环境"（ROS + 各类依赖 + 工具链），**不**把工作区源码烤进镜像、
#   也**不**在构建期 catkin_make。真正的工作区通过 bind mount 挂到
#   /root/catkin_ws，进容器后在挂载目录里执行 catkin_make。
#   —— 所以：镜像里放什么到 /root/catkin_ws 都会被 bind mount 遮住，别在这里 COPY 源码。
#
# 复现自当前在用的运行环境（探测得到）：
#   Ubuntu 20.04 / CUDA 11.8 + cuDNN 8.9.7 / ROS Noetic desktop /
#   apt 版 OpenCV 4.2.0（注意：无 CUDA 模块，DexiNed 经 cv2.dnn 跑）/
#   librealsense2 (intel apt 源) / moveit 全家桶 / easy_handeye 依赖 (aruco + visp)
#
# 构建：
#   docker build -f docs/Dockerfile -t xarm_tly:latest .
# 运行（Linux，接真机 + GPU + 相机 + X11）：
#   docker run -it --gpus all --net=host --privileged \
#       --device=/dev/bus/usb \
#       -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY \
#       -v $PWD:/root/catkin_ws xarm_tly:latest
#   进容器后：catkin_make && source devel/setup.bash
#
# 重要：第三方源码包 xarm_ros / realsense-ros / easy_handeye 已被 .gitignore 排除，
#       必须存在于你本地挂载进来的 src/ 里（本镜像不 clone、也无法透过 bind mount 提供）。
# =============================================================================

# CUDA 11.8 + cuDNN 8 + Ubuntu 20.04（Noetic 要求 focal）；devel 变体带 nvcc/头文件
ARG CUDA_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${CUDA_IMAGE}

# 全程非交互，且 RUN 用 bash（后面要 source）
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic
SHELL ["/bin/bash", "-c"]

# NVIDIA 运行时能力（GPU + OpenGL/GLX，供 rviz、DexiNed dnn 用）
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1

# -----------------------------------------------------------------------------
# 1) 基础系统工具 + apt 源准备（ROS、RealSense）
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl gnupg2 lsb-release ca-certificates \
        software-properties-common usbutils sudo vim \
    && rm -rf /var/lib/apt/lists/*

# ROS Noetic apt 源
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros1.list \
    && curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && sed -i '1s|^deb |deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] |' \
        /etc/apt/sources.list.d/ros1.list

# Intel RealSense apt 源（librealsense2 SDK）
RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
        | tee /etc/apt/keyrings/librealsense.pgp > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo focal main" \
        > /etc/apt/sources.list.d/librealsense.list

# -----------------------------------------------------------------------------
# 2) ROS Noetic + 项目所需的全部 ROS 包
#    （env image：src 在运行期才挂进来，故不能 rosdep，这里显式列全依赖）
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        # --- ROS 桌面版（roscpp/rospy/rviz/tf/rqt/image_transport 等） ---
        ros-noetic-desktop \
        # --- MoveIt 全家桶 + 运动规划（tly 编译期依赖 moveit_ros_planning_interface/core） ---
        ros-noetic-moveit \
        ros-noetic-moveit-visual-tools \
        ros-noetic-rviz-visual-tools \
        ros-noetic-pilz-industrial-motion-planner \
        # --- ros_control / controllers（xarm_ros 驱动 + 轨迹控制器） ---
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-combined-robot-hw \
        # --- 相机图像处理链（image_proc 校正、标定工具） ---
        ros-noetic-image-proc \
        ros-noetic-image-view \
        ros-noetic-camera-calibration \
        ros-noetic-cv-bridge \
        ros-noetic-vision-opencv \
        # --- 手眼标定 easy_handeye 依赖（aruco 检测 + visp 求解） ---
        ros-noetic-aruco-ros \
        ros-noetic-visp-hand2eye-calibration \
        ros-noetic-visp-bridge \
        # --- realsense-ros wrapper 依赖 ---
        ros-noetic-ddynamic-reconfigure \
        ros-noetic-diagnostic-updater \
        # --- 杂项 ---
        ros-noetic-rosparam-shortcuts \
        ros-noetic-warehouse-ros \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-xacro \
        # --- OpenCV（apt 4.2.0：C++ 开发库 + Python 绑定；无 CUDA，与现网一致） ---
        libopencv-dev python3-opencv \
        # --- RealSense SDK ---
        librealsense2-dev librealsense2-utils librealsense2-gl librealsense2-udev-rules \
        # --- catkin 构建工具链 & Python ---
        python3-catkin-tools python3-rosdep python3-osrf-pycommon \
        python3-pip python3-numpy python3-scipy python3-yaml python3-empy \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 3) Python 额外依赖（apt 里没有的用 pip；transforms3d 供标定脚本用）
# -----------------------------------------------------------------------------
RUN pip3 install --no-cache-dir transforms3d

# -----------------------------------------------------------------------------
# 4) rosdep 初始化（运行期若挂进新 src，可 rosdep install 补依赖）
# -----------------------------------------------------------------------------
RUN rosdep init && rosdep update --rosdistro ${ROS_DISTRO}

# -----------------------------------------------------------------------------
# 5) 登录 shell 自动 source ROS 与工作区（若已 catkin_make）
# -----------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/catkin_ws/devel/setup.bash ] && source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws

# 每次进容器/exec 都先把 ROS 环境 source 好，再执行命令
# （用 printf 生成，避免依赖 BuildKit 的 heredoc；\$@ / \$ 转义留给运行期展开）
RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
        'if [ -f /root/catkin_ws/devel/setup.bash ]; then' \
        '    source /root/catkin_ws/devel/setup.bash' \
        'fi' \
        'exec "$@"' \
        > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
