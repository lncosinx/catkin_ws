使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！


下载DexiNed模型（.onnx格式）：
```text
https://huggingface.co/opencv/edge_detection_dexined
```

将下载好的模型放到/root/catkin_ws/src/tly/module

下载opencv源码：
```text
https://github.com/opencv/opencv/releases
https://github.com/opencv/opencv_contrib/tags
```
解压到
编译opencv：
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

本地终端：
```bash
xhost +local:docker
```

相机内参：
```bash
rostopic echo /camera/color/camera_info
```

在输出的信息中找到 K 矩阵，它包含 9 个数字：[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]。

真实坐标：
```bash
rosrun tf tf_echo link_base link_eef
```

启动：
```bash
roslaunch tly tly.launch
```

测试时可能用到的命令：
```bash
roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.228
roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.228
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch xarm_description xarm6_upload.launch
rosrun tly vision_processor_node.py
rosrun tly single_block_test.py
rosrun tly strategy_node
rosrun tly xarm_controller_node
roslaunch easy_handeye publish.launch eye_on_hand:=true namespace_prefix:=xarm6_realsense_calibration
```