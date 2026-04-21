使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
使用时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！


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