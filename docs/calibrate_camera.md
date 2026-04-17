本地终端：
```bash
xhost +local:docker
```

启动 D415 相机

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

打开网站：https://chev.me/arucogen/
Dictionary 选择Original ArUco
Marker ID 随便填一个
marker_id和_marker_size请根据实际修改，_marker_size单位为m
这里要把 D415 的彩色话题映射给 aruco_ros：
将[test_aruco.py](../src/tly/scripts/test_aruco.py)和[xarm_calibration_setup.launch](../src/tly/launch/xarm_calibration_setup.launch)修改为实际值

测试aruco:
```bash
cd src/tly/scripts
python3 test_aruco.py
```
如果有输出，则说明aruco识别没有问题

启动标定：
```bash
roslaunch tly xarm_calibration_setup.launch
```

启动后，你会看到弹出一个带有三个窗口的 rqt 界面（easy_handeye 面板）以及一个 Rviz 窗口。
开启 xArm 拖动示教：按下机械臂末端的解锁按钮，或者通过 xArm Studio/ROS service 开启牵引示教模式。
采点循环（重复 10-15 次）：
动作 A：用手拖动 xArm6，让相机在不同的角度、不同的距离对准发光板上的 ArUco 码。
动作 B：确认 Rviz 里能看到相机的画面，且 ArUco 码的坐标轴没有乱跳（说明识别稳定）。
动作 C：在 rqt_easy_handeye 界面中，点击 "Take Sample"（采集样本）。
黄金采点法则：
不要只平移！一定要疯狂改变姿态（Pitch, Yaw, Roll）。例如：从左上角斜视标定板、从正上方俯视标定板、从右下角仰视标定板。
姿态变化越大，最终解算的矩阵越准。

采满 10-15 个点后，点击面板上的 "Compute"。
观察界面下方输出的结果：
你会看到一个 $4\times4$ 的矩阵，以及 Translation (x, y, z) 和 Rotation (四元数)。
常识自检：拿一把尺子，粗略量一下你的 D415 镜头中心到 xArm6 法兰盘中心（link6）的物理距离。对比刚才算出来的 Translation (x, y, z)。如果物理距离是 5cm，算出来是 0.05m 左右，说明标定极其成功！
确认无误后，点击 "Save"。
结果会自动保存到 ~/.ros/easy_handeye/xarm6_realsense_calibration.yaml 中。
之后你只需要在你的机器人启动 launch 文件里，利用 easy_handeye 提供的 publish.launch 把这个 yaml 播出来，TF 树就彻底打通了！

Calibration saved to /root/.ros/easy_handeye/xarm6_realsense_calibration_eye_on_hand.yaml