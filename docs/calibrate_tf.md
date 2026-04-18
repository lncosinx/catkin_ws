由于某些原因，机器人基坐标系实际使用时并不如我们所期望的那样，所以需要发布相应的TF补偿

启动相机：
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

启动带 MoveIt 的真实机械臂控制环境：
```bash
roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.228
```

TF补偿及相关参数设置，按照要求操作，结果保存在[config文件夹](../src/tly/config)：
```bash
rosrun tly calibration_tool.py
```
