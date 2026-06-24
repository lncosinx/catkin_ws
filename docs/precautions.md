本地终端：
```bash
xhost +local:docker
```

启动realsense-viewer:
```bash
realsense-viewer
```
一定要设置最大速度与最大加速度，防止将相机数据线折断
电磁阀控制为CO0，对应/xarm/set_controller_dout，io_num=1.