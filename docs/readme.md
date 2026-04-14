不保证下述节点完全可用，请根据实际修改，使用xarm_controller_node时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
不保证下述节点完全可用，请根据实际修改，使用xarm_controller_node时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！
不保证下述节点完全可用，请根据实际修改，使用xarm_controller_node时请一定确保控制器在身边，以便及时按下急停按钮！！！！！！！

启动相关节点：
```bash
roslaunch xarm_bringup xarm6_server.launch robot_ip:=192.168.1.228
roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.228
roslaunch realsense2_camera rs_camera.launch align_depth:=true
roslaunch xarm_description xarm6_upload.launch
rosrun tly vision_processor_node
rosrun tly tetris_strategy_node
rosrun tly xarm_controller_node
rosrun tf2_ros static_transform_publisher 0.0481108 -0.0242403 0.0212136 0.0215310 0.0272684 0.7157493 0.6974924 link6 camera_color_optical_frame
```

注意：rosrun tf2_ros static_transform_publisher的参数来源于/root/.ros/easy_handeye/xarm6_realsense_calibration_eye_on_hand.yaml
不保证上述节点完全可用，请根据实际修改
[xarm_controller_node](../src/tly/src/xarm_controller_node.cpp)里的参数部分请根据实际修改
[calibrate_boadr.py](../src/tly/scripts/calibrate_board.py)为标定白板各个格子的坐标，按照左上, 右上, 右下, 左下顺序点击白板的四个角落的格子，将其结果填入[vision_processor_node.py](../src/tly/scripts/vision_processor_node.py)的BOARD_GRID_POINTS
[hsv_tuner.py](../src/tly/scripts/hsv_tuner.py)为调节各个方块的hsv阈值，将其结果填入[vision_processor_node.py](../src/tly/scripts/vision_processor_node.py)的color_ranges
[test_aruco.py](../src/tly/scripts/test_aruco.py)为测试是否识别到ArUco码，需在启动相机节点后运行（参照[src/tly/docs/calibrate_readme.md](calibrate_readme.md)）


若在Windows上使用docker容器，请参考：https://learn.microsoft.com/en-us/windows/wsl/install，安装wsl，并参考：https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers，安装docker

参考[text](wsl_usb.md)使用usbipd_win来挂载usb设备，不要将wsl的网络模式设置为mirror，可能会连不上机械臂，请设置为nat

在 Windows 上导入镜像

确保你的 Windows 上已经启动了 Docker Desktop。

打开 Windows 的命令提示符（CMD）或 PowerShell，并导航到存放 .tar 文件的目录：

```powerShell
# 假设你的镜像压缩包在C:\Users\你的用户名\Desktop
cd C:\Users\你的用户名\Desktop 
```
使用 docker load 命令导入镜像：

```powerShell
# 语法：docker load -i <文件名.tar>
docker load -i xarm_tly.tar
```
导入完成后，你可以运行 
```powershell
docker images
```
命令，就能看到你的镜像已经成功转移到 Windows 的 Docker 中了。

之后请将catkin_ws解压后移入WSL 2 的文件系统中，而不能放在 Windows 的 C 盘或 D 盘。 打开 WSL 2 (Ubuntu) 的终端，在 Linux 目录下（例如 ~/catkin_ws）执行 code . 来启动 VS Code，这样 VS Code 才能正确读取到 WSLg 生成的 ${localEnv:DISPLAY} 等环境变量。

vscode安装相关插件：WSL, Dev Containers, Docker

在Windows上使用[text](../.devcontainer/devcontainer.json)生成容器，请将[text](../.devcontainer/devcontainer.json)修改为下面的内容：
```text
{
    "name": "xArm-Full-Workspace",
    "image": "xarm_tly:04042144",
    "workspaceFolder": "/root/catkin_ws",
    "workspaceMount": "source=${localWorkspaceFolder},target=/root/catkin_ws,type=bind",
    "runArgs": [
        "--gpus", "all",
        "--net=host",
        "--privileged",
        "--device=/dev/bus/usb",
        
        // 保留 X11 挂载
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
        // 【新增】挂载 WSLg 的核心套接字目录
        "-v", "/mnt/wslg:/mnt/wslg" 
    ],

    "containerEnv": {
        // 【修改】继承 WSL2 环境中的显示变量
        "DISPLAY": "${localEnv:DISPLAY}", 
        "WAYLAND_DISPLAY": "${localEnv:WAYLAND_DISPLAY}", 
        "XDG_RUNTIME_DIR": "/mnt/wslg/runtime-dir", 

        "NVIDIA_VISIBLE_DEVICES": "all",
        "NVIDIA_DRIVER_CAPABILITIES": "all",
        "QT_X11_NO_MITSHM": "1"
    },

    "postCreateCommand": "source /opt/ros/noetic/setup.bash && source devel/setup.bash",

    "customizations": {
        "vscode": {
            // 同样记得把 plugins 改为 extensions
            "extensions": [
                "ms-iot.vscode-ros",
                "ms-vscode.cpptools",
                "twxs.cmake"
            ]
        }
    }
}
```
之后在vscode里请按住 Ctrl + shift + P，搜索Dev Containers: Rebuild and Reopen Containers生成容器

编译文件：
```bash
catkin_make
```
