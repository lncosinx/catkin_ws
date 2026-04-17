本地终端：
```bash
xhost +local:docker
```

启动realsense-viewer:
```bash
realsense-viewer
```
调整相机参数并保存图片至[images](../src/tly/images),然后关闭

启动相机发布节点：
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
启动白色底板凸起标定程序，[calibrate_boadr.py](../src/tly/scripts/calibrate_board.py)为标定白板各个格子的坐标，按照左上, 右上, 右下, 左下顺序点击白板的四个角落的格子，将其结果填入[vision_processor_node.py](../src/tly/scripts/vision_processor_node.py)的BOARD_GRID_POINTS：
```bash
cd src/tly/scripts
python3 calibrate_board
```
将得到的坐标列表复制到[vision_processor_node.py](../src/tly/scripts/vision_processor_node.py)的board_grid_points

启动hsv调试程序：
```bash
python3 hsv_tuner.py
```
将调好的阈值一一复制到[vision_processor_node.py](../src/tly/scripts/vision_processor_node.py)

启动视觉处理程序：
```bash
python3 vision_processor_node.py
```

启动rviz，并添加images窗口，订阅/vision/debug_image观察效果
```bash
rviz
```