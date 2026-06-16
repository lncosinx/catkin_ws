cmake_minimum_required(VERSION 3.0.2)
project(tly)

# 强制使用 C++14 编译，支持最新的语言特性和 random 库
add_compile_options(-std=c++14)

# 开启 O3 最高级别优化！对于纯算力的 DLX 算法，这能让速度提升几十倍
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  xarm_msgs
  message_generation
)

# 寻找 OpenCV (确保包含 CUDA 模块)
# 建议修改为显式查找 dnn 模块
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

# /motion_cmds 消息：路径节点 -> 控制节点 (base 系抓放位姿队列)
add_message_files(
  FILES
  MotionTask.msg
  MotionPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs cv_bridge image_transport moveit_ros_planning_interface tf2 xarm_msgs message_runtime
)

# 定义包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/vision_processor_node.py
  scripts/table_tf_broadcaster.py
  scripts/single_block_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =========================================================
# 编译策略节点 (Strategy Node)
# =========================================================
add_executable(strategy_node src/strategy_node.cpp)
add_dependencies(strategy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(strategy_node
  ${catkin_LIBRARIES}
)

# =========================================================
# 编译执行控制节点 (Controller Node)
# =========================================================
add_executable(xarm_controller_node src/xarm_controller_node.cpp)
add_dependencies(xarm_controller_node ${catkin_EXPORTED_TARGETS} xarm_msgs_gencpp)
add_dependencies(xarm_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xarm_controller_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# =========================================================
# 编译视觉处理节点 (经典版：轮廓提取 + 模板匹配)
# =========================================================
add_executable(vision_processor_node_cpp src/vision_processor_node.cpp)
add_dependencies(vision_processor_node_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_processor_node_cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# =========================================================
# 编译视觉处理节点 (DexiNed 版：神经网络边缘检测 + 模板匹配)
# 与经典版话题/消息契约一致，靠 launch 的 type:= 切换。
# =========================================================
add_executable(vision_processor_node_dexined src/vision_processor_node_dexined.cpp)
add_dependencies(vision_processor_node_dexined ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_processor_node_dexined
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# =========================================================
# 编译路径规划节点 (Path Planner Node)
# 消费 /tetris_plan，做像素->base 抓取(含深度Z)+格子->base 放置解算，
# 发布 /motion_cmds (base 系抓放位姿队列) 供控制节点执行。
# =========================================================
add_executable(path_planner_node src/path_planner_node.cpp)
add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)