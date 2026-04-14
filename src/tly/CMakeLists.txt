cmake_minimum_required(VERSION 3.0.2)
project(tly)

# 强制使用 C++14 编译，支持最新的语言特性和 random 库
add_compile_options(-std=c++14)

# 开启 O3 最高级别优化！对于纯算力的 DLX 算法，这能让速度提升几十倍
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")

# 寻找相关的 catkin 依赖包 (新增了 geometry_msgs, sensor_msgs, cv_bridge)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_ros
  tf2_geometry_msgs
  xarm_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs cv_bridge moveit_ros_planning_interface tf2 xarm_msgs
)

# 定义包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/vision_processor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =========================================================
# 1. 编译策略节点 (Strategy Node)
#    纯粹的算力节点，不需要链接 MoveIt!，只需要 std_msgs 通信
# =========================================================
add_executable(strategy_node src/strategy_node.cpp)
add_dependencies(strategy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(strategy_node
  ${catkin_LIBRARIES}
)

# =========================================================
# 2. 编译执行控制节点 (Controller Node)
#    需要调用 MoveIt! API 控制真实机械臂，以及使用 TF2
# =========================================================
add_executable(xarm_controller_node src/xarm_controller_node.cpp)
add_dependencies(xarm_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xarm_controller_node
  ${catkin_LIBRARIES}
)