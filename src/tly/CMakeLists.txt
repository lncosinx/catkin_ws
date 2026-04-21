cmake_minimum_required(VERSION 3.0.2)
project(tly)

# 强制使用 C++14 编译，支持最新的语言特性和 random 库
add_compile_options(-std=c++14)

# 开启 O3 最高级别优化！对于纯算力的 DLX 算法，这能让速度提升几十倍
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")

# 🌟 1. 新增 message_generation 依赖
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
  message_generation 
)

# 🌟 2. 声明要编译的 Service 文件
# 确保在 tly 包的根目录下创建了 srv 文件夹，并把 GetPrecisePose.srv 放进去！
add_service_files(
  FILES
  GetPrecisePose.srv
)

# 🌟 3. 生成消息，依赖基础消息包
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 🌟 4. 在导出包时新增 message_runtime
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs cv_bridge moveit_ros_planning_interface tf2 xarm_msgs message_runtime
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

catkin_install_python(PROGRAMS
  scripts/table_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/single_block_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =========================================================
# 编译策略节点 (Strategy Node)
# =========================================================
add_executable(strategy_node src/strategy_node.cpp)
# 这里的 ${${PROJECT_NAME}_EXPORTED_TARGETS} 确保了先生成 srv 头文件，再编译 cpp
add_dependencies(strategy_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(strategy_node
  ${catkin_LIBRARIES}
)

# =========================================================
# 编译执行控制节点 (Controller Node)
# =========================================================
add_executable(xarm_controller_node src/xarm_controller_node.cpp)
add_dependencies(xarm_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xarm_controller_node
  ${catkin_LIBRARIES}
)