#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// --- 【核心物理映射参数】 ---
// 你需要根据实验室真实情况用直尺测量这些值！
const double GRID_SIZE = 0.017;       // 假设每个方块的边长是 17mm (0.017米)
const double BOARD_ORIGIN_X = 0.400; // 棋盘左上角格子的物理世界 X 坐标 (米)
const double BOARD_ORIGIN_Y = 0.000; // 棋盘左上角格子的物理世界 Y 坐标 (米)
const double PLACE_Z = 0.050;        // 放置时机械臂吸盘的物理 Z 高度 (米)
const double HOVER_Z = 0.150;        // 移动时的安全悬停高度 (米)

// 假设有一个函数可以从视觉系统获取散落方块的真实位置
// 现实中这应该通过订阅视觉系统的 TF 或话题来获取
bool getVisionPickPose(int piece_id, geometry_msgs::Pose& pick_pose) {
    // [Mock] 假装视觉告诉了我们这个方块在桌面的哪里
    pick_pose.position.x = 0.200; 
    pick_pose.position.y = 0.300; 
    pick_pose.position.z = 0.020; 
    // 假设抓取初始姿态朝下
    tf2::Quaternion q;
    q.setRPY(3.14159, 0, 0); 
    pick_pose.orientation = tf2::toMsg(q);
    return true;
}

// 接收到策略节点计划的回调函数
void planCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) return;

    int total_steps = msg->data[0];
    ROS_INFO("Received new plan! Total steps to execute: %d", total_steps);

    // 初始化 xArm MoveGroup
    static const std::string PLANNING_GROUP = "xarm6"; // 根据你的urdf可能为 xarm_manipulator
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.4);
    move_group.setMaxAccelerationScalingFactor(0.4);

    int data_index = 1;
    for (int i = 0; i < total_steps; ++i) {
        int id  = msg->data[data_index++];
        int way = msg->data[data_index++];
        int grid_x = msg->data[data_index++];
        int grid_y = msg->data[data_index++];

        ROS_INFO("Executing Step %d: Pick ID=%d, Place to Grid(%d, %d), Rotate way=%d", 
                  i+1, id, grid_x, grid_y, way);

        // ==========================================
        // 1. 抓取动作 (Pick)
        // ==========================================
        geometry_msgs::Pose pick_pose;
        if (!getVisionPickPose(id, pick_pose)) {
            ROS_ERROR("Vision lost piece ID=%d! Aborting execution.", id);
            return;
        }

        // 移至抓取点上方 -> 下降抓取 -> 抬起
        geometry_msgs::Pose hover_pick = pick_pose;
        hover_pick.position.z = HOVER_Z;
        move_group.setPoseTarget(hover_pick);
        move_group.move(); // 这里使用了简化的 move() 阻塞执行，实际可加容错判断

        move_group.setPoseTarget(pick_pose);
        move_group.move();

        // -> TODO: 这里调用你机械臂末端的吸盘/夹爪开启服务 (rosservice call) <-
        ROS_INFO(">>> Sucker turned ON <<<");
        ros::Duration(0.5).sleep(); 

        move_group.setPoseTarget(hover_pick);
        move_group.move();

        // ==========================================
        // 2. 放置动作 (Place) - 物理坐标映射核心！
        // ==========================================
        geometry_msgs::Pose place_pose;
        // 网格坐标转真实物理坐标 (注意 X/Y 方向的增减视你实验室的坐标系正反方向而定)
        place_pose.position.x = BOARD_ORIGIN_X + (grid_x * GRID_SIZE);
        place_pose.position.y = BOARD_ORIGIN_Y + (grid_y * GRID_SIZE);
        place_pose.position.z = PLACE_Z;

        // 计算所需的旋转角度 (way 0,1,2,3 分别代表旋转 0, 90, 180, 270 度)
        // 注意：xArm的末端通常是 Roll, Pitch 保持向下 (180度)，Yaw 决定方块旋转
        double yaw_angle = way * (3.14159 / 2.0); // way * 90度转弧度
        tf2::Quaternion q_place;
        q_place.setRPY(3.14159, 0, yaw_angle); // 末端朝下，绕Z轴转动相应的度数
        place_pose.orientation = tf2::toMsg(q_place);

        // 移至放置点上方 -> 下降放置 -> 抬起
        geometry_msgs::Pose hover_place = place_pose;
        hover_place.position.z = HOVER_Z;
        
        move_group.setPoseTarget(hover_place);
        move_group.move();

        move_group.setPoseTarget(place_pose);
        move_group.move();

        // -> TODO: 这里调用吸盘/夹爪关闭服务 <-
        ROS_INFO(">>> Sucker turned OFF <<<");
        ros::Duration(0.5).sleep();

        move_group.setPoseTarget(hover_place);
        move_group.move();
        
        ROS_INFO("Step %d Completed.", i+1);
    }
    
    ROS_INFO("All steps executed successfully. Moving to Home position.");
    // move_group.setNamedTarget("home");
    // move_group.move();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "xarm_controller_node");
    ros::NodeHandle nh;

    // MoveIt 需要异步 Spinner
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // 订阅策略节点发布的话题
    ros::Subscriber plan_sub = nh.subscribe("/tetris_plan", 1, planCallback);
    
    ROS_INFO("Controller Node Started. Waiting for strategy plan on '/tetris_plan'...");

    ros::waitForShutdown();
    return 0;
}