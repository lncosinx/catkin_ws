#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <xarm_msgs/SetDigitalIO.h>

// --- 【物理映射参数】 ---
const double GRID_SIZE = 0.017;       // 网格边长 17mm
const double BOARD_ORIGIN_X = 0.400;  // 棋盘左上角 X 物理坐标 (需通过TF或手工校准)
const double BOARD_ORIGIN_Y = 0.000;  // 棋盘左上角 Y 物理坐标
const double PLACE_Z = 0.050;         // 放置高度
const double HOVER_Z = 0.150;         // 安全悬停高度

// --- 【Realsense 相机内参】 (你需要从 /camera/color/camera_info 话题获取真实值) ---
const double CAM_FX = 910.0; 
const double CAM_FY = 910.0;
const double CAM_CX = 640.0; // 假设是 1280x720 分辨率
const double CAM_CY = 360.0;
const double TABLE_Z_IN_CAMERA = 0.35; // 假设拍照时，相机距离桌面的垂直深度是 35cm (最好订阅 depth 图获取真实Z)

class TetrisRobotController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber plan_sub_;
    ros::ServiceClient io_client_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    moveit::planning_interface::MoveGroupInterface move_group_;

public:
    TetrisRobotController() : 
        tf_listener_(tf_buffer_), 
        move_group_("xarm6") 
    {
        move_group_.setMaxVelocityScalingFactor(0.4);
        move_group_.setMaxAccelerationScalingFactor(0.4);
        plan_sub_ = nh_.subscribe("/tetris_plan", 1, &TetrisRobotController::planCallback, this);
        
        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>("/xarm/set_cgpio_digital");
        
        ROS_INFO("TF2 Listener initialized. Controller Ready.");
    }

    void setSuctionCup(bool on) {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = 0; // 保持你的旧设定，如果接在 CO 上记得加 8
        srv.request.value = on ? 1 : 0; 
        
        if (io_client_.call(srv)) {
            if (srv.response.ret == 0) {
                ROS_INFO("Suction cup (DO0) turned %s.", on ? "ON" : "OFF");
            } else {
                ROS_WARN("IO call succeeded, but returned xArm error code: %d", srv.response.ret);
            }
        } else {
            ROS_ERROR("Failed to call xArm IO service! Check if xarm driver is running.");
        }
    }

    bool transformPixelToRobotBase(int u, int v, geometry_msgs::Pose& out_pose) {
        double z_c = TABLE_Z_IN_CAMERA; 
        double x_c = (u - CAM_CX) * z_c / CAM_FX;
        double y_c = (v - CAM_CY) * z_c / CAM_FY;

        geometry_msgs::PointStamped pt_in_cam;
        pt_in_cam.header.frame_id = "camera_color_optical_frame";
        pt_in_cam.header.stamp = ros::Time(0); 
        pt_in_cam.point.x = x_c;
        pt_in_cam.point.y = y_c;
        pt_in_cam.point.z = z_c;

        geometry_msgs::PointStamped pt_in_base;

        try {
            tf_buffer_.transform(pt_in_cam, pt_in_base, "link_base", ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TF Conversion failed: %s", ex.what());
            return false;
        }

        out_pose.position.x = pt_in_base.point.x;
        out_pose.position.y = pt_in_base.point.y;
        out_pose.position.z = PLACE_Z; 

        // 默认朝向（稍后在代码中会被角度偏置覆盖）
        tf2::Quaternion q;
        q.setRPY(3.14159, 0, 0); 
        out_pose.orientation = tf2::toMsg(q);

        return true;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.empty()) return;

        int total_steps = msg->data[0];
        ROS_INFO("Received new plan! Total steps: %d", total_steps);

        int data_index = 1;
        for (int i = 0; i < total_steps; ++i) {
            int id  = msg->data[data_index++]; 
            int way = msg->data[data_index++]; 
            
            // 【核心修正】：读取策略发过来的网格中心点 (*2)
            int center_r_x2 = msg->data[data_index++]; 
            int center_c_x2 = msg->data[data_index++]; 
            
            int pixel_u = msg->data[data_index++]; 
            int pixel_v = msg->data[data_index++];
            
            // 【核心修正】：补上被遗漏的角度读取！
            int pick_angle = msg->data[data_index++]; 

            ROS_INFO("Step %d: Pick ID=%d at pixel(%d,%d) angle:%d, Place GridCenter(%d, %d)", 
                      i+1, id, pixel_u, pixel_v, pick_angle, center_r_x2, center_c_x2);

            // ==========================================
            // 1. 抓取动作 (Pick)
            // ==========================================
            geometry_msgs::Pose pick_pose;
            if (!transformPixelToRobotBase(pixel_u, pixel_v, pick_pose)) {
                ROS_ERROR("Aborting execution due to TF error.");
                return;
            }

            // 用吸盘的偏航角完美抵消零件在桌面的随机旋转
            double pick_yaw = pick_angle * (3.14159 / 180.0);
            tf2::Quaternion q_pick;
            q_pick.setRPY(3.14159, 0, pick_yaw); 
            pick_pose.orientation = tf2::toMsg(q_pick);

            geometry_msgs::Pose hover_pick = pick_pose;
            hover_pick.position.z = HOVER_Z;
            
            move_group_.setPoseTarget(hover_pick); move_group_.move();
            move_group_.setPoseTarget(pick_pose);  move_group_.move();
            
            ROS_INFO(">>> Sucker turned ON <<<");
            setSuctionCup(true); 
            ros::Duration(0.5).sleep(); 

            move_group_.setPoseTarget(hover_pick); move_group_.move();

            // ==========================================
            // 2. 放置动作 (Place) 
            // ==========================================
            geometry_msgs::Pose place_pose;
            
            // 【核心修正】：Center-to-Center 完美映射！除以 2 还原真实物理中心
            place_pose.position.x = BOARD_ORIGIN_X + (center_r_x2 / 2.0) * GRID_SIZE;
            place_pose.position.y = BOARD_ORIGIN_Y + (center_c_x2 / 2.0) * GRID_SIZE;
            place_pose.position.z = PLACE_Z;

            double yaw_angle = way * (3.14159 / 2.0); 
            tf2::Quaternion q_place;
            q_place.setRPY(3.14159, 0, yaw_angle); 
            place_pose.orientation = tf2::toMsg(q_place);

            geometry_msgs::Pose hover_place = place_pose;
            hover_place.position.z = HOVER_Z;
            
            move_group_.setPoseTarget(hover_place); move_group_.move();
            move_group_.setPoseTarget(place_pose);  move_group_.move();

            ROS_INFO(">>> Sucker turned OFF <<<");
            setSuctionCup(false); 
            ros::Duration(0.5).sleep();

            move_group_.setPoseTarget(hover_place); move_group_.move();
            
            ROS_INFO("Step %d Completed.", i+1);
        }
        
        ROS_INFO("All steps executed successfully.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "xarm_controller_node");
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    TetrisRobotController controller;
    ros::waitForShutdown();
    
    return 0;
}