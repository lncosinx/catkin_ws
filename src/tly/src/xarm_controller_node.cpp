#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <xarm_msgs/SetDigitalIO.h>

const double VELOCITY = 0.01;
const double ACCELEARTION = 0.01;

class TetrisRobotController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber plan_sub_;
    ros::ServiceClient io_client_;
    ros::Publisher status_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    moveit::planning_interface::MoveGroupInterface move_group_;

    // 动态加载的参数
    double GRID_SIZE, BOARD_ORIGIN_X, BOARD_ORIGIN_Y;
    double PICK_Z, PLACE_Z, HOVER_Z; // 【新增】：加入 PICK_Z
    double CAM_FX, CAM_FY, CAM_CX, CAM_CY, TABLE_Z_IN_CAMERA;

public:
    TetrisRobotController() : 
        tf_listener_(tf_buffer_), 
        move_group_("xarm6") 
    {
        // 1. 从 ROS 参数服务器加载配置文件 (如果没读到，使用后备默认值)
        nh_.param("/tetris/GRID_SIZE", GRID_SIZE, 0.017);
        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X, 0.0);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y, 0.0);
        nh_.param("/tetris/PICK_Z", PICK_Z, 0.0);  // 【新增】：从参数服务器读取抓取高度
        nh_.param("/tetris/PLACE_Z", PLACE_Z, 0.0);
        nh_.param("/tetris/HOVER_Z", HOVER_Z, 0.1);
        nh_.param("/tetris/CAM_FX", CAM_FX, 911.8016);
        nh_.param("/tetris/CAM_FY", CAM_FY, 911.2428);
        nh_.param("/tetris/CAM_CX", CAM_CX, 634.7139);
        nh_.param("/tetris/CAM_CY", CAM_CY, 357.0596);
        nh_.param("/tetris/TABLE_Z_IN_CAMERA", TABLE_Z_IN_CAMERA, 0.49);

        // 2. 将规划的“绝对参考系”切换为我们的自定义桌面坐标系！
        move_group_.setPoseReferenceFrame("table_frame");
        move_group_.setMaxVelocityScalingFactor(VELOCITY);
        move_group_.setMaxAccelerationScalingFactor(ACCELEARTION);
        
        plan_sub_ = nh_.subscribe("/tetris_plan", 1, &TetrisRobotController::planCallback, this);
        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>("/xarm/set_controller_dout");
        status_pub_ = nh_.advertise<std_msgs::Bool>("/robot_status", 1, true);
        
        std_msgs::Bool init_msg;
        init_msg.data = false;
        status_pub_.publish(init_msg);
        
        ROS_INFO("Controller Ready. Operating in [table_frame] coordinate system.");
    }

    void setSuctionCup(bool on) {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = 1; 
        srv.request.value = on ? 1 : 0; 
        if (!io_client_.call(srv)) {
            ROS_ERROR("Failed to call xArm IO service!");
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

        geometry_msgs::PointStamped pt_in_table;

        try {
            // 直接将像素目标转化为 table_frame (桌面平面) 上的点
            tf_buffer_.transform(pt_in_cam, pt_in_table, "table_frame", ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("TF Conversion failed: %s", ex.what());
            return false;
        }

        out_pose.position.x = pt_in_table.point.x;
        out_pose.position.y = pt_in_table.point.y;
        out_pose.position.z = PICK_Z; // 【核心修改】：抓取时(通过视觉转换的坐标)，Z高度必须使用 PICK_Z

        // 默认朝向：末端向下直指桌面
        tf2::Quaternion q;
        q.setRPY(3.14159, 0, 0); 
        out_pose.orientation = tf2::toMsg(q);

        return true;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.empty()) return;

        std_msgs::Bool status_msg;
        status_msg.data = true;
        status_pub_.publish(status_msg);

        int total_steps = msg->data[0];
        ROS_INFO("Received new plan! Total steps: %d.", total_steps);

        int data_index = 1;
        for (int i = 0; i < total_steps; ++i) {
            int id = msg->data[data_index++]; 
            int way = msg->data[data_index++]; 
            int center_r_x2 = msg->data[data_index++]; 
            int center_c_x2 = msg->data[data_index++]; 
            int pixel_u = msg->data[data_index++]; 
            int pixel_v = msg->data[data_index++];
            int pick_angle = msg->data[data_index++]; 

            // 1. Pick
            geometry_msgs::Pose pick_pose;
            if (!transformPixelToRobotBase(pixel_u, pixel_v, pick_pose)) return;

            double pick_yaw = pick_angle * (3.14159 / 180.0);
            tf2::Quaternion q_pick;
            q_pick.setRPY(3.14159, 0, pick_yaw); 
            pick_pose.orientation = tf2::toMsg(q_pick);

            geometry_msgs::Pose hover_pick = pick_pose;
            hover_pick.position.z = HOVER_Z;
            
            move_group_.setPoseTarget(hover_pick); move_group_.move();
            move_group_.setPoseTarget(pick_pose);  move_group_.move();
            
            setSuctionCup(true); ros::Duration(0.5).sleep(); 
            move_group_.setPoseTarget(hover_pick); move_group_.move();

            // 2. Place
            geometry_msgs::Pose place_pose;
            
            // 【核心修正】：因为我们新标定的 Y 轴在算法里指代了相反方向，所以这里用减号
            place_pose.position.x = BOARD_ORIGIN_X + (center_r_x2 / 2.0) * GRID_SIZE;
            place_pose.position.y = BOARD_ORIGIN_Y - (center_c_x2 / 2.0) * GRID_SIZE; 
            place_pose.position.z = PLACE_Z;

            double yaw_angle = way * (3.14159 / 2.0); 
            tf2::Quaternion q_place;
            q_place.setRPY(3.14159, 0, yaw_angle); 
            place_pose.orientation = tf2::toMsg(q_place);

            geometry_msgs::Pose hover_place = place_pose;
            hover_place.position.z = HOVER_Z;
            
            move_group_.setPoseTarget(hover_place); move_group_.move();
            move_group_.setPoseTarget(place_pose);  move_group_.move();

            setSuctionCup(false); ros::Duration(0.5).sleep();
            move_group_.setPoseTarget(hover_place); move_group_.move();
        }
        
        move_group_.setNamedTarget("home"); 
        move_group_.move();
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