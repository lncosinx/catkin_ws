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

    geometry_msgs::TransformStamped cam_to_table_tx_; // 保存拍照瞬间的相机状态

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


    // 2. 彻底重写坐标系转换函数（使用严谨的数学射线求交）
    bool transformPixelToRobotBase(int u, int v, geometry_msgs::Pose& out_pose) {
        // 步骤 A: 计算相机坐标系下的 3D 射线方向向量
        double ray_x_cam = (u - CAM_CX) / CAM_FX;
        double ray_y_cam = (v - CAM_CY) / CAM_FY;
        double ray_z_cam = 1.0;

        // 步骤 B: 提取相机的旋转矩阵，将射线旋转到桌面坐标系(table_frame)
        tf2::Quaternion q_cam;
        tf2::fromMsg(cam_to_table_tx_.transform.rotation, q_cam);
        tf2::Matrix3x3 m_cam(q_cam);
        
        tf2::Vector3 ray_cam(ray_x_cam, ray_y_cam, ray_z_cam);
        tf2::Vector3 ray_table = m_cam * ray_cam; // 现在的射线方向是相对于桌面的

        // 步骤 C: 提取相机的三维空间位置
        tf2::Vector3 cam_pos(
            cam_to_table_tx_.transform.translation.x,
            cam_to_table_tx_.transform.translation.y,
            cam_to_table_tx_.transform.translation.z
        );

        // 步骤 D: 射线与目标高度平面 (Z = PICK_Z) 求交点
        // 数学公式: cam_pos.z + t * ray_table.z = PICK_Z
        if (std::abs(ray_table.z()) < 1e-6) {
            ROS_ERROR("Camera is looking parallel to the table, cannot intersect!");
            return false;
        }
        
        double t = (PICK_Z - cam_pos.z()) / ray_table.z();
        if (t < 0) {
            ROS_ERROR("The object is behind the camera? Please check TF tree.");
            return false;
        }

        // 步骤 E: 计算出物理世界中绝对准确的 X 和 Y
        tf2::Vector3 intersect_pt = cam_pos + ray_table * t;

        out_pose.position.x = intersect_pt.x();
        out_pose.position.y = intersect_pt.y();
        out_pose.position.z = PICK_Z; // 高度严格锁定为积木厚度

        // 默认朝向：末端向下直指桌面
        tf2::Quaternion q_down;
        q_down.setRPY(3.14159, 0, 0); 
        out_pose.orientation = tf2::toMsg(q_down);

        return true;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (msg->data.empty()) return;
        
        try {
        cam_to_table_tx_ = tf_buffer_.lookupTransform("table_frame", "camera_color_optical_frame", ros::Time(0), ros::Duration(3.0));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Failed to lock observation camera pose: %s", ex.what());
        return;
    }
        
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
            place_pose.position.x = BOARD_ORIGIN_X - (center_r_x2 / 2.0) * GRID_SIZE;
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