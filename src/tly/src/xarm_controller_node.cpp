#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <xarm_msgs/SetDigitalIO.h>
#include <tly/GetPrecisePose.h> 
#include <queue>

const double VELOCITY = 0.05;
const double ACCELEARTION = 0.05;

// 1. 定义状态机枚举
enum class RobotState {
    IDLE,
    FLY_TO_HOVER,
    REQUEST_VISION,
    EXECUTE_PICK,
    EXECUTE_PLACE
};

// 2. 定义任务结构体
struct TaskGoal {
    int shape_type;
    int way;
    geometry_msgs::Pose coarse_pick_pose; // 策略节点给的粗略坐标
    geometry_msgs::Pose place_pose;       // 策略节点给的放置坐标
};

class TetrisRobotController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber plan_sub_;
    ros::ServiceClient io_client_;
    ros::Publisher status_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::ServiceClient vision_client_;
    ros::Timer control_timer_;
    
    RobotState current_state_ = RobotState::IDLE;
    std::queue<TaskGoal> task_queue_;
    TaskGoal current_task_;
    
    geometry_msgs::Pose precise_pick_pose_; // 修正后的精确抓取坐标
    
    moveit::planning_interface::MoveGroupInterface move_group_;

    geometry_msgs::TransformStamped cam_to_table_tx_; // 保存拍照瞬间的相机状态

    // 动态加载的参数
    double GRID_SIZE, BOARD_ORIGIN_X, BOARD_ORIGIN_Y;
    double PICK_Z, PLACE_Z, HOVER_Z; // 【新增】：加入 PICK_Z
    double CAM_FX, CAM_FY, CAM_CX, CAM_CY, TABLE_Z_IN_CAMERA;
    double CAM_OFFSET_X, CAM_OFFSET_Y;

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
        nh_.param("/tetris/CAM_OFFSET_X", CAM_OFFSET_X, 0.06); 
        nh_.param("/tetris/CAM_OFFSET_Y", CAM_OFFSET_Y, 0.00);

        // 2. 将规划的“绝对参考系”切换为我们的自定义桌面坐标系！
        move_group_.setPoseReferenceFrame("table_frame");
        move_group_.setMaxVelocityScalingFactor(VELOCITY);
        move_group_.setMaxAccelerationScalingFactor(ACCELEARTION);
        vision_client_ = nh_.serviceClient<tly::GetPrecisePose>("/vision/get_precise_pose");
        
        // 开启一个 10Hz 的定时器来驱动状态机
        control_timer_ = nh_.createTimer(ros::Duration(0.1), &TetrisRobotController::controlLoop, this);
        
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

        // 步骤 E: 计算出物理世界中初步的 X 和 Y (基于射线的理论交点)
        tf2::Vector3 intersect_pt = cam_pos + ray_table * t;

        // ==========================================
        // 🔧 工业级终极补偿：平移 + 缩放 (两步法)
        // ==========================================
        
        // 1. 静态补偿 (只负责修复画面【正中心】的绝对偏移)
        // 先全部重置为 0 开始调
        double COMP_X = -0.0045;   
        double COMP_Y = 0.0035;  

        // 计算方块落在距离相机正下方（视野中心）有多远
        double dist_x = intersect_pt.x() - cam_pos.x();
        double dist_y = intersect_pt.y() - cam_pos.y();

        // 2. 动态发散系数 (负责修复【越靠画面边缘偏得越多】的问题)
        // 填入极小的小数，例如 0.05 或 -0.03
        double SCALE_X = 0.155; 
        double SCALE_Y = 0.2; 

        // 最终坐标 = 理论坐标 + 固定平移 + 随距离放大的畸变修正
        out_pose.position.x = intersect_pt.x() + COMP_X + (dist_x * SCALE_X);
        out_pose.position.y = intersect_pt.y() + COMP_Y + (dist_y * SCALE_Y);
        out_pose.position.z = PICK_Z;

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
        ROS_INFO("Received new plan! Preparing %d steps for state machine...", total_steps);

        // 🌟 清空旧队列，防止接收到新规划时任务堆积
        while(!task_queue_.empty()) task_queue_.pop();

        int data_index = 1;
        for (int i = 0; i < total_steps; ++i) {
            int id = msg->data[data_index++]; 
            int way = msg->data[data_index++]; 
            int sum_r = msg->data[data_index++]; 
            int sum_c = msg->data[data_index++];
            int pixel_u = msg->data[data_index++]; 
            int pixel_v = msg->data[data_index++];
            int pick_angle = msg->data[data_index++]; 

            // 1. 初始化任务目标
            TaskGoal goal;
            goal.shape_type = id;
            goal.way = way;

            // 2. 计算粗略 Pick 坐标（用于悬停）
            if (!transformPixelToRobotBase(pixel_u, pixel_v, goal.coarse_pick_pose)) {
                ROS_WARN("Failed to calc coarse pose for block %d, skipping.", i);
                continue;
            }
            // 【核心修正】：悬停时，强制法兰盘朝向正前方 (Yaw=0)，不要提前旋转！
            // 这保证了相机坐标系的 X/Y 轴与桌面的 X/Y 轴平行，从而让后续的 dx, dy, angle 补偿完美对应。
            tf2::Quaternion q_hover;
            q_hover.setRPY(3.14159, 0, 0); 
            goal.coarse_pick_pose.orientation = tf2::toMsg(q_hover);

            // 3. 计算最终放置 Place 坐标
            goal.place_pose.position.x = BOARD_ORIGIN_X - (sum_r / 4.0) * GRID_SIZE;
            goal.place_pose.position.y = BOARD_ORIGIN_Y - (sum_c / 4.0) * GRID_SIZE; 
            goal.place_pose.position.z = PLACE_Z;
            double yaw_angle = way * (3.14159 / 2.0); 
            tf2::Quaternion q_place;
            q_place.setRPY(3.14159, 0, yaw_angle); 
            goal.place_pose.orientation = tf2::toMsg(q_place);

            // 4. 将任务塞进队列（此时机械臂不动！）
            task_queue_.push(goal);
        }
        
        ROS_INFO("Successfully queued %lu tasks.", task_queue_.size());
    
        // 5. 如果状态机处于空闲，点火起飞！交由 controlLoop 接管控制权
        if (current_state_ == RobotState::IDLE && !task_queue_.empty()) {
            ROS_INFO("State Machine: Transitioning from IDLE to FLY_TO_HOVER");
            current_state_ = RobotState::FLY_TO_HOVER;
        }
    }

    // 状态机核心驱动逻辑
    void controlLoop(const ros::TimerEvent&) {
        switch (current_state_) {
            case RobotState::IDLE:
                // 等待新任务
                break;

            case RobotState::FLY_TO_HOVER:
                if (task_queue_.empty()) {
                    current_state_ = RobotState::IDLE;
                    break;
                }
                current_task_ = task_queue_.front();
                task_queue_.pop();

                {
                    geometry_msgs::Pose hover_pose = current_task_.coarse_pick_pose;
                    hover_pose.position.z = HOVER_Z;
                    
                    // 🌟 修正 1：放弃 TF 直接相减，改回全局参数补偿
                    // 这里我们暴露了符号。具体是 + 还是 -，需要看实际物理表现。
                    hover_pose.position.x += CAM_OFFSET_X; 
                    hover_pose.position.y += CAM_OFFSET_Y;

                    move_group_.setPoseTarget(hover_pose);
                    move_group_.move(); // 阻塞式移动到悬停点
                    
                    ros::Duration(5).sleep(); // 稳定一下画面
                    current_state_ = RobotState::REQUEST_VISION;
                }
                break;

            case RobotState::REQUEST_VISION:
                {
                    tly::GetPrecisePose srv;
                    srv.request.target_shape_type = current_task_.shape_type;
                    
                    if (vision_client_.call(srv) && srv.response.success) {
                        
                        double z_dist = HOVER_Z - PICK_Z; 
                        
                        // 暴露像素到物理的映射极性
                        double dX_physical = (srv.response.dy * z_dist) / CAM_FY;
                        double dY_physical = (srv.response.dx * z_dist) / CAM_FX;
                        
                        ROS_INFO("Vision offset: dx_pix=%d, dy_pix=%d | Physical: dX=%.4f, dY=%.4f", 
                                  srv.response.dx, srv.response.dy, dX_physical, dY_physical);

                        precise_pick_pose_ = current_task_.coarse_pick_pose;

                        // ==========================================
                        // 🌟 核心修正 2：最终机械微调 (Fine-Tuning)
                        // 用于补偿相机光心与吸盘物理中心的那一两毫米安装误差
                        // ==========================================
                        double FINE_TUNE_X = 0.002; // 如果每次都往前偏，填负数，比如 -0.003
                        double FINE_TUNE_Y = -0.002; // 如果每次都往左偏，填负数，比如 -0.002

                        // 将视觉物理偏差 + 机械微调 加回到目标上
                        precise_pick_pose_.position.x += (dX_physical + FINE_TUNE_X); 
                        precise_pick_pose_.position.y += (dY_physical + FINE_TUNE_Y);
                        
                        double precise_yaw = srv.response.angle * (3.14159 / 180.0);
                        tf2::Quaternion q_pick;
                        q_pick.setRPY(3.14159, 0, precise_yaw);
                        precise_pick_pose_.orientation = tf2::toMsg(q_pick);

                        current_state_ = RobotState::EXECUTE_PICK;
                    } else {
                        ROS_WARN("Vision failed! Robot stopped to let you check the camera image.");
                        current_state_ = RobotState::IDLE;
                    }
                }
                break;

            case RobotState::EXECUTE_PICK:
                {
                    // 执行精准下降抓取
                    move_group_.setPoseTarget(precise_pick_pose_);
                    move_group_.move();
                    setSuctionCup(true);
                    ros::Duration(0.5).sleep();
                    
                    // 提起到悬停高度
                    geometry_msgs::Pose hover_pose = precise_pick_pose_;
                    hover_pose.position.z = HOVER_Z;
                    move_group_.setPoseTarget(hover_pose);
                    move_group_.move();
                    
                    current_state_ = RobotState::EXECUTE_PLACE;
                }
                break;

            case RobotState::EXECUTE_PLACE:
                {
                    // 飞到目标白板位置放置
                    geometry_msgs::Pose hover_place = current_task_.place_pose;
                    hover_place.position.z = HOVER_Z;
                    
                    move_group_.setPoseTarget(hover_place); move_group_.move();
                    move_group_.setPoseTarget(current_task_.place_pose); move_group_.move();
                    
                    setSuctionCup(false);
                    ros::Duration(0.5).sleep();
                    
                    move_group_.setPoseTarget(hover_place); move_group_.move();
                    
                    // 本方块完成，回到第一个状态抓下一个
                    current_state_ = RobotState::FLY_TO_HOVER; 
                }
                break;
        }
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