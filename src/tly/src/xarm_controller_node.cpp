#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

// 重新拥抱 MoveIt，但这次我们使用工业级的 Pilz 规划器
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <xarm_msgs/SetDigitalIO.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tly/GetPrecisePose.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <XmlRpcValue.h>
#include <cmath>
#include <queue>
#include <string>
#include <vector>
#include <signal.h>

struct PixelPoint
{
    int u = 0;
    int v = 0;
};
struct GridCenter
{
    double row = 0.0;
    double col = 0.0;
};
struct GridCell
{
    int row = 0;
    int col = 0;
};

struct TaskGoal
{
    int shape_type = -1;
    int way = 0;
    int pick_angle_deg = 0;
    PixelPoint pick_pixel;
    PixelPoint geom_pixel;
    bool has_geom_pixel = false;
    double pick_to_geom_dx = 0.0;
    double pick_to_geom_dy = 0.0;
    GridCenter place_grid_center;
    std::vector<GridCell> target_cells;
    geometry_msgs::Pose pick_pose;  // 以 base 为基准的抓取点
    geometry_msgs::Pose place_pose; // 以 base 为基准的放置点
};

class XarmTetrisController
{
public:
    XarmTetrisController()
        : pnh_("~"),
          tf_listener_(tf_buffer_)
    {
        loadParams();

        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &XarmTetrisController::cameraInfoCallback, this);

        ROS_INFO("[CTRL] Waiting for CameraInfo...");
        sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));
        if (cam_msg)
            cameraInfoCallback(cam_msg);

        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>(io_service_);
        precise_client_ = nh_.serviceClient<tly::GetPrecisePose>(precise_service_);

        plan_sub_ = nh_.subscribe(plan_topic_, 1, &XarmTetrisController::planCallback, this);
        status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
        publishBusy(false);

        // =======================================================
        // 初始化 MoveIt! 与 Pilz 规划器
        // =======================================================
        ROS_INFO("[CTRL] Initializing MoveGroup for 'xarm6'...");
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface("xarm6"));

        // 【关键配置】：指定规划管线为 Pilz
        move_group_->setPlanningPipelineId("");
        // 【核心优化】：将 TCP 设置为你昨天刚在 TF 里发布的实际吸盘尖端！
        move_group_->setEndEffectorLink("link_tcp");

        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        control_timer_ = nh_.createTimer(ros::Duration(0.05), &XarmTetrisController::controlLoop, this);
        ROS_INFO("[CTRL] Xarm Pilz Controller Ready!");
    }

    void emergencyStop()
    {
        ROS_ERROR("====================================================");
        ROS_ERROR("= [CTRL] Ctrl+C DETECTED! STOPPING MOVEIT!         =");
        ROS_ERROR("====================================================");
        if (move_group_)
        {
            move_group_->stop();
        }
    }

private:
    enum class State
    {
        IDLE,
        TAKE_NEXT_TASK,
        MOVE_TO_PICK_HOVER,
        OPTIONAL_PRECISE_CORRECTION,
        EXECUTE_PICK,
        MOVE_TO_PLACE_HOVER,
        EXECUTE_PLACE,
        FINISH
    };

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber plan_sub_, camera_info_sub_;
    ros::Publisher status_pub_;
    ros::Timer control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::ServiceClient io_client_, precise_client_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    State state_ = State::IDLE;
    std::queue<TaskGoal> tasks_;
    TaskGoal current_task_;

    std::string plan_topic_ = "/tetris_plan";
    std::string status_topic_ = "/robot_status";
    std::string io_service_ = "/xarm/set_controller_dout";
    std::string precise_service_ = "/vision/get_precise_pose";
    std::string camera_info_topic_ = "/camera/color/camera_info";
    std::string table_frame_ = "table_frame";
    std::string base_frame_ = "link_base";

    double velocity_scale_ = 0.3; // Pilz 比较平滑，速度可以适当开高一点
    double acceleration_scale_ = 0.3;

    double GRID_SIZE_ = 0.017;
    double BOARD_ORIGIN_X_ = 0.0;
    double BOARD_ORIGIN_Y_ = 0.0;

    // 【高度优化】：现在我们只需要 Hover 和 Target，去掉了多余的 SAFE_TRANSIT_Z
    double PICK_Z_ = 0.0;
    double PLACE_Z_ = 0.0;
    double HOVER_Z_ = 0.05; // 悬停高度：距离桌面 5cm，足够跨越普通积木块

    // 【恢复冗余设计】：抓取/放置的局部工艺微调偏移量（单位：米）
    double tcp_pick_offset_x_ = 0.0;
    double tcp_pick_offset_y_ = 0.0;
    double tcp_pick_offset_z_ = 0.0;
    double tcp_place_offset_x_ = 0.0;
    double tcp_place_offset_y_ = 0.0;
    double tcp_place_offset_z_ = 0.0;

    int suction_io_num_ = 1;
    bool camera_info_ready_ = false;
    cv::Mat K_, D_, P_;
    geometry_msgs::TransformStamped observation_cam_to_table_;

    double last_commanded_yaw_ = 0.0;
    bool has_last_commanded_yaw_ = false;

    // 角度解卷绕：确保手腕旋转永远走最短路径，不转无意义的 360 度
    double unwrapYaw(double target, double reference) const
    {
        double diff = target - reference;
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff < -M_PI)
            diff += 2.0 * M_PI;
        return reference + diff;
    }

    void loadParams()
    {
        pnh_.param("velocity_scale", velocity_scale_, velocity_scale_);
        pnh_.param("acceleration_scale", acceleration_scale_, acceleration_scale_);
        nh_.param("/tetris/PICK_Z", PICK_Z_, PICK_Z_);
        nh_.param("/tetris/PLACE_Z", PLACE_Z_, PLACE_Z_);
        nh_.param("/tetris/HOVER_Z", HOVER_Z_, HOVER_Z_);

        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X_, 0.3563);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y_, -0.3971);
        nh_.param("/tetris/GRID_SIZE", GRID_SIZE_, 0.0202);

        // 加载微调参数
        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        pnh_.param("tcp_pick_offset_z", tcp_pick_offset_z_, tcp_pick_offset_z_);
        pnh_.param("tcp_place_offset_x", tcp_place_offset_x_, tcp_place_offset_x_);
        pnh_.param("tcp_place_offset_y", tcp_place_offset_y_, tcp_place_offset_y_);
        pnh_.param("tcp_place_offset_z", tcp_place_offset_z_, tcp_place_offset_z_);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        K_ = cv::Mat(3, 3, CV_64F);
        P_ = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 9; ++i)
            K_.at<double>(i / 3, i % 3) = msg->K[i];
        for (int i = 0; i < 12; ++i)
            P_.at<double>(i / 4, i % 4) = msg->P[i];
        camera_info_ready_ = true;
    }

    double normalizeAngleRad(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    // Pilz 执行接口，带 IK 防翻转自动降级防御机制
    bool executePilzMotion(const geometry_msgs::Pose &target_pose, const std::string &planner_id = "PTP")
    {
        move_group_->setPlannerId(planner_id);

        if (planner_id == "PTP")
        {
            move_group_->setStartStateToCurrentState();

            // 1. 手动计算 IK，以便我们提前审查它
            robot_state::RobotState current_state = *move_group_->getCurrentState();
            const robot_state::JointModelGroup *jmg = current_state.getJointModelGroup("xarm6");

            if (!current_state.setFromIK(jmg, target_pose, 0.1))
            {
                ROS_ERROR("[CTRL] PTP IK Failed to find solution.");
                return false;
            }

            // 2. 提取新解和当前实际关节状态
            std::vector<double> new_joints, old_joints;
            current_state.copyJointGroupPositions(jmg, new_joints);
            move_group_->getCurrentState()->copyJointGroupPositions(jmg, old_joints);

            // 3. 审查前 4 个大关节，看是否有诡异的“大风车翻转”
            bool flip_detected = false;
            for (size_t i = 0; i < 4; ++i)
            {
                if (std::abs(new_joints[i] - old_joints[i]) > 1.5)
                { // 偏差大于约 85 度
                    flip_detected = true;
                    break;
                }
            }

            // 4. 如果探测到翻转，安全降级！
            if (flip_detected)
            {
                ROS_WARN("[CTRL] PTP IK Jump Detected! Safely falling back to Cartesian LIN.");
                return executePilzMotion(target_pose, "LIN"); // 递归调用，强行改为纯直线平移
            }
            else
            {
                move_group_->setJointValueTarget(current_state); // 审查通过，采用平顺解
            }
        }
        else
        {
            // LIN 直线运动，直接发送 Pose 即可，底层 Jacobian 会保证绝对不翻转
            move_group_->setPoseTarget(target_pose);
        }

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            return (move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        else
        {
            ROS_ERROR("[CTRL] Pilz %s Planning Failed!", planner_id.c_str());
            return false;
        }
    }

    geometry_msgs::Pose hoverFrom(const geometry_msgs::Pose &pose) const
    {
        geometry_msgs::Pose h = pose;
        h.position.z = HOVER_Z_;
        return h;
    }

    bool setSuction(bool on)
    {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = suction_io_num_;
        srv.request.value = on ? 1 : 0;
        if (!io_client_.call(srv))
            return false;
        ros::Duration(on ? 0.20 : 0.15).sleep();
        return true;
    }

    // 视觉像素到桌面的转换
    bool pixelToTablePoint(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        if (!camera_info_ready_)
            return false;
        double fx = P_.at<double>(0, 0), fy = P_.at<double>(1, 1);
        double cx = P_.at<double>(0, 2), cy = P_.at<double>(1, 2);
        cv::Vec3d ray_cam((px.u - cx) / fx, (px.v - cy) / fy, 1.0);

        tf2::Quaternion q;
        tf2::fromMsg(observation_cam_to_table_.transform.rotation, q);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 ray_table(
            R[0][0] * ray_cam[0] + R[0][1] * ray_cam[1] + R[0][2] * ray_cam[2],
            R[1][0] * ray_cam[0] + R[1][1] * ray_cam[1] + R[1][2] * ray_cam[2],
            R[2][0] * ray_cam[0] + R[2][1] * ray_cam[1] + R[2][2] * ray_cam[2]);

        tf2::Vector3 cam_pos(observation_cam_to_table_.transform.translation.x,
                             observation_cam_to_table_.transform.translation.y,
                             observation_cam_to_table_.transform.translation.z);

        double t = (PICK_Z_ - cam_pos.z()) / ray_table.z();
        if (t < 0.0)
            return false;
        tf2::Vector3 p = cam_pos + ray_table * t;
        out.x = p.x();
        out.y = p.y();
        out.z = p.z();
        return true;
    }

    // 生成垂直向下的 Pose
    geometry_msgs::Pose makeDownPoseBase(double x, double y, double z, double yaw_rad) const
    {
        geometry_msgs::PoseStamped pt_table, pt_base;
        pt_table.header.frame_id = table_frame_;
        pt_table.pose.position.x = x;
        pt_table.pose.position.y = y;
        pt_table.pose.position.z = z;

        tf2::Quaternion q;
        // 【保命修复 2】：打破万向节死锁！
        // M_PI - 0.002 相当于倾斜了约 0.1度，肉眼看不出，但能救机械臂的命
        q.setRPY(M_PI - 0.001, 0.001, yaw_rad);
        pt_table.pose.orientation = tf2::toMsg(q);

        try
        {
            tf_buffer_.transform(pt_table, pt_base, base_frame_, ros::Duration(1.0));
            return pt_base.pose;
        }
        catch (...)
        {
            ROS_ERROR("[CTRL] TF Transform failed to base_frame");
            return pt_base.pose;
        }
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty() || !camera_info_ready_)
            return;

        try
        {
            observation_cam_to_table_ = tf_buffer_.lookupTransform(table_frame_, "camera_color_optical_frame", ros::Time(0), ros::Duration(1.0));
        }
        catch (...)
        {
            return;
        }

        while (!tasks_.empty())
            tasks_.pop();

        int total = msg->data[0];
        if (total <= 0)
            return;

        int stride = (msg->data.size() - 1) / total;
        double ref_yaw = last_commanded_yaw_;

        for (int i = 0; i < total; ++i)
        {
            int base_idx = 1 + i * stride;
            TaskGoal goal;
            goal.shape_type = msg->data[base_idx + 0];
            goal.way = msg->data[base_idx + 1];
            goal.place_grid_center.row = msg->data[base_idx + 2] / 4.0;
            goal.place_grid_center.col = msg->data[base_idx + 3] / 4.0;
            goal.pick_pixel.u = msg->data[base_idx + 4];
            goal.pick_pixel.v = msg->data[base_idx + 5];

            // 【新增】：读取当前需要的角度
            double raw_yaw = msg->data[base_idx + 6] * M_PI / 180.0;
            if (!has_last_commanded_yaw_ && i == 0)
            {
                ref_yaw = raw_yaw;
                has_last_commanded_yaw_ = true;
            }
            // 平滑化：保证抓取旋转走最短路径
            raw_yaw = unwrapYaw(raw_yaw, ref_yaw);
            ref_yaw = raw_yaw;
            goal.pick_angle_deg = raw_yaw * 180.0 / M_PI; // 更新内部存值（如果需要的话）

            if (stride >= 9)
            {
                goal.geom_pixel.u = msg->data[base_idx + 7];
                goal.geom_pixel.v = msg->data[base_idx + 8];
                goal.has_geom_pixel = true;
            }
            else
            {
                goal.geom_pixel = goal.pick_pixel;
                goal.has_geom_pixel = false;
            }

            geometry_msgs::Point pick_p, geom_p;
            if (pixelToTablePoint(goal.pick_pixel, pick_p) && pixelToTablePoint(goal.geom_pixel, geom_p))
            {
                double pick_x = pick_p.x + std::cos(raw_yaw) * tcp_pick_offset_x_ - std::sin(raw_yaw) * tcp_pick_offset_y_;
                double pick_y = pick_p.y + std::sin(raw_yaw) * tcp_pick_offset_x_ + std::cos(raw_yaw) * tcp_pick_offset_y_;
                double pick_z = pick_p.z + tcp_pick_offset_z_;
                goal.pick_pose = makeDownPoseBase(pick_x, pick_y, pick_z, raw_yaw);

                // 平滑化：保证放置旋转走最短路径
                double place_yaw = -goal.way * M_PI / 2.0;
                place_yaw = unwrapYaw(place_yaw, ref_yaw);
                ref_yaw = place_yaw;

                double delta_yaw = place_yaw - raw_yaw;
                double dx = pick_p.x - geom_p.x;
                double dy = pick_p.y - geom_p.y;
                double dx_rotated = dx * std::cos(delta_yaw) - dy * std::sin(delta_yaw);
                double dy_rotated = dx * std::sin(delta_yaw) + dy * std::cos(delta_yaw);

                double centroid_place_x = BOARD_ORIGIN_X_ - goal.place_grid_center.row * GRID_SIZE_;
                double centroid_place_y = BOARD_ORIGIN_Y_ - goal.place_grid_center.col * GRID_SIZE_;
                double place_x = centroid_place_x + dx_rotated + std::cos(place_yaw) * tcp_place_offset_x_ - std::sin(place_yaw) * tcp_place_offset_y_;
                double place_y = centroid_place_y + dy_rotated + std::sin(place_yaw) * tcp_place_offset_x_ + std::cos(place_yaw) * tcp_place_offset_y_;

                goal.place_pose = makeDownPoseBase(place_x, place_y, PLACE_Z_ + tcp_place_offset_z_, place_yaw);
                tasks_.push(goal);
            }
        }
        last_commanded_yaw_ = ref_yaw;

        if (!tasks_.empty())
        {
            publishBusy(true);
            state_ = State::TAKE_NEXT_TASK;
        }
    }

    void publishBusy(bool busy)
    {
        std_msgs::Bool msg;
        msg.data = busy;
        status_pub_.publish(msg);
    }

    void controlLoop(const ros::TimerEvent &)
    {
        switch (state_)
        {
        case State::IDLE:
            return;
        case State::TAKE_NEXT_TASK:
            if (tasks_.empty())
            {
                state_ = State::FINISH;
                return;
            }
            current_task_ = tasks_.front();
            tasks_.pop();
            state_ = State::MOVE_TO_PICK_HOVER;
            return;

        case State::MOVE_TO_PICK_HOVER:

            // 【新增】：强制同步最新物理状态，防止 IK 乱翻转
            move_group_->setStartStateToCurrentState();

            // 高效的 PTP 关节运动前往 Hover 点，轨迹为柔顺的弧线
            ROS_INFO("[CTRL] Pilz [PTP] moving to Pick Hover...");
            if (!executePilzMotion(hoverFrom(current_task_.pick_pose), "PTP"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::EXECUTE_PICK;
            return;

        case State::EXECUTE_PICK:
            // 严格的 LIN 笛卡尔直线下降！
            ROS_INFO("[CTRL] Pilz [LIN] moving strictly DOWN to Pick...");
            if (!executePilzMotion(current_task_.pick_pose, "LIN"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            setSuction(true);

            // 直线上升抽出，不偏不倚
            if (!executePilzMotion(hoverFrom(current_task_.pick_pose), "LIN"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            // 【新增】：给机械臂 0.2 秒物理停稳的时间，然后重置状态起点
            ros::Duration(0.2).sleep();
            move_group_->setStartStateToCurrentState();
            state_ = State::MOVE_TO_PLACE_HOVER;
            return;

        case State::MOVE_TO_PLACE_HOVER:
            move_group_->setStartStateToCurrentState();
            ROS_INFO("[CTRL] Pilz [PTP] moving to Place Hover...");
            if (!executePilzMotion(hoverFrom(current_task_.place_pose), "PTP"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::EXECUTE_PLACE;
            return;

        case State::EXECUTE_PLACE:
            ROS_INFO("[CTRL] Pilz [LIN] moving strictly DOWN to Place...");
            if (!executePilzMotion(current_task_.place_pose, "LIN"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            setSuction(false);

            if (!executePilzMotion(hoverFrom(current_task_.place_pose), "LIN"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::TAKE_NEXT_TASK;
            return;

        case State::FINISH:
            ROS_INFO("[CTRL] Task list finished successfully!");
            publishBusy(false);
            state_ = State::IDLE;
            return;
        }
    }
};

XarmTetrisController *g_controller_ptr = nullptr;

void sigintHandler(int sig)
{
    if (g_controller_ptr)
        g_controller_ptr->emergencyStop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_controller_node", ros::init_options::NoSigintHandler);
    ros::AsyncSpinner spinner(2); // MoveIt 需要 Spinner
    spinner.start();

    XarmTetrisController controller;
    g_controller_ptr = &controller;
    signal(SIGINT, sigintHandler);

    ros::waitForShutdown();
    return 0;
}