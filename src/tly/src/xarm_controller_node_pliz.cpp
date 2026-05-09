#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

#include <xarm_msgs/SetDigitalIO.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <XmlRpcValue.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <string>
#include <vector>
#include <limits>
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
    GridCenter place_grid_center;
    geometry_msgs::Pose pick_pose;
    geometry_msgs::Pose place_pose;
};

class XarmTetrisController
{
public:
    XarmTetrisController() : pnh_("~"), tf_listener_(tf_buffer_)
    {
        loadParams();
        velocity_scale_ = std::max(0.01, std::min(velocity_scale_, max_velocity_scale_allowed_));
        acceleration_scale_ = std::max(0.01, std::min(acceleration_scale_, max_acceleration_scale_allowed_));
        ROS_WARN("[CTRL][SPEED] Pilz velocity_scale=%.3f acceleration_scale=%.3f (caps %.3f / %.3f)",
                 velocity_scale_, acceleration_scale_, max_velocity_scale_allowed_, max_acceleration_scale_allowed_);
        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &XarmTetrisController::cameraInfoCallback, this);
        ROS_INFO("[CTRL] Waiting for CameraInfo...");
        sensor_msgs::CameraInfoConstPtr cam_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));
        if (cam_msg)
            cameraInfoCallback(cam_msg);

        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>(io_service_);
        plan_sub_ = nh_.subscribe(plan_topic_, 1, &XarmTetrisController::planCallback, this);
        status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
        publishBusy(false);

        ROS_INFO("[CTRL] Initializing MoveGroup for 'xarm6'...");
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface("xarm6"));
        // Do not force a named planning pipeline here.
        // In this xArm MoveIt launch, Pilz is loaded as the default/single pipeline;
        // requesting the name "pilz_industrial_motion_planner" causes move_group to reject it.
        move_group_->setPlanningPipelineId("");
        move_group_->setEndEffectorLink(eef_link_);
        move_group_->setPoseReferenceFrame(base_frame_);

        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

        control_timer_ = nh_.createTimer(ros::Duration(0.05), &XarmTetrisController::controlLoop, this);
        ROS_INFO("[CTRL] Xarm Pilz Controller Ready! (Anti-C31 & Smart Guard Active)");
    }

    void emergencyStop()
    {
        if (move_group_)
            move_group_->stop();
    }

private:
    enum class State
    {
        IDLE,
        TAKE_NEXT_TASK,
        MOVE_TO_PICK_HOVER,
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
    ros::ServiceClient io_client_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    State state_ = State::IDLE;
    std::queue<TaskGoal> tasks_;
    TaskGoal current_task_;

    std::string plan_topic_ = "/tetris_plan", status_topic_ = "/robot_status";
    std::string io_service_ = "/xarm/set_controller_dout", camera_info_topic_ = "/camera/color/camera_info";
    std::string table_frame_ = "table_frame", base_frame_ = "link_base", eef_link_ = "link_tcp";
    std::string camera_frame_ = "camera_color_optical_frame";

    double velocity_scale_ = 0.04, acceleration_scale_ = 0.03;
    double max_velocity_scale_allowed_ = 0.08, max_acceleration_scale_allowed_ = 0.06;
    double GRID_SIZE_ = 0.017, BOARD_ORIGIN_X_ = 0.0, BOARD_ORIGIN_Y_ = 0.0;
    int board_rows_ = 14, board_cols_ = 10;
    bool board_centers_loaded_ = false, place_z_map_loaded_ = false;
    std::vector<std::vector<cv::Vec3d>> board_centers_table_;
    std::vector<std::vector<double>> place_z_map_;

    double PICK_Z_ = 0.0, PLACE_Z_ = 0.0, HOVER_Z_ = 0.05;
    double tcp_pick_offset_z_ = 0.0, tcp_place_offset_z_ = 0.0;
    double tcp_pick_offset_x_ = 0.0, tcp_pick_offset_y_ = 0.0;
    double tcp_place_offset_x_ = 0.0, tcp_place_offset_y_ = 0.0;

    int max_tasks_per_plan_ = 1;
    bool enable_joint_jump_guard_ = false; // 默认不按“总旋转角”硬拦截 PTP，只限速/限加速度+检查轨迹突变
    bool enable_trajectory_guard_ = true;
    double max_ptp_big_joint_jump_rad_ = 2.80;
    double max_ptp_wrist_joint_jump_rad_ = 3.20;
    double warn_ptp_joint_jump_rad_ = 1.20;
    double max_lin_branch_jump_rad_ = 0.80;
    double max_traj_segment_joint_delta_rad_ = 0.35;
    double place_release_z_margin_ = 0.002;

    // Loaded transport: after suction is on, keep roll/pitch fixed to the pick pose.
    // Yaw is allowed to change to the strategy target yaw. This reduces soft corrugated-tube twisting.
    bool lock_loaded_roll_pitch_ = true;
    bool use_loaded_lin_transport_ = true;
    // Hover height is controlled only by /tetris/HOVER_Z.
    // No separate loaded/approach hover parameters are used.

    bool require_board_map_ = true;
    bool require_place_z_map_ = true;

    bool use_true_pick_plane_ = true;
    bool pick_plane_loaded_ = false;
    tf2::Vector3 pick_plane_point_base_{0.0, 0.0, 0.0};
    tf2::Vector3 pick_plane_normal_base_{0.0, 0.0, 1.0};

    bool use_pick_homography_ = true;
    bool pick_homography_loaded_ = false;
    double pick_homography_H_[9] = {0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0};

    // Legacy affine correction: kept as fallback only. If homography is loaded, homography wins.
    bool use_pick_affine_correction_ = true;
    bool pick_affine_loaded_ = false;
    double pick_affine_cx_ = 0.0;
    double pick_affine_cy_ = 0.0;
    double pick_affine_x_[3] = {0.0, 0.0, 0.0};
    double pick_affine_y_[3] = {0.0, 0.0, 0.0};

    int suction_io_num_ = 1;
    bool camera_info_ready_ = false;
    cv::Mat P_;
    geometry_msgs::TransformStamped observation_cam_to_table_;
    geometry_msgs::TransformStamped observation_cam_to_base_;
    geometry_msgs::TransformStamped observation_base_to_table_;

    // ==========================================================
    // 核心安全系统重建：智能区分 PTP 跨域移动 和 LIN 直线运动
    // ==========================================================
    bool isSameIKBranch(const robot_state::RobotState &s1, const robot_state::RobotState &s2, const std::string &label) const
    {
        const robot_state::JointModelGroup *jmg = s1.getJointModelGroup("xarm6");
        std::vector<double> j1, j2;
        s1.copyJointGroupPositions(jmg, j1);
        s2.copyJointGroupPositions(jmg, j2);
        for (size_t i = 0; i < j1.size(); ++i)
        {
            // Target 和 Hover 在物理空间只差 5 厘米！如果逆解跳变超过 0.8 rad，绝对是跨分支了，必须拦截。
            double diff = std::abs(j1[i] - j2[i]);
            if (diff > max_lin_branch_jump_rad_)
            {
                ROS_ERROR("[SAFETY] LIN %s: Hover->Target Branch jump on Joint %zu (diff %.2f > %.2f). Move blocked.", label.c_str(), i + 1, diff, max_lin_branch_jump_rad_);
                return false;
            }
        }
        return true;
    }

    bool isSafePTPJump(const robot_state::RobotState &s1, const robot_state::RobotState &s2, const std::string &label) const
    {
        const robot_state::JointModelGroup *jmg = s1.getJointModelGroup("xarm6");
        std::vector<double> j1, j2;
        s1.copyJointGroupPositions(jmg, j1);
        s2.copyJointGroupPositions(jmg, j2);

        for (size_t i = 0; i < j1.size(); ++i)
        {
            double diff = std::abs(j1[i] - j2[i]);

            // 重要：PTP 跨区域移动时，总旋转角大并不等于危险；危险主要来自速度/加速度过大、
            // 轨迹中间有突变、或者规划器选了绕腕/绕底座的奇怪路径。
            // 所以默认只警告较大的总角度变化，不硬拦截；真正的硬保护交给速度/加速度缩放
            // 和 validatePlanTrajectory() 的“轨迹段突变”检查。
            if (diff > warn_ptp_joint_jump_rad_)
            {
                ROS_WARN("[SAFETY] PTP %s: joint%zu total move %.2f rad. Not blocking; limited by vel/acc and trajectory guard.",
                         label.c_str(), i + 1, diff);
            }

            if (enable_joint_jump_guard_)
            {
                double limit = (i < 3) ? max_ptp_big_joint_jump_rad_ : max_ptp_wrist_joint_jump_rad_;
                if (diff > limit)
                {
                    ROS_ERROR("[SAFETY] PTP %s rejected: joint%zu total move %.2f rad > %.2f rad. enable_joint_jump_guard=true.",
                              label.c_str(), i + 1, diff, limit);
                    return false;
                }
            }
        }
        return true;
    }

    bool solveIKForPose(const geometry_msgs::Pose &pose, robot_state::RobotState &state, const std::string &tag) const
    {
        const robot_state::JointModelGroup *jmg = state.getJointModelGroup("xarm6");
        bool ok = state.setFromIK(jmg, pose, eef_link_, 0.40);
        if (!ok)
            ROS_ERROR("[CTRL][IK] Failed for %s, eef=%s, xyz=(%.4f, %.4f, %.4f)", tag.c_str(), eef_link_.c_str(), pose.position.x, pose.position.y, pose.position.z);
        return ok;
    }

    bool moveToHoverSameIKBranch(const geometry_msgs::Pose &target_pose, const std::string &label)
    {
        geometry_msgs::Pose hover_pose = hoverFromTarget(target_pose);

        // 1. 求目标抓取点的 IK
        robot_state::RobotState target_state = *move_group_->getCurrentState();
        if (!solveIKForPose(target_pose, target_state, label + " target"))
            return false;

        // 2. 将此 IK 传给 Hover 作为种子，求解上方 5cm 的姿态
        robot_state::RobotState hover_state = target_state;
        if (!solveIKForPose(hover_pose, hover_state, label + " hover"))
            return false;

        // 3. 检查这 5 厘米的下降是否会导致分支跳变（保证 LIN 绝对安全）
        if (!isSameIKBranch(target_state, hover_state, label))
            return false;

        // 4. 检查当前位置飞跃到 Hover 点，是否需要扭曲 360 度（保证 PTP 绝对安全）
        robot_state::RobotStatePtr current_state = move_group_->getCurrentState();
        if (!isSafePTPJump(*current_state, hover_state, label))
            return false;

        move_group_->setPlannerId("PTP");
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        move_group_->setJointValueTarget(hover_state);
        ROS_INFO("[CTRL] Pilz [PTP] moving to %s Hover...", label.c_str());
        return planAndExecuteCurrentTarget("PTP", label + " hover");
    }

    bool validatePlanTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan, const std::string &label) const
    {
        if (!enable_trajectory_guard_)
            return true;
        const auto &jt = plan.trajectory_.joint_trajectory;
        if (jt.points.empty())
            return true;
        for (size_t k = 1; k < jt.points.size(); ++k)
        {
            const auto &a = jt.points[k - 1].positions;
            const auto &b = jt.points[k].positions;
            size_t n = std::min(a.size(), b.size());
            for (size_t j = 0; j < n; ++j)
            {
                double d = std::abs(b[j] - a[j]);
                if (d > max_traj_segment_joint_delta_rad_)
                {
                    ROS_ERROR("[SAFETY] Trajectory %s rejected: segment %zu joint%zu jump %.3f rad > %.3f rad",
                              label.c_str(), k, j + 1, d, max_traj_segment_joint_delta_rad_);
                    return false;
                }
            }
        }
        return true;
    }

    bool planAndExecuteCurrentTarget(const std::string &planner_id, const std::string &label)
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("[CTRL] Pilz %s Planning Failed for %s!", planner_id.c_str(), label.c_str());
            return false;
        }
        if (!validatePlanTrajectory(my_plan, label))
        {
            move_group_->stop();
            return false;
        }
        bool ok = (move_group_->execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok)
            move_group_->stop();
        return ok;
    }

    bool executePilzMotion(const geometry_msgs::Pose &target_pose, const std::string &planner_id, const std::string &label)
    {
        move_group_->setPlannerId(planner_id);
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        move_group_->setPoseReferenceFrame(base_frame_);
        move_group_->setPoseTarget(target_pose, eef_link_);
        return planAndExecuteCurrentTarget(planner_id, label);
    }
    // ==========================================================

    geometry_msgs::Pose makeDownPoseBase(double x, double y, double z, double yaw_rad) const
    {
        geometry_msgs::PoseStamped pt_table, pt_base;
        pt_table.header.frame_id = table_frame_;
        pt_table.pose.position.x = x;
        pt_table.pose.position.y = y;
        pt_table.pose.position.z = z;
        tf2::Quaternion q;

        // 【终极武器】：加入 2 度 (0.035 rad) 的 Pitch 微倾！
        // 彻底破坏 J5=0 奇点，让 KDL 永远有解。因为 MoveIt 针对 link_tcp 控制，所以物理坐标 0 误差！
        q.setRPY(M_PI, 0.035, yaw_rad);

        pt_table.pose.orientation = tf2::toMsg(q);
        try
        {
            tf_buffer_.transform(pt_table, pt_base, base_frame_, ros::Duration(1.0));
            return pt_base.pose;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] TF transform table->base failed in makeDownPoseBase: %s", ex.what());
            geometry_msgs::Pose invalid;
            invalid.position.x = std::numeric_limits<double>::quiet_NaN();
            invalid.position.y = std::numeric_limits<double>::quiet_NaN();
            invalid.position.z = std::numeric_limits<double>::quiet_NaN();
            return invalid;
        }
    }

    bool poseFinite(const geometry_msgs::Pose &p) const
    {
        return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
               std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
               std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
    }

    geometry_msgs::Pose hoverFromTarget(const geometry_msgs::Pose &target_pose) const
    {
        geometry_msgs::Pose hover = target_pose;
        // HOVER_Z_ is the absolute hover height in the same frame as the generated base pose.
        // If the target is already higher than HOVER_Z_, keep the target height to avoid moving downward.
        hover.position.z = std::max(HOVER_Z_, target_pose.position.z);
        return hover;
    }

    void getPoseRPY(const geometry_msgs::Pose &pose, double &roll, double &pitch, double &yaw) const
    {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    geometry_msgs::Pose lockRollPitchKeepYaw(const geometry_msgs::Pose &target_pose,
                                             const geometry_msgs::Pose &roll_pitch_ref_pose,
                                             const std::string &label) const
    {
        double ref_r = 0.0, ref_p = 0.0, ref_y = 0.0;
        double tgt_r = 0.0, tgt_p = 0.0, tgt_y = 0.0;
        getPoseRPY(roll_pitch_ref_pose, ref_r, ref_p, ref_y);
        getPoseRPY(target_pose, tgt_r, tgt_p, tgt_y);

        geometry_msgs::Pose out = target_pose;
        tf2::Quaternion q;
        q.setRPY(ref_r, ref_p, tgt_y);
        out.orientation = tf2::toMsg(q);

        ROS_INFO("[CTRL][LOADED_RP_LOCK] %s: roll %.2f->%.2f deg, pitch %.2f->%.2f deg, yaw kept %.2f deg",
                 label.c_str(),
                 tgt_r * 180.0 / M_PI, ref_r * 180.0 / M_PI,
                 tgt_p * 180.0 / M_PI, ref_p * 180.0 / M_PI,
                 tgt_y * 180.0 / M_PI);
        return out;
    }

    bool checkHoverTargetSameIKBranch(const geometry_msgs::Pose &target_pose,
                                      const geometry_msgs::Pose &hover_pose,
                                      const std::string &label)
    {
        robot_state::RobotState target_state = *move_group_->getCurrentState();
        if (!solveIKForPose(target_pose, target_state, label + " target"))
            return false;

        robot_state::RobotState hover_state = target_state;
        if (!solveIKForPose(hover_pose, hover_state, label + " hover"))
            return false;

        return isSameIKBranch(target_state, hover_state, label);
    }

    bool moveLoadedToHoverLin(const geometry_msgs::Pose &target_pose, const std::string &label)
    {
        geometry_msgs::Pose hover_pose = hoverFromTarget(target_pose);

        if (!checkHoverTargetSameIKBranch(target_pose, hover_pose, label))
            return false;

        ROS_INFO("[CTRL] Pilz [LIN] loaded transport to %s Hover with roll/pitch locked, yaw allowed.", label.c_str());
        return executePilzMotion(hover_pose, "LIN", label + " loaded transport hover");
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

    double normalizeAngleRad(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    bool xmlToDouble(XmlRpc::XmlRpcValue &v, double &out) const
    {
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            out = static_cast<double>(v);
            return true;
        }
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            out = static_cast<int>(v);
            return true;
        }
        return false;
    }

    bool readVec3(XmlRpc::XmlRpcValue &v, cv::Vec3d &out) const
    {
        if (v.getType() != XmlRpc::XmlRpcValue::TypeArray || v.size() < 3)
            return false;
        double x, y, z;
        if (!xmlToDouble(v[0], x) || !xmlToDouble(v[1], y) || !xmlToDouble(v[2], z))
            return false;
        out = cv::Vec3d(x, y, z);
        return true;
    }

    bool loadBoardCenters14x10()
    {
        XmlRpc::XmlRpcValue centers;
        if (!nh_.getParam("/tetris/BOARD_CENTERS_14x10_TABLE", centers) || centers.getType() != XmlRpc::XmlRpcValue::TypeArray || centers.size() <= 0)
            return false;
        board_rows_ = centers.size();
        board_cols_ = centers[0].size();
        board_centers_table_.assign(board_rows_, std::vector<cv::Vec3d>(board_cols_));
        for (int r = 0; r < board_rows_; ++r)
        {
            for (int c = 0; c < board_cols_; ++c)
            {
                if (!readVec3(centers[r][c], board_centers_table_[r][c]))
                    return false;
            }
        }
        return true;
    }

    bool loadPlaceZMap14x10()
    {
        XmlRpc::XmlRpcValue zmap;
        if (!nh_.getParam("/tetris/PLACE_Z_MAP_14x10", zmap) || zmap.getType() != XmlRpc::XmlRpcValue::TypeArray || zmap.size() <= 0)
            return false;
        int rows = zmap.size(), cols = zmap[0].size();
        place_z_map_.assign(rows, std::vector<double>(cols, PLACE_Z_));
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                if (!xmlToDouble(zmap[r][c], place_z_map_[r][c]))
                    return false;
            }
        }
        return true;
    }

    bool readVector3List(XmlRpc::XmlRpcValue &value, tf2::Vector3 &out) const
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
            return false;
        double x = 0.0, y = 0.0, z = 0.0;
        if (!xmlToDouble(value[0], x) || !xmlToDouble(value[1], y) || !xmlToDouble(value[2], z))
            return false;
        out = tf2::Vector3(x, y, z);
        return true;
    }

    bool readDoubleArray(XmlRpc::XmlRpcValue &value, std::vector<double> &out) const
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
            return false;
        out.clear();
        for (int i = 0; i < value.size(); ++i)
        {
            double x = 0.0;
            if (!xmlToDouble(value[i], x))
                return false;
            out.push_back(x);
        }
        return true;
    }

    bool loadPickPlaneIfAvailable()
    {
        if (!use_true_pick_plane_)
            return false;

        XmlRpc::XmlRpcValue plane;
        if (!nh_.getParam("/tetris/PICK_SURFACE_PLANE_BASE", plane) ||
            plane.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !plane.hasMember("point") || !plane.hasMember("normal"))
        {
            ROS_WARN("[CTRL] /tetris/PICK_SURFACE_PLANE_BASE not found, fallback to flat PICK_Z plane.");
            return false;
        }

        tf2::Vector3 p, n;
        if (!readVector3List(plane["point"], p) || !readVector3List(plane["normal"], n) || n.length() < 1e-9)
        {
            ROS_WARN("[CTRL] Invalid PICK_SURFACE_PLANE_BASE, fallback to flat PICK_Z plane.");
            return false;
        }

        pick_plane_point_base_ = p;
        pick_plane_normal_base_ = n.normalized();
        ROS_INFO("[CTRL] Loaded true pick plane in %s: point=(%.6f, %.6f, %.6f), normal=(%.6f, %.6f, %.6f)",
                 base_frame_.c_str(),
                 pick_plane_point_base_.x(), pick_plane_point_base_.y(), pick_plane_point_base_.z(),
                 pick_plane_normal_base_.x(), pick_plane_normal_base_.y(), pick_plane_normal_base_.z());
        return true;
    }

    bool loadPickHomographyIfAvailable()
    {
        if (!use_pick_homography_)
            return false;

        XmlRpc::XmlRpcValue hg;
        if (!nh_.getParam("/tetris/PICK_HOMOGRAPHY", hg) || hg.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[CTRL] /tetris/PICK_HOMOGRAPHY not found, fallback to affine/true-plane XY.");
            return false;
        }

        if (hg.hasMember("enabled"))
        {
            try
            {
                bool enabled = static_cast<bool>(hg["enabled"]);
                if (!enabled)
                {
                    ROS_WARN("[CTRL] PICK_HOMOGRAPHY exists but enabled=false, fallback to affine/true-plane XY.");
                    return false;
                }
            }
            catch (...)
            {
            }
        }

        if (!hg.hasMember("matrix"))
        {
            ROS_WARN("[CTRL] PICK_HOMOGRAPHY missing matrix field.");
            return false;
        }

        std::vector<double> H;
        if (!readDoubleArray(hg["matrix"], H) || H.size() < 9)
        {
            ROS_WARN("[CTRL] Invalid PICK_HOMOGRAPHY matrix. Need 9 numbers.");
            return false;
        }

        for (int i = 0; i < 9; ++i)
            pick_homography_H_[i] = H[i];

        double rms = -1.0;
        if (hg.hasMember("rms_m"))
            xmlToDouble(hg["rms_m"], rms);

        ROS_INFO("[CTRL] Loaded pick homography: H=[%.9g %.9g %.9g; %.9g %.9g %.9g; %.9g %.9g %.9g], rms=%.4f mm",
                 pick_homography_H_[0], pick_homography_H_[1], pick_homography_H_[2],
                 pick_homography_H_[3], pick_homography_H_[4], pick_homography_H_[5],
                 pick_homography_H_[6], pick_homography_H_[7], pick_homography_H_[8],
                 rms >= 0.0 ? rms * 1000.0 : -1.0);
        return true;
    }

    bool loadPickAffineCorrectionIfAvailable()
    {
        if (!use_pick_affine_correction_)
            return false;

        XmlRpc::XmlRpcValue corr;
        if (!nh_.getParam("/tetris/PICK_AFFINE_CORRECTION", corr) || corr.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[CTRL] /tetris/PICK_AFFINE_CORRECTION not found, no affine XY correction.");
            return false;
        }

        if (corr.hasMember("enabled"))
        {
            try
            {
                bool enabled = static_cast<bool>(corr["enabled"]);
                if (!enabled)
                    return false;
            }
            catch (...)
            {
            }
        }

        if (!corr.hasMember("pixel_center") || !corr.hasMember("coeff_x") || !corr.hasMember("coeff_y"))
        {
            ROS_WARN("[CTRL] PICK_AFFINE_CORRECTION missing pixel_center/coeff_x/coeff_y.");
            return false;
        }

        std::vector<double> pc, cx, cy;
        if (!readDoubleArray(corr["pixel_center"], pc) || !readDoubleArray(corr["coeff_x"], cx) ||
            !readDoubleArray(corr["coeff_y"], cy) || pc.size() < 2 || cx.size() < 3 || cy.size() < 3)
        {
            ROS_WARN("[CTRL] Invalid PICK_AFFINE_CORRECTION values.");
            return false;
        }

        pick_affine_cx_ = pc[0];
        pick_affine_cy_ = pc[1];
        for (int i = 0; i < 3; ++i)
        {
            pick_affine_x_[i] = cx[i];
            pick_affine_y_[i] = cy[i];
        }

        ROS_INFO("[CTRL] Loaded pick affine correction: center=(%.2f, %.2f), x=(%.9f, %.9f, %.9f), y=(%.9f, %.9f, %.9f)",
                 pick_affine_cx_, pick_affine_cy_,
                 pick_affine_x_[0], pick_affine_x_[1], pick_affine_x_[2],
                 pick_affine_y_[0], pick_affine_y_[1], pick_affine_y_[2]);
        return true;
    }

    void loadParams()
    {
        pnh_.param("velocity_scale", velocity_scale_, velocity_scale_);
        pnh_.param("acceleration_scale", acceleration_scale_, acceleration_scale_);
        pnh_.param("max_velocity_scale_allowed", max_velocity_scale_allowed_, max_velocity_scale_allowed_);
        pnh_.param("max_acceleration_scale_allowed", max_acceleration_scale_allowed_, max_acceleration_scale_allowed_);
        nh_.param("/tetris/PICK_Z", PICK_Z_, PICK_Z_);
        nh_.param("/tetris/PLACE_Z", PLACE_Z_, PLACE_Z_);
        nh_.param("/tetris/HOVER_Z", HOVER_Z_, HOVER_Z_);
        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X_, 0.3563);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y_, -0.3971);
        nh_.param("/tetris/GRID_SIZE", GRID_SIZE_, 0.0202);
        nh_.param("/tetris/calibration_frames/eef_frame", eef_link_, eef_link_);

        pnh_.param("tcp_pick_offset_z", tcp_pick_offset_z_, tcp_pick_offset_z_);
        pnh_.param("tcp_place_offset_z", tcp_place_offset_z_, tcp_place_offset_z_);
        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        pnh_.param("tcp_place_offset_x", tcp_place_offset_x_, tcp_place_offset_x_);
        pnh_.param("tcp_place_offset_y", tcp_place_offset_y_, tcp_place_offset_y_);

        pnh_.param("max_tasks_per_plan", max_tasks_per_plan_, max_tasks_per_plan_);
        pnh_.param("enable_joint_jump_guard", enable_joint_jump_guard_, enable_joint_jump_guard_);
        pnh_.param("enable_trajectory_guard", enable_trajectory_guard_, enable_trajectory_guard_);
        pnh_.param("max_ptp_big_joint_jump_rad", max_ptp_big_joint_jump_rad_, max_ptp_big_joint_jump_rad_);
        pnh_.param("max_ptp_wrist_joint_jump_rad", max_ptp_wrist_joint_jump_rad_, max_ptp_wrist_joint_jump_rad_);
        pnh_.param("warn_ptp_joint_jump_rad", warn_ptp_joint_jump_rad_, warn_ptp_joint_jump_rad_);
        pnh_.param("max_lin_branch_jump_rad", max_lin_branch_jump_rad_, max_lin_branch_jump_rad_);
        pnh_.param("max_traj_segment_joint_delta_rad", max_traj_segment_joint_delta_rad_, max_traj_segment_joint_delta_rad_);
        pnh_.param("place_release_z_margin", place_release_z_margin_, place_release_z_margin_);
        pnh_.param("lock_loaded_roll_pitch", lock_loaded_roll_pitch_, lock_loaded_roll_pitch_);
        pnh_.param("use_loaded_lin_transport", use_loaded_lin_transport_, use_loaded_lin_transport_);
        pnh_.param("require_board_map", require_board_map_, require_board_map_);
        pnh_.param("require_place_z_map", require_place_z_map_, require_place_z_map_);
        pnh_.param("camera_frame", camera_frame_, camera_frame_);
        pnh_.param("use_true_pick_plane", use_true_pick_plane_, use_true_pick_plane_);
        pnh_.param("use_pick_homography", use_pick_homography_, use_pick_homography_);
        pnh_.param("use_pick_affine_correction", use_pick_affine_correction_, use_pick_affine_correction_);

        board_centers_loaded_ = loadBoardCenters14x10();
        place_z_map_loaded_ = loadPlaceZMap14x10();
        pick_plane_loaded_ = loadPickPlaneIfAvailable();
        pick_homography_loaded_ = loadPickHomographyIfAvailable();
        pick_affine_loaded_ = pick_homography_loaded_ ? false : loadPickAffineCorrectionIfAvailable();
        ROS_INFO("[CTRL] eef_link=%s camera_frame=%s board_centers=%s place_z_map=%s pick_plane=%s pick_homography=%s pick_affine=%s max_tasks=%d joint_guard=%s loaded_rp_lock=%s loaded_lin=%s hover_z=%.3fm",
                 eef_link_.c_str(), camera_frame_.c_str(),
                 board_centers_loaded_ ? "loaded" : "MISSING",
                 place_z_map_loaded_ ? "loaded" : "MISSING",
                 pick_plane_loaded_ ? "loaded" : "flat_PICK_Z",
                 pick_homography_loaded_ ? "loaded" : "off",
                 pick_affine_loaded_ ? "loaded" : "off",
                 max_tasks_per_plan_,
                 enable_joint_jump_guard_ ? "hard" : "warn_only",
                 lock_loaded_roll_pitch_ ? "on" : "off",
                 use_loaded_lin_transport_ ? "on" : "off",
                 HOVER_Z_);
        if (require_board_map_ && !board_centers_loaded_)
            ROS_ERROR("[CTRL] Required /tetris/BOARD_CENTERS_14x10_TABLE is missing. Controller will refuse plans.");
        if (require_place_z_map_ && !place_z_map_loaded_)
            ROS_ERROR("[CTRL] Required /tetris/PLACE_Z_MAP_14x10 is missing. Controller will refuse plans.");
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        P_ = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 12; ++i)
            P_.at<double>(i / 4, i % 4) = msg->P[i];
        camera_info_ready_ = true;
    }

    bool pixelToNormalizedRay(const PixelPoint &px, cv::Vec3d &ray) const
    {
        if (!camera_info_ready_)
            return false;
        double fx = P_.at<double>(0, 0);
        double fy = P_.at<double>(1, 1);
        double cx = P_.at<double>(0, 2);
        double cy = P_.at<double>(1, 2);
        if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9)
            return false;
        ray = cv::Vec3d((px.u - cx) / fx, (px.v - cy) / fy, 1.0);
        return true;
    }

    bool pixelToFlatPickZTablePoint(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        cv::Vec3d ray_cam;
        if (!pixelToNormalizedRay(px, ray_cam))
            return false;
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
        double denom = ray_table.z();
        if (std::abs(denom) < 1e-9)
            return false;
        double t = (PICK_Z_ - cam_pos.z()) / denom;
        if (t < 0.0)
            return false;
        tf2::Vector3 p = cam_pos + ray_table * t;
        out.x = p.x();
        out.y = p.y();
        out.z = p.z();
        return true;
    }

    bool pixelToTruePickPlaneTablePoint(const PixelPoint &px, geometry_msgs::Point &out_table) const
    {
        cv::Vec3d ray_cam;
        if (!pixelToNormalizedRay(px, ray_cam))
            return false;

        tf2::Quaternion q;
        tf2::fromMsg(observation_cam_to_base_.transform.rotation, q);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 ray_base(
            R[0][0] * ray_cam[0] + R[0][1] * ray_cam[1] + R[0][2] * ray_cam[2],
            R[1][0] * ray_cam[0] + R[1][1] * ray_cam[1] + R[1][2] * ray_cam[2],
            R[2][0] * ray_cam[0] + R[2][1] * ray_cam[1] + R[2][2] * ray_cam[2]);

        tf2::Vector3 cam_pos_base(observation_cam_to_base_.transform.translation.x,
                                  observation_cam_to_base_.transform.translation.y,
                                  observation_cam_to_base_.transform.translation.z);
        double denom = pick_plane_normal_base_.dot(ray_base);
        if (std::abs(denom) < 1e-9)
            return false;
        double t = pick_plane_normal_base_.dot(pick_plane_point_base_ - cam_pos_base) / denom;
        if (t < 0.0)
            return false;

        tf2::Vector3 hit_base = cam_pos_base + ray_base * t;
        geometry_msgs::PointStamped p_base, p_table;
        p_base.header.frame_id = base_frame_;
        p_base.header.stamp = ros::Time(0);
        p_base.point.x = hit_base.x();
        p_base.point.y = hit_base.y();
        p_base.point.z = hit_base.z();
        try
        {
            tf2::doTransform(p_base, p_table, observation_base_to_table_);
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] true pick plane base->table transform failed: %s", ex.what());
            return false;
        }
        out_table = p_table.point;
        return true;
    }

    bool pixelToPickPointTable(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        if (use_true_pick_plane_ && pick_plane_loaded_)
            return pixelToTruePickPlaneTablePoint(px, out);
        return pixelToFlatPickZTablePoint(px, out);
    }

    bool applyPickHomography(const PixelPoint &px, geometry_msgs::Point &p) const
    {
        if (!use_pick_homography_ || !pick_homography_loaded_)
            return false;

        const double u = static_cast<double>(px.u);
        const double v = static_cast<double>(px.v);
        const double *H = pick_homography_H_;
        const double W = H[6] * u + H[7] * v + H[8];
        if (std::abs(W) < 1e-9)
        {
            ROS_ERROR("[CTRL] PICK_HOMOGRAPHY invalid W near zero for pixel=(%d,%d).", px.u, px.v);
            return false;
        }

        // Homography matrix in tetris_config maps image pixel (u,v) directly to table-frame XY.
        // Keep Z from the true pick plane / flat PICK_Z intersection, only replace XY.
        p.x = (H[0] * u + H[1] * v + H[2]) / W;
        p.y = (H[3] * u + H[4] * v + H[5]) / W;
        return true;
    }

    bool applyPickXYCalibration(const PixelPoint &px, geometry_msgs::Point &p) const
    {
        if (applyPickHomography(px, p))
            return true;

        if (!use_pick_affine_correction_ || !pick_affine_loaded_)
            return true;

        double du = static_cast<double>(px.u) - pick_affine_cx_;
        double dv = static_cast<double>(px.v) - pick_affine_cy_;
        double dx = pick_affine_x_[0] + pick_affine_x_[1] * du + pick_affine_x_[2] * dv;
        double dy = pick_affine_y_[0] + pick_affine_y_[1] * du + pick_affine_y_[2] * dv;
        p.x += dx;
        p.y += dy;
        return true;
    }

    cv::Vec3d bilinearBoardCenter(double row, double col) const
    {
        if (!board_centers_loaded_ || board_centers_table_.empty())
            return cv::Vec3d(BOARD_ORIGIN_X_ - row * GRID_SIZE_, BOARD_ORIGIN_Y_ - col * GRID_SIZE_, PLACE_Z_);
        double r = std::max(0.0, std::min(row, static_cast<double>(board_rows_ - 1)));
        double c = std::max(0.0, std::min(col, static_cast<double>(board_cols_ - 1)));
        int r0 = static_cast<int>(std::floor(r)), c0 = static_cast<int>(std::floor(c));
        int r1 = std::min(r0 + 1, board_rows_ - 1), c1 = std::min(c0 + 1, board_cols_ - 1);
        double tr = r - r0, tc = c - c0;
        return board_centers_table_[r0][c0] * ((1.0 - tr) * (1.0 - tc)) + board_centers_table_[r1][c0] * (tr * (1.0 - tc)) + board_centers_table_[r0][c1] * ((1.0 - tr) * tc) + board_centers_table_[r1][c1] * (tr * tc);
    }

    double bilinearPlaceZ(double row, double col) const
    {
        if (!place_z_map_loaded_ || place_z_map_.empty())
            return PLACE_Z_;
        int rows = place_z_map_.size(), cols = place_z_map_[0].size();
        double r = std::max(0.0, std::min(row, static_cast<double>(rows - 1))), c = std::max(0.0, std::min(col, static_cast<double>(cols - 1)));
        int r0 = static_cast<int>(std::floor(r)), c0 = static_cast<int>(std::floor(c));
        int r1 = std::min(r0 + 1, rows - 1), c1 = std::min(c0 + 1, cols - 1);
        double tr = r - r0, tc = c - c0;
        return (1.0 - tr) * (1.0 - tc) * place_z_map_[r0][c0] + tr * (1.0 - tc) * place_z_map_[r1][c0] + (1.0 - tr) * tc * place_z_map_[r0][c1] + tr * tc * place_z_map_[r1][c1];
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty() || !camera_info_ready_)
            return;
        try
        {
            observation_cam_to_table_ = tf_buffer_.lookupTransform(table_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            if (use_true_pick_plane_ && pick_plane_loaded_)
            {
                observation_cam_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
                observation_base_to_table_ = tf_buffer_.lookupTransform(table_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));
            }
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] TF lookup failed in planCallback: %s", ex.what());
            return;
        }

        if ((require_board_map_ && !board_centers_loaded_) || (require_place_z_map_ && !place_z_map_loaded_))
        {
            ROS_ERROR("[CTRL] Refusing plan because required board/place maps are not loaded.");
            return;
        }

        while (!tasks_.empty())
            tasks_.pop();
        int total = msg->data[0];
        if (total <= 0)
            return;
        int stride = (msg->data.size() - 1) / total;
        if (stride < 7 || static_cast<int>(msg->data.size()) < 1 + total * stride)
        {
            ROS_ERROR("[CTRL] Bad plan payload: total=%d stride=%d size=%zu", total, stride, msg->data.size());
            return;
        }
        int exec_total = total;
        if (max_tasks_per_plan_ > 0 && total > max_tasks_per_plan_)
        {
            ROS_WARN("[CTRL][SAFETY] Received %d tasks, executing only first %d task(s).", total, max_tasks_per_plan_);
            exec_total = max_tasks_per_plan_;
        }

        for (int i = 0; i < exec_total; ++i)
        {
            int base_idx = 1 + i * stride;
            TaskGoal goal;
            goal.shape_type = msg->data[base_idx + 0];
            goal.way = msg->data[base_idx + 1];
            goal.place_grid_center.row = msg->data[base_idx + 2] / 4.0;
            goal.place_grid_center.col = msg->data[base_idx + 3] / 4.0;
            goal.pick_pixel.u = msg->data[base_idx + 4];
            goal.pick_pixel.v = msg->data[base_idx + 5];

            double raw_yaw = normalizeAngleRad(msg->data[base_idx + 6] * M_PI / 180.0);
            goal.pick_angle_deg = raw_yaw * 180.0 / M_PI;

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
            if (pixelToPickPointTable(goal.pick_pixel, pick_p) && pixelToPickPointTable(goal.geom_pixel, geom_p))
            {
                if (!applyPickXYCalibration(goal.pick_pixel, pick_p) || !applyPickXYCalibration(goal.geom_pixel, geom_p))
                {
                    ROS_ERROR("[CTRL] Skip task %d: pick XY calibration failed.", i + 1);
                    continue;
                }
                double pick_x = pick_p.x + std::cos(raw_yaw) * tcp_pick_offset_x_ - std::sin(raw_yaw) * tcp_pick_offset_y_;
                double pick_y = pick_p.y + std::sin(raw_yaw) * tcp_pick_offset_x_ + std::cos(raw_yaw) * tcp_pick_offset_y_;
                double pick_z = pick_p.z + tcp_pick_offset_z_;
                goal.pick_pose = makeDownPoseBase(pick_x, pick_y, pick_z, raw_yaw);
                if (!poseFinite(goal.pick_pose))
                {
                    ROS_ERROR("[CTRL] Skip task %d: invalid pick_pose after TF transform.", i + 1);
                    continue;
                }

                double place_yaw = normalizeAngleRad(-goal.way * M_PI / 2.0);
                double delta_yaw = place_yaw - raw_yaw;
                double dx = pick_p.x - geom_p.x, dy = pick_p.y - geom_p.y;
                double dx_rotated = dx * std::cos(delta_yaw) - dy * std::sin(delta_yaw);
                double dy_rotated = dx * std::sin(delta_yaw) + dy * std::cos(delta_yaw);

                cv::Vec3d center = bilinearBoardCenter(goal.place_grid_center.row, goal.place_grid_center.col);
                double place_x = center[0] + dx_rotated + std::cos(place_yaw) * tcp_place_offset_x_ - std::sin(place_yaw) * tcp_place_offset_y_;
                double place_y = center[1] + dy_rotated + std::sin(place_yaw) * tcp_place_offset_x_ + std::cos(place_yaw) * tcp_place_offset_y_;
                double place_z = bilinearPlaceZ(goal.place_grid_center.row, goal.place_grid_center.col) + tcp_place_offset_z_ + place_release_z_margin_;

                goal.place_pose = makeDownPoseBase(place_x, place_y, place_z, place_yaw);
                if (!poseFinite(goal.place_pose))
                {
                    ROS_ERROR("[CTRL] Skip task %d: invalid place_pose after TF transform.", i + 1);
                    continue;
                }
                ROS_INFO("[CTRL][TASK %02d] shape=%d way=%d center=(%.2f, %.2f) pick_table=(%.4f, %.4f, %.4f) place_table=(%.4f, %.4f, %.4f) yaw_pick=%.1f yaw_place=%.1f pick_z_src=%s pick_xy=%s",
                         i + 1, goal.shape_type, goal.way, goal.place_grid_center.row, goal.place_grid_center.col,
                         pick_x, pick_y, pick_z, place_x, place_y, place_z, raw_yaw * 180.0 / M_PI, place_yaw * 180.0 / M_PI,
                         (use_true_pick_plane_ && pick_plane_loaded_) ? "true_plane" : "flat_PICK_Z",
                         pick_homography_loaded_ ? "homography" : ((use_pick_affine_correction_ && pick_affine_loaded_) ? "affine" : "raw_plane"));
                tasks_.push(goal);
            }
        }
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
            if (!moveToHoverSameIKBranch(current_task_.pick_pose, "Pick"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::EXECUTE_PICK;
            return;

        case State::EXECUTE_PICK:
        {
            if (!executePilzMotion(current_task_.pick_pose, "LIN", "Pick down"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            setSuction(true);

            // Once the block is attached to the soft corrugated suction cup, prevent roll/pitch changes.
            // Yaw is still allowed to change to match the target way. This reduces block twist during transport.
            if (lock_loaded_roll_pitch_)
                current_task_.place_pose = lockRollPitchKeepYaw(current_task_.place_pose, current_task_.pick_pose, "place_pose_after_pick");

            geometry_msgs::Pose pick_hover = hoverFromTarget(current_task_.pick_pose);
            if (!executePilzMotion(pick_hover, "LIN", "Pick up loaded"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }

            ros::Duration(0.1).sleep();
            move_group_->setStartStateToCurrentState();
            state_ = State::MOVE_TO_PLACE_HOVER;
            return;
        }

        case State::MOVE_TO_PLACE_HOVER:
        {
            bool ok = false;
            if (use_loaded_lin_transport_)
                ok = moveLoadedToHoverLin(current_task_.place_pose, "Place");
            else
                ok = moveToHoverSameIKBranch(current_task_.place_pose, "Place");

            if (!ok)
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::EXECUTE_PLACE;
            return;
        }

        case State::EXECUTE_PLACE:
        {
            if (!executePilzMotion(current_task_.place_pose, "LIN", "Place down"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            setSuction(false);

            geometry_msgs::Pose place_hover = hoverFromTarget(current_task_.place_pose);
            if (!executePilzMotion(place_hover, "LIN", "Place up"))
            {
                state_ = State::IDLE;
                publishBusy(false);
                return;
            }
            state_ = State::TAKE_NEXT_TASK;
            return;
        }

        case State::FINISH:
            ROS_INFO("[CTRL] All Tasks safely completed!");
            publishBusy(false);
            state_ = State::IDLE;
            return;
        }
    }
};

XarmTetrisController *g_ptr = nullptr;
void sigintHandler(int sig)
{
    if (g_ptr)
        g_ptr->emergencyStop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_controller_node", ros::init_options::NoSigintHandler);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    XarmTetrisController controller;
    g_ptr = &controller;
    signal(SIGINT, sigintHandler);
    ros::waitForShutdown();
    return 0;
}