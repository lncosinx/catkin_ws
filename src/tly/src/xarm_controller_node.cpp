#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <xarm_msgs/SetDigitalIO.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/RobotMsg.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core.hpp>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

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
    std::vector<GridCell> target_cells;
    bool has_target_cells = false;

    bool has_pick_to_geom_offset = false;
    double pick_minus_geom_x = 0.0;
    double pick_minus_geom_y = 0.0;

    geometry_msgs::Pose pick_pose_table;
    geometry_msgs::Pose place_pose_table;
};

class XarmTetrisController
{
public:
    XarmTetrisController()
        : pnh_("~"), tf_listener_(tf_buffer_)
    {
        loadParams();

        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &XarmTetrisController::cameraInfoCallback, this);
        robot_state_sub_ = nh_.subscribe("/xarm/xarm_states", 10, &XarmTetrisController::robotStateCallback, this);

        ROS_INFO("[CTRL] Waiting for CameraInfo on %s ...", camera_info_topic_.c_str());
        sensor_msgs::CameraInfoConstPtr cam_msg =
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));
        if (cam_msg)
            cameraInfoCallback(cam_msg);
        else
            ROS_WARN("[CTRL] CameraInfo not received within 5 seconds.");

        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>(io_service_);
        set_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_mode");
        set_state_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("/xarm/set_state");
        move_line_client_ = nh_.serviceClient<xarm_msgs::Move>("/xarm/move_line");

        ROS_INFO("[CTRL] Waiting for xArm native services...");
        set_mode_client_.waitForExistence();
        set_state_client_.waitForExistence();
        move_line_client_.waitForExistence();
        io_client_.waitForExistence();

        setNativeModeOnce();
        nh_.setParam("/xarm/wait_for_finish", xarm_wait_for_finish_);

        plan_sub_ = nh_.subscribe(plan_topic_, 1, &XarmTetrisController::planCallback, this);
        status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
        publishBusy(false);

        control_timer_ = nh_.createTimer(ros::Duration(control_period_), &XarmTetrisController::controlLoop, this);

        ROS_INFO("[CTRL] SIMPLE move_line controller ready: eef=%s, fixed_rp=(%.3f, %.3f), hover_z=%.4f, max_tasks=%d",
                 eef_frame_.c_str(), fixed_roll_rad_, fixed_pitch_rad_, HOVER_Z_, max_tasks_per_plan_);
        ROS_INFO("[CTRL] pick yaw: homography_probe=%.1f px, v_sign=%.0f, offset=%.2f deg",
                 yaw_homography_probe_px_, yaw_homography_v_sign_, pick_yaw_offset_rad_ * 180.0 / M_PI);
        ROS_INFO("[CTRL] place shake: %s cycles=%d dx=%.1fmm dy=%.1fmm v=%.1f a=%.1f",
                 enable_place_shake_ ? "ON" : "OFF", place_shake_cycles_,
                 place_shake_dx_m_ * 1000.0, place_shake_dy_m_ * 1000.0,
                 place_shake_speed_mm_s_, place_shake_acc_mm_s2_);
    }

    void emergencyStop()
    {
        ROS_ERROR("[CTRL] Ctrl+C detected. Sending xArm STOP state=4.");
        xarm_msgs::SetInt16 srv;
        srv.request.data = 4;
        set_state_client_.call(srv);
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber plan_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber robot_state_sub_;
    ros::Publisher status_pub_;
    ros::Timer control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::ServiceClient io_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient set_state_client_;
    ros::ServiceClient move_line_client_;

    State state_ = State::IDLE;
    std::queue<TaskGoal> tasks_;
    TaskGoal current_task_;

    std::string plan_topic_ = "/tetris_plan";
    std::string status_topic_ = "/robot_status";
    std::string io_service_ = "/xarm/set_controller_dout";
    std::string camera_info_topic_ = "/camera/color/camera_info";
    std::string base_frame_ = "link_base";
    std::string camera_frame_ = "camera_color_optical_frame";
    std::string eef_frame_ = "link_tcp";

    // board_frame 在 base 的常量位姿（由 /tetris/BOARD_POSE_BASE 派生），取代旧 table_frame TF。
    bool board_pose_loaded_ = false;
    tf2::Transform board_to_base_; // board_frame -> base
    tf2::Transform base_to_board_; // 逆

    double control_period_ = 0.05;
    bool xarm_wait_for_finish_ = true;
    bool verify_native_xyz_after_motion_ = true;
    double native_xyz_tolerance_mm_ = 6.0;
    double native_verify_timeout_s_ = 1.0; // 校验位置时最长等待新上报到位的时间
    double native_verify_poll_hz_ = 50.0;  // 校验轮询频率
    // 腕部 yaw 软限位（sim_yaw / TCP base-yaw 空间的近似 J6 上界）。A/B 方案选择以"腕部转角最小"
    // 为目标、此限位为约束：预测 |腕角| 超过它的方案才被排除。硬限 ±2π，默认留余量。
    double wrist_soft_limit_rad_ = 5.5; // ≈315°

    // 坐标 / 标定参数
    double GRID_SIZE_ = 0.0202;
    double BOARD_ORIGIN_X_ = 0.3563;
    double BOARD_ORIGIN_Y_ = -0.3971;
    double PICK_Z_ = 0.0;
    double PLACE_Z_ = 0.0;
    double HOVER_Z_ = 0.08;

    bool use_true_pick_plane_ = true;
    bool pick_plane_loaded_ = false;
    tf2::Vector3 pick_plane_point_base_{0.0, 0.0, 0.0};
    tf2::Vector3 pick_plane_normal_base_{0.0, 0.0, 1.0};

    bool use_pick_homography_ = true;
    bool pick_homography_loaded_ = false;
    double pick_homography_H_[9] = {0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0};

    bool use_board_map_ = true;
    bool board_centers_loaded_ = false;
    bool place_z_map_loaded_ = false;
    bool use_cell_max_place_z_ = true;
    bool require_board_map_ = true;
    bool require_place_z_map_ = true;
    int board_rows_ = 14;
    int board_cols_ = 10;
    std::vector<std::vector<geometry_msgs::Point>> board_centers_;
    std::vector<std::vector<double>> place_z_map_;

    double tcp_pick_offset_x_ = 0.0;
    double tcp_pick_offset_y_ = 0.0;
    double tcp_pick_offset_z_ = 0.0;
    double pick_yaw_offset_rad_ = 0.0;
    double yaw_homography_probe_px_ = 30.0;
    double yaw_homography_v_sign_ = 1.0; // +1: image v down, -1: if visual angle uses math y-up
    double tcp_place_offset_x_ = 0.0;
    double tcp_place_offset_y_ = 0.0;
    double tcp_place_offset_z_ = 0.0;
    double place_release_z_margin_ = 0.004;

    // roll/pitch 固定，yaw 只按任务角度给；不连续累计，不解绕。
    double fixed_roll_rad_ = M_PI;
    double fixed_pitch_rad_ = 0.0;

    // 速度：全部是 xArm native move_line 的 mm/s、mm/s^2。
    double transit_speed_mm_s_ = 60.0;
    double transit_acc_mm_s2_ = 100.0;
    double loaded_transit_speed_mm_s_ = 45.0;
    double loaded_transit_acc_mm_s2_ = 80.0;
    double pick_down_speed_mm_s_ = 25.0;
    double pick_down_acc_mm_s2_ = 80.0;
    double place_down_speed_mm_s_ = 20.0;
    double place_down_acc_mm_s2_ = 60.0;
    double lift_speed_mm_s_ = 45.0;
    double lift_acc_mm_s2_ = 100.0;

    // Optional pre-release shake at placement surface.
    // Executed after place_down while suction is still ON, then returns to center before suction OFF.
    bool enable_place_shake_ = false;
    int place_shake_cycles_ = 1;
    double place_shake_dx_m_ = 0.0055; // +/- X shake amplitude in table frame, meters
    double place_shake_dy_m_ = 0.0060; // +/- Y shake amplitude in table frame, meters
    double place_shake_speed_mm_s_ = 15.0;
    double place_shake_acc_mm_s2_ = 50.0;

    int max_tasks_per_plan_ = 1;
    bool stop_on_motion_failure_ = true;

    int suction_io_num_ = 1;
    double suction_on_wait_ = 0.25;
    double suction_off_wait_ = 0.20;

    bool camera_info_ready_ = false;
    cv::Mat P_;

    geometry_msgs::TransformStamped observation_cam_to_table_;
    geometry_msgs::TransformStamped observation_cam_to_base_;
    geometry_msgs::TransformStamped observation_base_to_table_;

    bool robot_state_ready_ = false;
    std::vector<float> native_pose_{0, 0, 0, 3.14159f, 0, 0};
    // native_pose_ 由 robotStateCallback（spinner 线程）写、verifyReachedXYZ（控制线程）读，加锁保护。
    std::mutex native_pose_mutex_;
    ros::Time native_pose_stamp_;

    static double normalizeAngleRad(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    static double clampDouble(double x, double lo, double hi)
    {
        return std::max(lo, std::min(hi, x));
    }

    static bool xmlToDouble(XmlRpc::XmlRpcValue &v, double &out)
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

    bool readPointList(XmlRpc::XmlRpcValue &value, geometry_msgs::Point &p) const
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
            return false;
        double x = 0.0, y = 0.0, z = 0.0;
        if (!xmlToDouble(value[0], x) || !xmlToDouble(value[1], y) || !xmlToDouble(value[2], z))
            return false;
        p.x = x;
        p.y = y;
        p.z = z;
        return true;
    }

    void loadParams()
    {
        pnh_.param("plan_topic", plan_topic_, plan_topic_);
        pnh_.param("status_topic", status_topic_, status_topic_);
        pnh_.param("io_service", io_service_, io_service_);
        pnh_.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
        pnh_.param("base_frame", base_frame_, base_frame_);
        pnh_.param("camera_frame", camera_frame_, camera_frame_);
        pnh_.param("eef_frame", eef_frame_, eef_frame_);

        std::string calibrated_eef;
        if (nh_.getParam("/tetris/calibration_frames/eef_frame", calibrated_eef) && !calibrated_eef.empty())
            eef_frame_ = calibrated_eef;
        if (eef_frame_ == "link6")
        {
            ROS_WARN("[CTRL] eef_frame is link6, forcing link_tcp for suction TCP TF reference.");
            eef_frame_ = "link_tcp";
        }

        pnh_.param("control_period", control_period_, control_period_);
        pnh_.param("xarm_wait_for_finish", xarm_wait_for_finish_, xarm_wait_for_finish_);
        pnh_.param("verify_native_xyz_after_motion", verify_native_xyz_after_motion_, verify_native_xyz_after_motion_);
        pnh_.param("native_xyz_tolerance_mm", native_xyz_tolerance_mm_, native_xyz_tolerance_mm_);
        pnh_.param("native_verify_timeout_s", native_verify_timeout_s_, native_verify_timeout_s_);
        pnh_.param("native_verify_poll_hz", native_verify_poll_hz_, native_verify_poll_hz_);
        pnh_.param("wrist_soft_limit_rad", wrist_soft_limit_rad_, wrist_soft_limit_rad_);

        pnh_.param("use_true_pick_plane", use_true_pick_plane_, use_true_pick_plane_);
        pnh_.param("use_pick_homography", use_pick_homography_, use_pick_homography_);
        pnh_.param("use_board_map", use_board_map_, use_board_map_);
        pnh_.param("use_cell_max_place_z", use_cell_max_place_z_, use_cell_max_place_z_);
        pnh_.param("require_board_map", require_board_map_, require_board_map_);
        pnh_.param("require_place_z_map", require_place_z_map_, require_place_z_map_);

        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        // 波纹管补偿：优先用标定写入的 /tetris/TCP_PICK_OFFSET_Z，launch 同名私有参数可覆盖。
        nh_.getParam("/tetris/TCP_PICK_OFFSET_Z", tcp_pick_offset_z_);
        pnh_.param("tcp_pick_offset_z", tcp_pick_offset_z_, tcp_pick_offset_z_);

        // 推荐在 launch 里用角度制 pick_yaw_offset_deg；保留 rad 参数兼容旧配置。
        double pick_yaw_offset_deg = 0.0;
        pnh_.param("pick_yaw_offset_deg", pick_yaw_offset_deg, 0.0);
        pick_yaw_offset_rad_ = pick_yaw_offset_deg * M_PI / 180.0;
        pnh_.param("pick_yaw_offset_rad", pick_yaw_offset_rad_, pick_yaw_offset_rad_);
        pnh_.param("yaw_homography_probe_px", yaw_homography_probe_px_, yaw_homography_probe_px_);
        pnh_.param("yaw_homography_v_sign", yaw_homography_v_sign_, yaw_homography_v_sign_);
        yaw_homography_v_sign_ = (yaw_homography_v_sign_ >= 0.0) ? 1.0 : -1.0;

        pnh_.param("tcp_place_offset_x", tcp_place_offset_x_, tcp_place_offset_x_);
        pnh_.param("tcp_place_offset_y", tcp_place_offset_y_, tcp_place_offset_y_);
        // 波纹管补偿：优先用标定写入的 /tetris/TCP_PLACE_OFFSET_Z，launch 同名私有参数可覆盖。
        nh_.getParam("/tetris/TCP_PLACE_OFFSET_Z", tcp_place_offset_z_);
        pnh_.param("tcp_place_offset_z", tcp_place_offset_z_, tcp_place_offset_z_);
        pnh_.param("place_release_z_margin", place_release_z_margin_, place_release_z_margin_);

        pnh_.param("fixed_roll_rad", fixed_roll_rad_, fixed_roll_rad_);
        pnh_.param("fixed_pitch_rad", fixed_pitch_rad_, fixed_pitch_rad_);

        pnh_.param("transit_speed_mm_s", transit_speed_mm_s_, transit_speed_mm_s_);
        pnh_.param("transit_acc_mm_s2", transit_acc_mm_s2_, transit_acc_mm_s2_);
        pnh_.param("loaded_transit_speed_mm_s", loaded_transit_speed_mm_s_, loaded_transit_speed_mm_s_);
        pnh_.param("loaded_transit_acc_mm_s2", loaded_transit_acc_mm_s2_, loaded_transit_acc_mm_s2_);
        pnh_.param("pick_down_speed_mm_s", pick_down_speed_mm_s_, pick_down_speed_mm_s_);
        pnh_.param("pick_down_acc_mm_s2", pick_down_acc_mm_s2_, pick_down_acc_mm_s2_);
        pnh_.param("place_down_speed_mm_s", place_down_speed_mm_s_, place_down_speed_mm_s_);
        pnh_.param("place_down_acc_mm_s2", place_down_acc_mm_s2_, place_down_acc_mm_s2_);
        pnh_.param("lift_speed_mm_s", lift_speed_mm_s_, lift_speed_mm_s_);
        pnh_.param("lift_acc_mm_s2", lift_acc_mm_s2_, lift_acc_mm_s2_);

        pnh_.param("enable_place_shake", enable_place_shake_, enable_place_shake_);
        pnh_.param("place_shake_cycles", place_shake_cycles_, place_shake_cycles_);
        pnh_.param("place_shake_dx_m", place_shake_dx_m_, place_shake_dx_m_);
        pnh_.param("place_shake_dy_m", place_shake_dy_m_, place_shake_dy_m_);
        pnh_.param("place_shake_speed_mm_s", place_shake_speed_mm_s_, place_shake_speed_mm_s_);
        pnh_.param("place_shake_acc_mm_s2", place_shake_acc_mm_s2_, place_shake_acc_mm_s2_);
        place_shake_cycles_ = std::max(0, place_shake_cycles_);
        place_shake_dx_m_ = std::max(0.0, place_shake_dx_m_);
        place_shake_dy_m_ = std::max(0.0, place_shake_dy_m_);

        pnh_.param("max_tasks_per_plan", max_tasks_per_plan_, max_tasks_per_plan_);
        pnh_.param("stop_on_motion_failure", stop_on_motion_failure_, stop_on_motion_failure_);
        pnh_.param("suction_io_num", suction_io_num_, suction_io_num_);
        pnh_.param("suction_on_wait", suction_on_wait_, suction_on_wait_);
        pnh_.param("suction_off_wait", suction_off_wait_, suction_off_wait_);

        nh_.param("/tetris/GRID_SIZE", GRID_SIZE_, GRID_SIZE_);
        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X_, BOARD_ORIGIN_X_);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y_, BOARD_ORIGIN_Y_);
        nh_.param("/tetris/PICK_Z", PICK_Z_, PICK_Z_);
        nh_.param("/tetris/PLACE_Z", PLACE_Z_, PLACE_Z_);
        nh_.param("/tetris/HOVER_Z", HOVER_Z_, HOVER_Z_);

        board_pose_loaded_ = loadBoardPose();
        board_centers_loaded_ = loadBoardCenters14x10();
        place_z_map_loaded_ = loadPlaceZMap14x10();
        pick_plane_loaded_ = loadPickPlaneIfAvailable();
        pick_homography_loaded_ = loadPickHomographyIfAvailable();

        ROS_INFO("[CTRL] calib: board_pose=%s board=%s zmap=%s pick_plane=%s pick_homography=%s",
                 board_pose_loaded_ ? "loaded" : "MISSING",
                 board_centers_loaded_ ? "loaded" : "MISSING",
                 place_z_map_loaded_ ? "loaded" : "MISSING",
                 pick_plane_loaded_ ? "loaded" : "flat_PICK_Z",
                 pick_homography_loaded_ ? "loaded" : "off");
    }

    bool loadBoardPose()
    {
        XmlRpc::XmlRpcValue bp;
        if (!nh_.getParam("/tetris/BOARD_POSE_BASE", bp) ||
            bp.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !bp.hasMember("origin") || !bp.hasMember("rpy"))
        {
            ROS_ERROR("[CTRL] /tetris/BOARD_POSE_BASE not found; run calibrate_board first.");
            return false;
        }
        tf2::Vector3 origin, rpy;
        if (!readVector3List(bp["origin"], origin) || !readVector3List(bp["rpy"], rpy))
        {
            ROS_ERROR("[CTRL] Invalid BOARD_POSE_BASE.");
            return false;
        }
        tf2::Quaternion q;
        q.setRPY(rpy.x(), rpy.y(), rpy.z());
        board_to_base_.setRotation(q);
        board_to_base_.setOrigin(origin);
        base_to_board_ = board_to_base_.inverse();
        ROS_INFO("[CTRL] BOARD_POSE_BASE loaded: origin=(%.4f,%.4f,%.4f) rpy=(%.4f,%.4f,%.4f)",
                 origin.x(), origin.y(), origin.z(), rpy.x(), rpy.y(), rpy.z());
        return true;
    }

    static double unwrapAngle(double current, double target)
    {
        double diff = target - current;
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff < -M_PI)
            diff += 2.0 * M_PI;
        return current + diff;
    }

    bool loadBoardCenters14x10()
    {
        if (!use_board_map_)
            return false;

        XmlRpc::XmlRpcValue centers;
        if (!nh_.getParam("/tetris/BOARD_CENTERS_14x10_BOARD", centers) ||
            centers.getType() != XmlRpc::XmlRpcValue::TypeArray || centers.size() <= 0)
        {
            ROS_WARN("[CTRL] /tetris/BOARD_CENTERS_14x10_BOARD not found.");
            return false;
        }

        board_rows_ = centers.size();
        board_cols_ = centers[0].size();
        board_centers_.assign(board_rows_, std::vector<geometry_msgs::Point>(board_cols_));
        for (int r = 0; r < board_rows_; ++r)
        {
            if (centers[r].getType() != XmlRpc::XmlRpcValue::TypeArray || centers[r].size() != board_cols_)
                return false;
            for (int c = 0; c < board_cols_; ++c)
            {
                if (!readPointList(centers[r][c], board_centers_[r][c]))
                    return false;
            }
        }
        return true;
    }

    bool loadPlaceZMap14x10()
    {
        XmlRpc::XmlRpcValue zmap;
        if (!nh_.getParam("/tetris/PLACE_Z_MAP_14x10", zmap) ||
            zmap.getType() != XmlRpc::XmlRpcValue::TypeArray || zmap.size() <= 0)
        {
            ROS_WARN("[CTRL] /tetris/PLACE_Z_MAP_14x10 not found.");
            return false;
        }

        int rows = zmap.size();
        int cols = zmap[0].size();
        place_z_map_.assign(rows, std::vector<double>(cols, PLACE_Z_));
        for (int r = 0; r < rows; ++r)
        {
            if (zmap[r].getType() != XmlRpc::XmlRpcValue::TypeArray || zmap[r].size() != cols)
                return false;
            for (int c = 0; c < cols; ++c)
            {
                if (!xmlToDouble(zmap[r][c], place_z_map_[r][c]))
                    return false;
            }
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
            ROS_WARN("[CTRL] /tetris/PICK_SURFACE_PLANE_BASE not found; fallback to flat PICK_Z.");
            return false;
        }

        tf2::Vector3 p, n;
        if (!readVector3List(plane["point"], p) || !readVector3List(plane["normal"], n) || n.length() < 1e-9)
        {
            ROS_WARN("[CTRL] Invalid PICK_SURFACE_PLANE_BASE; fallback to flat PICK_Z.");
            return false;
        }

        pick_plane_point_base_ = p;
        pick_plane_normal_base_ = n.normalized();
        ROS_INFO("[CTRL] Loaded true pick plane in %s: point=(%.6f, %.6f, %.6f), normal=(%.6f, %.6f, %.6f)",
                 base_frame_.c_str(), p.x(), p.y(), p.z(),
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
            ROS_WARN("[CTRL] /tetris/PICK_HOMOGRAPHY not found; raw true-plane XY will be used.");
            return false;
        }

        if (hg.hasMember("enabled"))
        {
            try
            {
                bool enabled = static_cast<bool>(hg["enabled"]);
                if (!enabled)
                    return false;
            }
            catch (...)
            {
            }
        }

        if (!hg.hasMember("matrix"))
            return false;

        std::vector<double> H;
        if (!readDoubleArray(hg["matrix"], H) || H.size() < 9)
            return false;

        for (int i = 0; i < 9; ++i)
            pick_homography_H_[i] = H[i];

        double rms = -1.0;
        if (hg.hasMember("rms_m"))
            xmlToDouble(hg["rms_m"], rms);

        ROS_INFO("[CTRL] Loaded PICK_HOMOGRAPHY, rms=%.3f mm", rms >= 0.0 ? rms * 1000.0 : -1.0);
        return true;
    }

    void setNativeModeOnce()
    {
        xarm_msgs::SetInt16 mode_srv;
        mode_srv.request.data = 0;
        if (!set_mode_client_.call(mode_srv))
            ROS_WARN("[CTRL] Failed to call /xarm/set_mode.");

        xarm_msgs::SetInt16 state_srv;
        state_srv.request.data = 0;
        if (!set_state_client_.call(state_srv))
            ROS_WARN("[CTRL] Failed to call /xarm/set_state.");
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        P_ = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 12; ++i)
            P_.at<double>(i / 4, i % 4) = msg->P[i];
        if (!msg->header.frame_id.empty())
            camera_frame_ = msg->header.frame_id;
        camera_info_ready_ = true;
    }

    void robotStateCallback(const xarm_msgs::RobotMsg::ConstPtr &msg)
    {
        if (msg->pose.size() >= 6)
        {
            std::lock_guard<std::mutex> lk(native_pose_mutex_);
            native_pose_.assign(msg->pose.begin(), msg->pose.begin() + 6);
            native_pose_stamp_ = ros::Time::now();
            robot_state_ready_ = true;
        }
    }

    bool getObservationTransforms()
    {
        if (!board_pose_loaded_)
        {
            ROS_ERROR_THROTTLE(2.0, "[CTRL] BOARD_POSE_BASE not loaded; cannot compute observations.");
            return false;
        }
        try
        {
            // 唯一动态 TF：相机随臂动。其余用常量 board 位姿合成（base 化，不再查 table_frame）。
            observation_cam_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            tf2::Transform cam_to_base;
            {
                tf2::Quaternion q;
                tf2::fromMsg(observation_cam_to_base_.transform.rotation, q);
                cam_to_base.setRotation(q);
                const auto &tr = observation_cam_to_base_.transform.translation;
                cam_to_base.setOrigin(tf2::Vector3(tr.x, tr.y, tr.z));
            }
            // board<-cam = (board<-base) * (base<-cam)；board<-base = base_to_board_(常量)。
            tf2::Transform cam_to_board = base_to_board_ * cam_to_base;
            observation_cam_to_table_.transform = tf2::toMsg(cam_to_board);
            observation_base_to_table_.transform = tf2::toMsg(base_to_board_);
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] TF lookup failed: %s", ex.what());
            return false;
        }
    }

    bool pixelToNormalizedRay(const PixelPoint &px, cv::Vec3d &ray) const
    {
        if (!camera_info_ready_ || P_.empty())
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
        if (std::abs(ray_table.z()) < 1e-9)
            return false;
        double t = (PICK_Z_ - cam_pos.z()) / ray_table.z();
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
            ROS_ERROR("[CTRL] base->table transform for pick point failed: %s", ex.what());
            return false;
        }
        out_table = p_table.point;
        return true;
    }

    bool applyPickHomographyXY(double u, double v, geometry_msgs::Point &p) const
    {
        if (!use_pick_homography_ || !pick_homography_loaded_)
            return false;

        const double *H = pick_homography_H_;
        const double W = H[6] * u + H[7] * v + H[8];
        if (std::abs(W) < 1e-9)
            return false;

        // H maps pixel -> table XY. Z still comes from true pick plane / flat plane.
        p.x = (H[0] * u + H[1] * v + H[2]) / W;
        p.y = (H[3] * u + H[4] * v + H[5]) / W;
        return true;
    }

    bool applyPickHomography(const PixelPoint &px, geometry_msgs::Point &p) const
    {
        return applyPickHomographyXY(static_cast<double>(px.u), static_cast<double>(px.v), p);
    }

    double yawFromHomography(double u, double v, double angle_img_rad) const
    {
        if (!use_pick_homography_ || !pick_homography_loaded_)
            return normalizeAngleRad(angle_img_rad);

        const double L = std::max(5.0, yaw_homography_probe_px_);
        const double u1 = u + L * std::cos(angle_img_rad);
        const double v1 = v + yaw_homography_v_sign_ * L * std::sin(angle_img_rad);

        geometry_msgs::Point p0, p1;
        if (!applyPickHomographyXY(u, v, p0) || !applyPickHomographyXY(u1, v1, p1))
            return normalizeAngleRad(angle_img_rad);

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        if (std::hypot(dx, dy) < 1e-9)
            return normalizeAngleRad(angle_img_rad);

        return normalizeAngleRad(std::atan2(dy, dx));
    }

    bool pixelToPickPointTable(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        bool ok = false;
        // 1. 先用真实的 3D 射线求交，拿到带有高度信息的物理坐标 (重点是获取正确的 out.z)
        if (use_true_pick_plane_ && pick_plane_loaded_)
            ok = pixelToTruePickPlaneTablePoint(px, out);
        else
            ok = pixelToFlatPickZTablePoint(px, out);

        if (!ok)
            return false;

        // 如果不使用单应性矩阵，直接返回 3D 射线的结果
        if (!use_pick_homography_ || !pick_homography_loaded_)
            return true;

        // ==========================================================
        // 2. 核心修复：消除 2.5D 视觉的“高度视差 (Parallax Error)”
        // ==========================================================
        double u_flat = px.u;
        double v_flat = px.v;

        if (camera_info_ready_ && !P_.empty())
        {
            // 获取镜头光心
            double cx = P_.at<double>(0, 2);
            double cy = P_.at<double>(1, 2);

            // 获取镜头到桌面的绝对高度，以及积木吸取面的绝对高度
            double cam_z = std::abs(observation_cam_to_table_.transform.translation.z);
            double block_z = std::abs(out.z);

            // 只有当相机在积木上方时才进行压缩补偿
            if (cam_z > block_z + 0.05)
            {
                // 计算视差收缩比例： (相机高度 - 积木高度) / 相机高度
                double ratio = (cam_z - block_z) / cam_z;

                // 将像素强行向光心 (cx, cy) 拉回，模拟把积木顶面“拍扁”到标定纸的平面上！
                u_flat = cx + (px.u - cx) * ratio;
                v_flat = cy + (px.v - cy) * ratio;
            }
        }

        // ==========================================================
        // 3. 把“拍扁去畸变”后的像素喂给 2D 单应性矩阵
        // ==========================================================
        geometry_msgs::Point hom_p;
        if (applyPickHomographyXY(u_flat, v_flat, hom_p))
        {
            out.x = hom_p.x;
            out.y = hom_p.y;
            // 【注意】：这里绝不覆盖 out.z，完美保留 3D 射线算出的下压高度！
            return true;
        }
        return false;
    }

    geometry_msgs::Point bilinearBoardCenter(double row, double col) const
    {
        geometry_msgs::Point fallback;
        fallback.x = BOARD_ORIGIN_X_ - row * GRID_SIZE_;
        fallback.y = BOARD_ORIGIN_Y_ - col * GRID_SIZE_;
        fallback.z = PLACE_Z_;
        if (!board_centers_loaded_ || board_centers_.empty())
            return fallback;

        row = clampDouble(row, 0.0, static_cast<double>(board_rows_ - 1));
        col = clampDouble(col, 0.0, static_cast<double>(board_cols_ - 1));
        int r0 = static_cast<int>(std::floor(row));
        int c0 = static_cast<int>(std::floor(col));
        int r1 = std::min(r0 + 1, board_rows_ - 1);
        int c1 = std::min(c0 + 1, board_cols_ - 1);
        double tr = row - r0;
        double tc = col - c0;
        const auto &p00 = board_centers_[r0][c0];
        const auto &p10 = board_centers_[r1][c0];
        const auto &p01 = board_centers_[r0][c1];
        const auto &p11 = board_centers_[r1][c1];
        geometry_msgs::Point p;
        p.x = (1 - tr) * (1 - tc) * p00.x + tr * (1 - tc) * p10.x + (1 - tr) * tc * p01.x + tr * tc * p11.x;
        p.y = (1 - tr) * (1 - tc) * p00.y + tr * (1 - tc) * p10.y + (1 - tr) * tc * p01.y + tr * tc * p11.y;
        p.z = (1 - tr) * (1 - tc) * p00.z + tr * (1 - tc) * p10.z + (1 - tr) * tc * p01.z + tr * tc * p11.z;
        return p;
    }

    double bilinearPlaceZ(double row, double col) const
    {
        if (!place_z_map_loaded_ || place_z_map_.empty())
            return PLACE_Z_;
        int rows = place_z_map_.size();
        int cols = place_z_map_[0].size();
        row = clampDouble(row, 0.0, static_cast<double>(rows - 1));
        col = clampDouble(col, 0.0, static_cast<double>(cols - 1));
        int r0 = static_cast<int>(std::floor(row));
        int c0 = static_cast<int>(std::floor(col));
        int r1 = std::min(r0 + 1, rows - 1);
        int c1 = std::min(c0 + 1, cols - 1);
        double tr = row - r0;
        double tc = col - c0;
        return (1 - tr) * (1 - tc) * place_z_map_[r0][c0] + tr * (1 - tc) * place_z_map_[r1][c0] +
               (1 - tr) * tc * place_z_map_[r0][c1] + tr * tc * place_z_map_[r1][c1];
    }

    double placeZFromTargetCells(const TaskGoal &goal) const
    {
        double z = bilinearPlaceZ(goal.place_grid_center.row, goal.place_grid_center.col);
        if (!use_cell_max_place_z_ || !goal.has_target_cells || goal.target_cells.empty() || !place_z_map_loaded_)
            return z;

        double z_max = -1e9;
        int rows = place_z_map_.size();
        int cols = place_z_map_[0].size();
        for (const auto &cell : goal.target_cells)
        {
            int r = std::max(0, std::min(rows - 1, cell.row));
            int c = std::max(0, std::min(cols - 1, cell.col));
            z_max = std::max(z_max, place_z_map_[r][c]);
        }
        return z_max;
    }

    geometry_msgs::Pose makePoseTable(double x, double y, double z, double yaw_rad) const
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(fixed_roll_rad_, fixed_pitch_rad_, normalizeAngleRad(yaw_rad));
        pose.orientation = tf2::toMsg(q);
        return pose;
    }

    geometry_msgs::Pose hoverFrom(const geometry_msgs::Pose &target) const
    {
        geometry_msgs::Pose h = target;
        h.position.z = HOVER_Z_;
        if (h.position.z < target.position.z + 0.010)
            h.position.z = target.position.z + 0.010;
        return h;
    }

    bool computePickToGeomOffset(const PixelPoint &pick_px, const PixelPoint &geom_px,
                                 double &pick_minus_geom_x, double &pick_minus_geom_y) const
    {
        geometry_msgs::Point pick_p, geom_p;
        if (!pixelToPickPointTable(pick_px, pick_p) || !pixelToPickPointTable(geom_px, geom_p))
            return false;
        pick_minus_geom_x = pick_p.x - geom_p.x;
        pick_minus_geom_y = pick_p.y - geom_p.y;
        return true;
    }

    // 【修改】：添加 double &sim_yaw 引用参数
    bool buildTask(TaskGoal &goal, double &sim_yaw)
    {
        geometry_msgs::Point pick_p;
        if (!pixelToPickPointTable(goal.pick_pixel, pick_p))
            return false;

        // 计算基础角度
        double pick_yaw_base = yawFromHomography(goal.pick_pixel.u,
                                                 goal.pick_pixel.v,
                                                 goal.pick_angle_deg * M_PI / 180.0) +
                               pick_yaw_offset_rad_;
        pick_yaw_base = normalizeAngleRad(pick_yaw_base);
        double place_yaw_base = normalizeAngleRad(-goal.way * M_PI / 2.0);

        // 预测器坐标系对齐
        std::vector<float> native_test;

        // 提取 0 度方案的真实物理 Yaw
        poseTableToNativeBase(makePoseTable(0, 0, 0, pick_yaw_base), native_test);
        double pick_base_A = native_test[5];
        poseTableToNativeBase(makePoseTable(0, 0, 0, place_yaw_base), native_test);
        double place_base_A = native_test[5];

        // 提取 180 度翻转方案的真实物理 Yaw
        poseTableToNativeBase(makePoseTable(0, 0, 0, pick_yaw_base + M_PI), native_test);
        double pick_base_B = native_test[5];
        poseTableToNativeBase(makePoseTable(0, 0, 0, place_yaw_base + M_PI), native_test);
        double place_base_B = native_test[5];

        // 用转换后的物理角度预测两套方案的"腕部转角"。固件按就近解执行，给定姿态的旋转量已最小；
        // 唯一的自由度是 A/B 这 180° 翻转（吸盘对 180° 对称，pick/place 同步翻转后落点不变）。
        // 目标：腕部转角最小——xArm 各轴同时到达，多转的 yaw 会成为整段运动的速度瓶颈。
        // 限位仅作约束：预测 |腕角| 超软限位的方案才被排除（避免长期累积撞 ±2π 硬限）。
        // 方案 A：不翻转
        double pick_unwrapped_A = unwrapAngle(sim_yaw, pick_base_A);
        double place_unwrapped_A = unwrapAngle(pick_unwrapped_A, place_base_A);
        double travel_A = std::abs(pick_unwrapped_A - sim_yaw) + std::abs(place_unwrapped_A - pick_unwrapped_A);
        double max_abs_A = std::max(std::abs(pick_unwrapped_A), std::abs(place_unwrapped_A));

        // 方案 B：翻转 180 度
        double pick_unwrapped_B = unwrapAngle(sim_yaw, pick_base_B);
        double place_unwrapped_B = unwrapAngle(pick_unwrapped_B, place_base_B);
        double travel_B = std::abs(pick_unwrapped_B - sim_yaw) + std::abs(place_unwrapped_B - pick_unwrapped_B);
        double max_abs_B = std::max(std::abs(pick_unwrapped_B), std::abs(place_unwrapped_B));

        const bool feasible_A = max_abs_A <= wrist_soft_limit_rad_;
        const bool feasible_B = max_abs_B <= wrist_soft_limit_rad_;

        bool choose_B;
        if (feasible_A && feasible_B)
            choose_B = travel_B < travel_A; // 都不越限：选腕部转角更小的（保速度）
        else if (feasible_A)
            choose_B = false; // 仅 A 可行
        else if (feasible_B)
            choose_B = true; // 仅 B 可行
        else
            choose_B = max_abs_B < max_abs_A; // 都越限（极端）：退回 |腕角| 更小的，尽量别撞限位

        double final_pick_yaw, final_place_yaw;
        if (choose_B)
        {
            final_pick_yaw = normalizeAngleRad(pick_yaw_base + M_PI);
            final_place_yaw = normalizeAngleRad(place_yaw_base + M_PI);
            sim_yaw = place_unwrapped_B; // 更新预测器到 B 的物理落点
        }
        else
        {
            final_pick_yaw = pick_yaw_base;
            final_place_yaw = place_yaw_base;
            sim_yaw = place_unwrapped_A; // 更新预测器到 A 的物理落点
        }
        ROS_INFO("[CTRL][YAW] scheme=%s travel(A=%.0f,B=%.0f) max|wrist|(A=%.0f,B=%.0f) soft=%.0f deg",
                 choose_B ? "B(flip)" : "A", travel_A * 180.0 / M_PI, travel_B * 180.0 / M_PI,
                 max_abs_A * 180.0 / M_PI, max_abs_B * 180.0 / M_PI, wrist_soft_limit_rad_ * 180.0 / M_PI);

        // --- 使用 final_pick_yaw 计算 TCP 偏移 ---
        double pick_x = pick_p.x + std::cos(final_pick_yaw) * tcp_pick_offset_x_ - std::sin(final_pick_yaw) * tcp_pick_offset_y_;
        double pick_y = pick_p.y + std::sin(final_pick_yaw) * tcp_pick_offset_x_ + std::cos(final_pick_yaw) * tcp_pick_offset_y_;
        double pick_z = pick_p.z + tcp_pick_offset_z_;
        goal.pick_pose_table = makePoseTable(pick_x, pick_y, pick_z, final_pick_yaw);

        // 几何中心偏移逻辑
        if (goal.has_geom_pixel)
        {
            double dx = 0.0, dy = 0.0;
            if (computePickToGeomOffset(goal.pick_pixel, goal.geom_pixel, dx, dy))
            {
                goal.pick_minus_geom_x = dx;
                goal.pick_minus_geom_y = dy;
                goal.has_pick_to_geom_offset = true;
            }
        }

        double place_x = bilinearBoardCenter(goal.place_grid_center.row, goal.place_grid_center.col).x;
        double place_y = bilinearBoardCenter(goal.place_grid_center.row, goal.place_grid_center.col).y;

        if (goal.has_pick_to_geom_offset)
        {
            // delta 不受 180 度翻转影响，数学上完美自洽
            double delta = final_place_yaw - final_pick_yaw;
            double dx_rot = goal.pick_minus_geom_x * std::cos(delta) - goal.pick_minus_geom_y * std::sin(delta);
            double dy_rot = goal.pick_minus_geom_x * std::sin(delta) + goal.pick_minus_geom_y * std::cos(delta);
            place_x += dx_rot;
            place_y += dy_rot;
        }
        place_x += std::cos(final_place_yaw) * tcp_place_offset_x_ - std::sin(final_place_yaw) * tcp_place_offset_y_;
        place_y += std::sin(final_place_yaw) * tcp_place_offset_x_ + std::cos(final_place_yaw) * tcp_place_offset_y_;
        double place_z = placeZFromTargetCells(goal) + place_release_z_margin_ + tcp_place_offset_z_;

        goal.place_pose_table = makePoseTable(place_x, place_y, place_z, final_place_yaw);

        ROS_INFO("[CTRL][TASK] shape=%d pick=(%.3f,%.3f) place=(%.3f,%.3f) yaw_pick=%.1f yaw_place=%.1f",
                 goal.shape_type, pick_x, pick_y, place_x, place_y,
                 final_pick_yaw * 180.0 / M_PI, final_place_yaw * 180.0 / M_PI);
        return true;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty() || !camera_info_ready_)
            return;

        if (require_board_map_ && !board_centers_loaded_)
        {
            ROS_ERROR("[CTRL] Refuse plan: required BOARD_CENTERS_14x10_BOARD missing.");
            return;
        }
        if (require_place_z_map_ && !place_z_map_loaded_)
        {
            ROS_ERROR("[CTRL] Refuse plan: required PLACE_Z_MAP_14x10 missing.");
            return;
        }
        if (!getObservationTransforms())
            return;

        while (!tasks_.empty())
            tasks_.pop();

        int total = msg->data[0];
        if (total <= 0)
            return;
        int stride = (msg->data.size() - 1) / total;
        if (stride < 7)
        {
            ROS_ERROR("[CTRL] Invalid plan stride=%d", stride);
            return;
        }

        int exec_total = total;
        if (max_tasks_per_plan_ > 0 && exec_total > max_tasks_per_plan_)
        {
            ROS_WARN("[CTRL][SAFETY] Received %d tasks; executing first %d task(s).", total, max_tasks_per_plan_);
            exec_total = max_tasks_per_plan_;
        }

        // 【新增】：获取机械臂当前的真实偏航角，作为模拟器的起点
        double sim_yaw = 0.0;
        if (robot_state_ready_)
        {
            sim_yaw = static_cast<double>(native_pose_[5]);
        }

        for (int i = 0; i < exec_total; ++i)
        {
            int base = 1 + i * stride;
            TaskGoal goal;
            goal.shape_type = msg->data[base + 0];
            goal.way = msg->data[base + 1];
            goal.place_grid_center.row = msg->data[base + 2] / 4.0;
            goal.place_grid_center.col = msg->data[base + 3] / 4.0;
            goal.pick_pixel.u = msg->data[base + 4];
            goal.pick_pixel.v = msg->data[base + 5];
            goal.pick_angle_deg = msg->data[base + 6];

            if (stride >= 9)
            {
                goal.geom_pixel.u = msg->data[base + 7];
                goal.geom_pixel.v = msg->data[base + 8];
                goal.has_geom_pixel = true;
            }
            else
            {
                goal.geom_pixel = goal.pick_pixel;
                goal.has_geom_pixel = false;
            }

            int cell_start = -1;
            if (stride >= 17)
                cell_start = base + 9;
            else if (stride >= 15)
                cell_start = base + 7;

            if (cell_start > 0 && cell_start + 7 < base + stride)
            {
                goal.has_target_cells = true;
                for (int k = 0; k < 4; ++k)
                {
                    GridCell cell;
                    cell.row = msg->data[cell_start + 2 * k];
                    cell.col = msg->data[cell_start + 2 * k + 1];
                    goal.target_cells.push_back(cell);
                }
            }

            // 【修改】：把 sim_yaw 传递给 buildTask
            if (buildTask(goal, sim_yaw))
                tasks_.push(goal);
            else
                ROS_WARN("[CTRL] Skip task %d: failed to build pick/place pose.", i + 1);
        }

        if (!tasks_.empty())
        {
            publishBusy(true);
            state_ = State::TAKE_NEXT_TASK;
        }
    }

    bool poseTableToNativeBase(const geometry_msgs::Pose &target_table, std::vector<float> &pose_mm_rad) const
    {
        // board pose -> base native：用常量 board_to_base_，不走 TF。
        if (!board_pose_loaded_)
        {
            ROS_ERROR("[CTRL] BOARD_POSE_BASE not loaded; cannot convert pose.");
            return false;
        }
        tf2::Transform t_board;
        tf2::Quaternion q;
        tf2::fromMsg(target_table.orientation, q);
        t_board.setRotation(q);
        t_board.setOrigin(tf2::Vector3(target_table.position.x, target_table.position.y, target_table.position.z));

        tf2::Transform t_base = board_to_base_ * t_board;
        tf2::Vector3 pos = t_base.getOrigin();
        double r = 0.0, p = 0.0, y = 0.0;
        tf2::Matrix3x3(t_base.getRotation()).getRPY(r, p, y);

        pose_mm_rad = {
            static_cast<float>(pos.x() * 1000.0),
            static_cast<float>(pos.y() * 1000.0),
            static_cast<float>(pos.z() * 1000.0),
            static_cast<float>(r),
            static_cast<float>(p),
            static_cast<float>(normalizeAngleRad(y))};
        return true;
    }

    bool verifyReachedXYZ(const std::vector<float> &target, const std::string &label)
    {
        if (!verify_native_xyz_after_motion_)
            return true;
        if (!robot_state_ready_)
        {
            ROS_WARN("[CTRL][VERIFY] No /xarm/xarm_states yet; skip verification for %s.", label.c_str());
            return true;
        }

        // 不再用固定 sleep 盲等：上报有延迟/抖动，定值要么太短读到运动前的陈旧位姿、
        // 要么白白浪费时间。改为轮询，只采信进入本函数后才到达的"新鲜"上报，
        // 等其进入容差即判定到位；超时仍未到位才判失败。
        const ros::Time t_enter = ros::Time::now();
        const ros::Time deadline = t_enter + ros::Duration(native_verify_timeout_s_);
        ros::Rate rate(native_verify_poll_hz_ > 0.0 ? native_verify_poll_hz_ : 50.0);

        bool got_fresh = false;
        double err = std::numeric_limits<double>::infinity();
        float cx = 0, cy = 0, cz = 0;
        while (ros::ok())
        {
            std::vector<float> pose_now;
            ros::Time stamp;
            {
                std::lock_guard<std::mutex> lk(native_pose_mutex_);
                pose_now = native_pose_;
                stamp = native_pose_stamp_;
            }
            if (stamp > t_enter) // 只用运动结束后到达的上报，避免拿运动前位姿误判
            {
                got_fresh = true;
                cx = pose_now[0];
                cy = pose_now[1];
                cz = pose_now[2];
                double dx = static_cast<double>(cx) - target[0];
                double dy = static_cast<double>(cy) - target[1];
                double dz = static_cast<double>(cz) - target[2];
                err = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (err <= native_xyz_tolerance_mm_)
                {
                    ROS_INFO("[CTRL][VERIFY] %s reached target: err=%.2f mm", label.c_str(), err);
                    return true;
                }
            }
            if (ros::Time::now() >= deadline)
                break;
            rate.sleep();
        }

        if (!got_fresh)
        {
            ROS_ERROR("[CTRL][VERIFY] %s: no fresh /xarm state within %.2fs; cannot verify.",
                      label.c_str(), native_verify_timeout_s_);
            return false;
        }
        ROS_ERROR("[CTRL][VERIFY] %s did not reach target within %.2fs: err=%.2f mm > %.2f mm. cur=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f)",
                  label.c_str(), native_verify_timeout_s_, err, native_xyz_tolerance_mm_,
                  cx, cy, cz, target[0], target[1], target[2]);
        return false;
    }

    bool moveLineNative(const geometry_msgs::Pose &target_table, double speed_mm_s, double acc_mm_s2, const std::string &label)
    {
        std::vector<float> target;
        if (!poseTableToNativeBase(target_table, target))
            return false;

        if (robot_state_ready_)
        {
            target[3] = unwrapAngle(native_pose_[3], target[3]); // Roll
            target[4] = unwrapAngle(native_pose_[4], target[4]); // Pitch
            target[5] = unwrapAngle(native_pose_[5], target[5]); // Yaw
        }

        xarm_msgs::Move srv;
        srv.request.pose = target;
        srv.request.mvvelo = static_cast<float>(speed_mm_s);
        srv.request.mvacc = static_cast<float>(acc_mm_s2);
        srv.request.mvtime = 0;
        srv.request.mvradii = 0;

        ROS_INFO("[CTRL][MOVE_LINE] %s target_base=(%.1f, %.1f, %.1f, %.3f, %.3f, %.3f), v=%.1f, a=%.1f",
                 label.c_str(), target[0], target[1], target[2], target[3], target[4], target[5],
                 speed_mm_s, acc_mm_s2);

        if (!move_line_client_.call(srv))
        {
            ROS_ERROR("[CTRL] Failed to call /xarm/move_line for %s", label.c_str());
            return false;
        }
        if (srv.response.ret != 0)
        {
            ROS_ERROR("[CTRL] /xarm/move_line failed for %s: ret=%d msg=%s", label.c_str(), srv.response.ret, srv.response.message.c_str());
            return false;
        }
        return verifyReachedXYZ(target, label);
    }

    bool executePlaceShake(const geometry_msgs::Pose &center_pose)
    {
        if (!enable_place_shake_ || place_shake_cycles_ <= 0)
            return true;
        if (place_shake_dx_m_ <= 0.0 && place_shake_dy_m_ <= 0.0)
            return true;

        ROS_INFO("[CTRL][PLACE_SHAKE] pre-release shake: cycles=%d dx=%.1fmm dy=%.1fmm",
                 place_shake_cycles_, place_shake_dx_m_ * 1000.0, place_shake_dy_m_ * 1000.0);

        for (int cycle = 0; cycle < place_shake_cycles_; ++cycle)
        {
            // 中心点
            geometry_msgs::Pose p = center_pose;

            // 第一步：向右下对角线猛挤
            geometry_msgs::Pose p_out = center_pose;
            p_out.position.x += place_shake_dx_m_;
            p_out.position.y += place_shake_dy_m_;
            if (!moveLineNative(p_out, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_diag_out"))
                return false;

            // 第二步：向左上对角线猛挤
            geometry_msgs::Pose p_in = center_pose;
            p_in.position.x -= place_shake_dx_m_;
            p_in.position.y -= place_shake_dy_m_;
            if (!moveLineNative(p_in, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_diag_in"))
                return false;

            // 第三步：回正
            if (!moveLineNative(p, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_center"))
                return false;
        }

        return true;
    }

    bool setSuction(bool on)
    {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = suction_io_num_;
        srv.request.value = on ? 1 : 0;
        if (!io_client_.call(srv))
        {
            ROS_ERROR("[CTRL] Failed to set suction %s", on ? "ON" : "OFF");
            return false;
        }
        ros::Duration(on ? suction_on_wait_ : suction_off_wait_).sleep();
        return true;
    }

    void publishBusy(bool busy)
    {
        std_msgs::Bool msg;
        msg.data = busy;
        status_pub_.publish(msg);
    }

    void failOrFinish(const std::string &reason)
    {
        ROS_ERROR("[CTRL] %s", reason.c_str());
        if (stop_on_motion_failure_)
        {
            while (!tasks_.empty())
                tasks_.pop();
            publishBusy(false);
            state_ = State::IDLE;
        }
        else
        {
            state_ = State::TAKE_NEXT_TASK;
        }
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
        {
            geometry_msgs::Pose pick_hover = hoverFrom(current_task_.pick_pose_table);
            if (!moveLineNative(pick_hover, transit_speed_mm_s_, transit_acc_mm_s2_, "pick_hover"))
            {
                failOrFinish("move_line to pick hover failed.");
                return;
            }
            state_ = State::EXECUTE_PICK;
            return;
        }

        case State::EXECUTE_PICK:
        {
            geometry_msgs::Pose pick_hover = hoverFrom(current_task_.pick_pose_table);
            if (!moveLineNative(current_task_.pick_pose_table, pick_down_speed_mm_s_, pick_down_acc_mm_s2_, "pick_down"))
            {
                failOrFinish("move_line pick down failed; suction will NOT turn on.");
                return;
            }
            if (!setSuction(true))
            {
                failOrFinish("suction ON failed.");
                return;
            }
            if (!moveLineNative(pick_hover, lift_speed_mm_s_, lift_acc_mm_s2_, "pick_lift"))
            {
                failOrFinish("move_line pick lift failed.");
                return;
            }
            state_ = State::MOVE_TO_PLACE_HOVER;
            return;
        }

        case State::MOVE_TO_PLACE_HOVER:
        {
            geometry_msgs::Pose place_hover = hoverFrom(current_task_.place_pose_table);
            if (!moveLineNative(place_hover, loaded_transit_speed_mm_s_, loaded_transit_acc_mm_s2_, "loaded_place_hover"))
            {
                failOrFinish("move_line loaded transit to place hover failed.");
                return;
            }
            state_ = State::EXECUTE_PLACE;
            return;
        }

        case State::EXECUTE_PLACE:
        {
            geometry_msgs::Pose place_hover = hoverFrom(current_task_.place_pose_table);
            if (!moveLineNative(current_task_.place_pose_table, place_down_speed_mm_s_, place_down_acc_mm_s2_, "place_down"))
            {
                failOrFinish("move_line place down failed; suction remains ON.");
                return;
            }
            if (!executePlaceShake(current_task_.place_pose_table))
            {
                failOrFinish("place shake failed; suction remains ON.");
                return;
            }
            if (!setSuction(false))
            {
                failOrFinish("suction OFF failed.");
                return;
            }
            if (!moveLineNative(place_hover, lift_speed_mm_s_, lift_acc_mm_s2_, "place_lift"))
            {
                failOrFinish("move_line place lift failed.");
                return;
            }
            state_ = State::TAKE_NEXT_TASK;
            return;
        }

        case State::FINISH:
            ROS_INFO("[CTRL] All tasks finished using SIMPLE move_line controller.");
            publishBusy(false);
            state_ = State::IDLE;
            return;
        }
    }
};

XarmTetrisController *g_controller = nullptr;

void sigintHandler(int)
{
    if (g_controller)
        g_controller->emergencyStop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_controller_node", ros::init_options::NoSigintHandler);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    XarmTetrisController controller;
    g_controller = &controller;
    signal(SIGINT, sigintHandler);

    ros::waitForShutdown();
    return 0;
}