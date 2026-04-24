#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <xarm_msgs/SetDigitalIO.h>
#include <tly/GetPrecisePose.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <XmlRpcValue.h>

#include <cmath>
#include <cstdio>
#include <queue>
#include <string>
#include <vector>
#include <algorithm>
#include <array>

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
    GridCenter place_grid_center;

    // Backward compatible: old plan has no explicit cells.
    // New extended plan may include 4 target cells for better place-z calculation.
    bool has_target_cells = false;
    std::vector<GridCell> target_cells;

    geometry_msgs::Pose pick_pose;
    geometry_msgs::Pose place_pose;
};

class XarmTetrisController
{
public:
    XarmTetrisController()
        : pnh_("~"),
          tf_listener_(tf_buffer_),
          move_group_("xarm6")
    {
        loadParams();

        move_group_.setPoseReferenceFrame(table_frame_);
        move_group_.setMaxVelocityScalingFactor(velocity_scale_);
        move_group_.setMaxAccelerationScalingFactor(acceleration_scale_);
        move_group_.setPlanningTime(planning_time_);
        move_group_.setNumPlanningAttempts(planning_attempts_);

        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &XarmTetrisController::cameraInfoCallback, this);

        ROS_INFO("[CTRL] Waiting for CameraInfo on %s ...", camera_info_topic_.c_str());
        sensor_msgs::CameraInfoConstPtr cam_msg =
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));

        if (cam_msg)
        {
            cameraInfoCallback(cam_msg);
            ROS_INFO("[CTRL] CameraInfo received before subscribing plan.");
        }
        else
        {
            ROS_WARN("[CTRL] CameraInfo not received within 5 seconds. Will still subscribe and wait later.");
        }

        plan_sub_ = nh_.subscribe(plan_topic_, 1, &XarmTetrisController::planCallback, this);

        io_client_ = nh_.serviceClient<xarm_msgs::SetDigitalIO>(io_service_);
        precise_client_ = nh_.serviceClient<tly::GetPrecisePose>(precise_service_);

        status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
        publishBusy(false);

        control_timer_ = nh_.createTimer(ros::Duration(control_period_), &XarmTetrisController::controlLoop, this);

        ROS_INFO("[CTRL] Refactored xArm controller ready.");
        ROS_INFO("[CTRL] plan=%s camera_info=%s table_frame=%s camera_frame=%s use_precise_service=%s",
                 plan_topic_.c_str(), camera_info_topic_.c_str(), table_frame_.c_str(),
                 camera_frame_.empty() ? "(from CameraInfo/Image TF)" : camera_frame_.c_str(),
                 use_precise_service_ ? "true" : "false");
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber plan_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher status_pub_;
    ros::ServiceClient io_client_;
    ros::ServiceClient precise_client_;
    ros::Timer control_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    moveit::planning_interface::MoveGroupInterface move_group_;

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
    std::string camera_frame_ = "camera_color_optical_frame";

    double velocity_scale_ = 0.05;
    double acceleration_scale_ = 0.05;
    double planning_time_ = 5.0;
    int planning_attempts_ = 5;
    double control_period_ = 0.05;

    double GRID_SIZE_ = 0.017;
    double BOARD_ORIGIN_X_ = 0.0;
    double BOARD_ORIGIN_Y_ = 0.0;
    double PICK_Z_ = 0.0;
    double PLACE_Z_ = 0.0;
    double HOVER_Z_ = 0.10;

    // True pick plane in base_frame. When available, pick pixels are projected
    // onto this arbitrary 3D plane instead of assuming table_frame Z=PICK_Z.
    bool use_true_pick_plane_ = true;
    bool pick_plane_loaded_ = false;
    tf2::Vector3 pick_plane_point_base_{0.0, 0.0, 0.0};
    tf2::Vector3 pick_plane_normal_base_{0.0, 0.0, 1.0};

    // If extended plan includes 4 target cells, use their PLACE_Z_MAP values.
    bool use_cell_max_place_z_ = true;
    double place_z_extra_margin_ = 0.0;

    // Pixel-position dependent affine correction for pick XY in table_frame.
    bool use_pick_affine_correction_ = true;
    bool pick_affine_loaded_ = false;
    double pick_affine_cx_ = 0.0;
    double pick_affine_cy_ = 0.0;
    double pick_affine_x_[3] = {0.0, 0.0, 0.0};
    double pick_affine_y_[3] = {0.0, 0.0, 0.0};

    double tcp_pick_offset_x_ = 0.0;
    double tcp_pick_offset_y_ = 0.0;
    double tcp_pick_offset_z_ = 0.0;
    double tcp_place_offset_x_ = 0.0;
    double tcp_place_offset_y_ = 0.0;

    // Per-shape/per-target-way pick yaw calibration.
    // This fixes cases where the vision angle for an asymmetric piece is correct in
    // position but 180 deg ambiguous for only some planned orientations.
    // corr[shape][way] is added to the visual pick angle before building pick_pose.
    std::array<std::array<int, 4>, 7> pick_yaw_corr_by_shape_way_{};

    bool use_precise_service_ = false;
    bool assume_image_rectified_ = false;
    bool use_board_map_ = true;
    bool stop_on_motion_failure_ = true;

    int suction_io_num_ = 1;
    double suction_on_wait_ = 0.35;
    double suction_off_wait_ = 0.35;

    bool camera_info_ready_ = false;
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat P_;

    bool board_centers_loaded_ = false;
    bool place_z_map_loaded_ = false;
    std::vector<std::vector<geometry_msgs::Point>> board_centers_; // [14][10]
    std::vector<std::vector<double>> place_z_map_;                 // [14][10]

    geometry_msgs::TransformStamped observation_cam_to_table_;
    geometry_msgs::TransformStamped observation_cam_to_base_;
    geometry_msgs::TransformStamped observation_base_to_table_;
    bool observation_tf_ready_ = false;

    void loadParams()
    {
        pnh_.param("plan_topic", plan_topic_, plan_topic_);
        pnh_.param("status_topic", status_topic_, status_topic_);
        pnh_.param("io_service", io_service_, io_service_);
        pnh_.param("precise_service", precise_service_, precise_service_);
        pnh_.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
        pnh_.param("table_frame", table_frame_, table_frame_);
        pnh_.param("base_frame", base_frame_, base_frame_);
        pnh_.param("camera_frame", camera_frame_, camera_frame_);
        pnh_.param("velocity_scale", velocity_scale_, velocity_scale_);
        pnh_.param("acceleration_scale", acceleration_scale_, acceleration_scale_);
        pnh_.param("planning_time", planning_time_, planning_time_);
        pnh_.param("planning_attempts", planning_attempts_, planning_attempts_);
        pnh_.param("control_period", control_period_, control_period_);
        pnh_.param("use_precise_service", use_precise_service_, use_precise_service_);
        pnh_.param("use_true_pick_plane", use_true_pick_plane_, use_true_pick_plane_);
        pnh_.param("use_pick_affine_correction", use_pick_affine_correction_, use_pick_affine_correction_);
        pnh_.param("use_cell_max_place_z", use_cell_max_place_z_, use_cell_max_place_z_);
        pnh_.param("place_z_extra_margin", place_z_extra_margin_, place_z_extra_margin_);
        pnh_.param("assume_image_rectified", assume_image_rectified_, assume_image_rectified_);
        pnh_.param("use_board_map", use_board_map_, use_board_map_);
        pnh_.param("stop_on_motion_failure", stop_on_motion_failure_, stop_on_motion_failure_);
        pnh_.param("suction_io_num", suction_io_num_, suction_io_num_);
        pnh_.param("suction_on_wait", suction_on_wait_, suction_on_wait_);
        pnh_.param("suction_off_wait", suction_off_wait_, suction_off_wait_);
        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        pnh_.param("tcp_pick_offset_z", tcp_pick_offset_z_, tcp_pick_offset_z_);
        pnh_.param("tcp_place_offset_x", tcp_place_offset_x_, tcp_place_offset_x_);
        pnh_.param("tcp_place_offset_y", tcp_place_offset_y_, tcp_place_offset_y_);

        loadPickYawCalibrationParams();

        nh_.param("/tetris/GRID_SIZE", GRID_SIZE_, GRID_SIZE_);
        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X_, BOARD_ORIGIN_X_);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y_, BOARD_ORIGIN_Y_);
        nh_.param("/tetris/PICK_Z", PICK_Z_, PICK_Z_);
        nh_.param("/tetris/PLACE_Z", PLACE_Z_, PLACE_Z_);
        nh_.param("/tetris/HOVER_Z", HOVER_Z_, HOVER_Z_);

        loadBoardMapIfAvailable();
        loadPickPlaneIfAvailable();
        loadPickAffineCorrectionIfAvailable();
        ROS_INFO("[CTRL] use_true_pick_plane=%s pick_plane_loaded=%s base_frame=%s",
                 use_true_pick_plane_ ? "true" : "false",
                 pick_plane_loaded_ ? "true" : "false",
                 base_frame_.c_str());
        ROS_INFO("[CTRL] place_z: use_cell_max=%s extra_margin=%.4f",
                 use_cell_max_place_z_ ? "true" : "false", place_z_extra_margin_);
        ROS_INFO("[CTRL] pick_affine: use=%s loaded=%s center=(%.2f, %.2f) coeff_x=(%.6g, %.6g, %.6g) coeff_y=(%.6g, %.6g, %.6g)",
                 use_pick_affine_correction_ ? "true" : "false",
                 pick_affine_loaded_ ? "true" : "false",
                 pick_affine_cx_, pick_affine_cy_,
                 pick_affine_x_[0], pick_affine_x_[1], pick_affine_x_[2],
                 pick_affine_y_[0], pick_affine_y_[1], pick_affine_y_[2]);
        ROS_INFO("[CTRL] TCP offsets: pick=(%.4f, %.4f, %.4f), place=(%.4f, %.4f)",
                 tcp_pick_offset_x_, tcp_pick_offset_y_, tcp_pick_offset_z_,
                 tcp_place_offset_x_, tcp_place_offset_y_);
    }

    static int normalizeDeg360(int a)
    {
        a %= 360;
        if (a < 0)
            a += 360;
        return a;
    }

    static int normalizeWay(int way)
    {
        way %= 4;
        if (way < 0)
            way += 4;
        return way;
    }

    void loadPickYawCalibrationParams()
    {
        for (auto &row : pick_yaw_corr_by_shape_way_)
            row.fill(0);

        // Coarse per-shape correction, applied to all target ways of that shape.
        // Shape IDs: 0=line, 1=square, 2=T, 3=L_left, 4=L_right, 5=Z_left, 6=Z_right.
        for (int shape = 0; shape < 7; ++shape)
        {
            int corr = 0;
            std::string key = "pick_yaw_corr_shape" + std::to_string(shape) + "_deg";
            pnh_.param(key, corr, 0);
            corr = normalizeDeg360(corr);
            for (int way = 0; way < 4; ++way)
                pick_yaw_corr_by_shape_way_[shape][way] = corr;
        }

        // Fine per-shape + per-target-way correction. This is what you should use
        // when only SOME L pieces are 180 deg off. Example launch param:
        //   <param name="pick_yaw_corr_shape3_way0_deg" value="180" />
        // means: L_left planned with way=0 uses visual_pick_angle + 180 deg.
        for (int shape = 0; shape < 7; ++shape)
        {
            for (int way = 0; way < 4; ++way)
            {
                int corr = pick_yaw_corr_by_shape_way_[shape][way];
                std::string key = "pick_yaw_corr_shape" + std::to_string(shape) +
                                  "_way" + std::to_string(way) + "_deg";
                pnh_.param(key, corr, corr);
                pick_yaw_corr_by_shape_way_[shape][way] = normalizeDeg360(corr);
            }
        }

        ROS_INFO("[CTRL] pick yaw correction table deg: shape0=[%d,%d,%d,%d] shape1=[%d,%d,%d,%d] shape2=[%d,%d,%d,%d] shape3=[%d,%d,%d,%d] shape4=[%d,%d,%d,%d] shape5=[%d,%d,%d,%d] shape6=[%d,%d,%d,%d]",
                 pick_yaw_corr_by_shape_way_[0][0], pick_yaw_corr_by_shape_way_[0][1], pick_yaw_corr_by_shape_way_[0][2], pick_yaw_corr_by_shape_way_[0][3],
                 pick_yaw_corr_by_shape_way_[1][0], pick_yaw_corr_by_shape_way_[1][1], pick_yaw_corr_by_shape_way_[1][2], pick_yaw_corr_by_shape_way_[1][3],
                 pick_yaw_corr_by_shape_way_[2][0], pick_yaw_corr_by_shape_way_[2][1], pick_yaw_corr_by_shape_way_[2][2], pick_yaw_corr_by_shape_way_[2][3],
                 pick_yaw_corr_by_shape_way_[3][0], pick_yaw_corr_by_shape_way_[3][1], pick_yaw_corr_by_shape_way_[3][2], pick_yaw_corr_by_shape_way_[3][3],
                 pick_yaw_corr_by_shape_way_[4][0], pick_yaw_corr_by_shape_way_[4][1], pick_yaw_corr_by_shape_way_[4][2], pick_yaw_corr_by_shape_way_[4][3],
                 pick_yaw_corr_by_shape_way_[5][0], pick_yaw_corr_by_shape_way_[5][1], pick_yaw_corr_by_shape_way_[5][2], pick_yaw_corr_by_shape_way_[5][3],
                 pick_yaw_corr_by_shape_way_[6][0], pick_yaw_corr_by_shape_way_[6][1], pick_yaw_corr_by_shape_way_[6][2], pick_yaw_corr_by_shape_way_[6][3]);
    }

    int correctedPickAngleDeg(int shape_type, int target_way, int raw_angle_deg) const
    {
        int a = normalizeDeg360(raw_angle_deg);
        if (shape_type < 0 || shape_type >= 7)
            return a;
        int way = normalizeWay(target_way);
        int corr = pick_yaw_corr_by_shape_way_[shape_type][way];
        return normalizeDeg360(a + corr);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        K_ = cv::Mat(3, 3, CV_64F);
        P_ = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 9; ++i)
            K_.at<double>(i / 3, i % 3) = msg->K[i];
        for (int i = 0; i < 12; ++i)
            P_.at<double>(i / 4, i % 4) = msg->P[i];

        D_ = cv::Mat(1, static_cast<int>(msg->D.size()), CV_64F);
        for (size_t i = 0; i < msg->D.size(); ++i)
            D_.at<double>(0, static_cast<int>(i)) = msg->D[i];

        if (camera_frame_.empty())
            camera_frame_ = msg->header.frame_id;
        camera_info_ready_ = true;
    }

    static bool xmlRpcToDouble(XmlRpc::XmlRpcValue &v, double &out)
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

    bool readPointList(XmlRpc::XmlRpcValue &value, geometry_msgs::Point &p)
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
            return false;
        double x, y, z;
        if (!xmlRpcToDouble(value[0], x) || !xmlRpcToDouble(value[1], y) || !xmlRpcToDouble(value[2], z))
            return false;
        p.x = x;
        p.y = y;
        p.z = z;
        return true;
    }

    void loadBoardMapIfAvailable()
    {
        if (!use_board_map_)
            return;

        XmlRpc::XmlRpcValue centers;
        if (nh_.getParam("/tetris/BOARD_CENTERS_14x10_TABLE", centers) &&
            centers.getType() == XmlRpc::XmlRpcValue::TypeArray && centers.size() == 14)
        {
            board_centers_.assign(14, std::vector<geometry_msgs::Point>(10));
            bool ok = true;
            for (int r = 0; r < 14 && ok; ++r)
            {
                if (centers[r].getType() != XmlRpc::XmlRpcValue::TypeArray || centers[r].size() != 10)
                {
                    ok = false;
                    break;
                }
                for (int c = 0; c < 10; ++c)
                {
                    if (!readPointList(centers[r][c], board_centers_[r][c]))
                    {
                        ok = false;
                        break;
                    }
                }
            }
            board_centers_loaded_ = ok;
            if (ok)
                ROS_INFO("[CTRL] Loaded /tetris/BOARD_CENTERS_14x10_TABLE.");
            else
                ROS_WARN("[CTRL] BOARD_CENTERS_14x10_TABLE exists but parsing failed; fallback to GRID_SIZE.");
        }

        XmlRpc::XmlRpcValue zmap;
        if (nh_.getParam("/tetris/PLACE_Z_MAP_14x10", zmap) &&
            zmap.getType() == XmlRpc::XmlRpcValue::TypeArray && zmap.size() == 14)
        {
            place_z_map_.assign(14, std::vector<double>(10, PLACE_Z_));
            bool ok = true;
            for (int r = 0; r < 14 && ok; ++r)
            {
                if (zmap[r].getType() != XmlRpc::XmlRpcValue::TypeArray || zmap[r].size() != 10)
                {
                    ok = false;
                    break;
                }
                for (int c = 0; c < 10; ++c)
                {
                    if (!xmlRpcToDouble(zmap[r][c], place_z_map_[r][c]))
                    {
                        ok = false;
                        break;
                    }
                }
            }
            place_z_map_loaded_ = ok;
            if (ok)
                ROS_INFO("[CTRL] Loaded /tetris/PLACE_Z_MAP_14x10.");
            else
                ROS_WARN("[CTRL] PLACE_Z_MAP_14x10 exists but parsing failed; fallback to PLACE_Z.");
        }
    }

    bool readVector3List(XmlRpc::XmlRpcValue &value, tf2::Vector3 &out)
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
            return false;

        double x, y, z;
        if (!xmlRpcToDouble(value[0], x) || !xmlRpcToDouble(value[1], y) || !xmlRpcToDouble(value[2], z))
            return false;

        out = tf2::Vector3(x, y, z);
        return true;
    }

    void loadPickPlaneIfAvailable()
    {
        if (!use_true_pick_plane_)
            return;

        XmlRpc::XmlRpcValue plane;
        if (!nh_.getParam("/tetris/PICK_SURFACE_PLANE_BASE", plane) ||
            plane.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[CTRL] /tetris/PICK_SURFACE_PLANE_BASE not found. Fallback to table_frame Z=PICK_Z.");
            pick_plane_loaded_ = false;
            return;
        }

        if (!plane.hasMember("point") || !plane.hasMember("normal"))
        {
            ROS_WARN("[CTRL] PICK_SURFACE_PLANE_BASE lacks point/normal. Fallback to table_frame Z=PICK_Z.");
            pick_plane_loaded_ = false;
            return;
        }

        tf2::Vector3 p, n;
        if (!readVector3List(plane["point"], p) || !readVector3List(plane["normal"], n))
        {
            ROS_WARN("[CTRL] Failed to parse PICK_SURFACE_PLANE_BASE. Fallback to table_frame Z=PICK_Z.");
            pick_plane_loaded_ = false;
            return;
        }

        if (n.length() < 1e-9)
        {
            ROS_WARN("[CTRL] PICK_SURFACE_PLANE_BASE normal has zero length. Fallback to table_frame Z=PICK_Z.");
            pick_plane_loaded_ = false;
            return;
        }

        pick_plane_point_base_ = p;
        pick_plane_normal_base_ = n.normalized();
        pick_plane_loaded_ = true;

        ROS_INFO("[CTRL] Loaded true pick plane in %s: point=(%.6f, %.6f, %.6f), normal=(%.6f, %.6f, %.6f)",
                 base_frame_.c_str(),
                 pick_plane_point_base_.x(), pick_plane_point_base_.y(), pick_plane_point_base_.z(),
                 pick_plane_normal_base_.x(), pick_plane_normal_base_.y(), pick_plane_normal_base_.z());
    }

    bool readDoubleArray(XmlRpc::XmlRpcValue &value, std::vector<double> &out)
    {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
            return false;

        out.clear();
        for (int i = 0; i < value.size(); ++i)
        {
            double x;
            if (!xmlRpcToDouble(value[i], x))
                return false;
            out.push_back(x);
        }
        return true;
    }

    void loadPickAffineCorrectionIfAvailable()
    {
        if (!use_pick_affine_correction_)
            return;

        XmlRpc::XmlRpcValue corr;
        if (!nh_.getParam("/tetris/PICK_AFFINE_CORRECTION", corr) ||
            corr.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[CTRL] /tetris/PICK_AFFINE_CORRECTION not found. Pick affine correction disabled.");
            pick_affine_loaded_ = false;
            return;
        }

        if (corr.hasMember("enabled"))
        {
            try
            {
                bool enabled = static_cast<bool>(corr["enabled"]);
                if (!enabled)
                {
                    ROS_WARN("[CTRL] PICK_AFFINE_CORRECTION exists but enabled=false.");
                    pick_affine_loaded_ = false;
                    return;
                }
            }
            catch (...)
            {
                // Ignore malformed enabled field and continue parsing coefficients.
            }
        }

        if (!corr.hasMember("pixel_center") || !corr.hasMember("coeff_x") || !corr.hasMember("coeff_y"))
        {
            ROS_WARN("[CTRL] PICK_AFFINE_CORRECTION lacks pixel_center/coeff_x/coeff_y. Disabled.");
            pick_affine_loaded_ = false;
            return;
        }

        std::vector<double> pc, cx, cy;
        if (!readDoubleArray(corr["pixel_center"], pc) ||
            !readDoubleArray(corr["coeff_x"], cx) ||
            !readDoubleArray(corr["coeff_y"], cy) ||
            pc.size() < 2 || cx.size() < 3 || cy.size() < 3)
        {
            ROS_WARN("[CTRL] Failed to parse PICK_AFFINE_CORRECTION arrays. Disabled.");
            pick_affine_loaded_ = false;
            return;
        }

        pick_affine_cx_ = pc[0];
        pick_affine_cy_ = pc[1];
        for (int i = 0; i < 3; ++i)
        {
            pick_affine_x_[i] = cx[i];
            pick_affine_y_[i] = cy[i];
        }

        pick_affine_loaded_ = true;

        ROS_INFO("[CTRL] Loaded pick affine correction: center=(%.3f, %.3f), x=(%.9g, %.9g, %.9g), y=(%.9g, %.9g, %.9g)",
                 pick_affine_cx_, pick_affine_cy_,
                 pick_affine_x_[0], pick_affine_x_[1], pick_affine_x_[2],
                 pick_affine_y_[0], pick_affine_y_[1], pick_affine_y_[2]);
    }

    static double clampDouble(double x, double lo, double hi)
    {
        return std::max(lo, std::min(hi, x));
    }

    geometry_msgs::Point bilinearBoardCenter(double row, double col) const
    {
        geometry_msgs::Point fallback;
        fallback.x = BOARD_ORIGIN_X_ - row * GRID_SIZE_;
        fallback.y = BOARD_ORIGIN_Y_ - col * GRID_SIZE_;
        fallback.z = PLACE_Z_;

        if (!board_centers_loaded_)
            return fallback;

        row = clampDouble(row, 0.0, 13.0);
        col = clampDouble(col, 0.0, 9.0);

        int r0 = static_cast<int>(std::floor(row));
        int c0 = static_cast<int>(std::floor(col));
        int r1 = std::min(13, r0 + 1);
        int c1 = std::min(9, c0 + 1);
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
        if (!place_z_map_loaded_)
            return PLACE_Z_;

        row = clampDouble(row, 0.0, 13.0);
        col = clampDouble(col, 0.0, 9.0);

        int r0 = static_cast<int>(std::floor(row));
        int c0 = static_cast<int>(std::floor(col));
        int r1 = std::min(13, r0 + 1);
        int c1 = std::min(9, c0 + 1);
        double tr = row - r0;
        double tc = col - c0;

        double z00 = place_z_map_[r0][c0];
        double z10 = place_z_map_[r1][c0];
        double z01 = place_z_map_[r0][c1];
        double z11 = place_z_map_[r1][c1];

        return (1 - tr) * (1 - tc) * z00 + tr * (1 - tc) * z10 + (1 - tr) * tc * z01 + tr * tc * z11;
    }

    double placeZFromTargetCells(const TaskGoal &goal) const
    {
        double z_center = bilinearPlaceZ(goal.place_grid_center.row, goal.place_grid_center.col);

        if (!goal.has_target_cells || goal.target_cells.empty() || !place_z_map_loaded_ || !use_cell_max_place_z_)
            return z_center + place_z_extra_margin_;

        double z_max = -1e9;
        for (const auto &cell : goal.target_cells)
        {
            int r = std::max(0, std::min(13, cell.row));
            int c = std::max(0, std::min(9, cell.col));
            z_max = std::max(z_max, place_z_map_[r][c]);
        }

        return z_max + place_z_extra_margin_;
    }

    geometry_msgs::Pose makeDownPose(double x, double y, double z, double yaw_rad) const
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, yaw_rad);
        pose.orientation = tf2::toMsg(q);
        return pose;
    }

    bool getCurrentCameraTransforms(geometry_msgs::TransformStamped &cam_to_table,
                                    geometry_msgs::TransformStamped &cam_to_base,
                                    geometry_msgs::TransformStamped &base_to_table)
    {
        if (camera_frame_.empty())
        {
            ROS_ERROR("[CTRL] camera_frame is empty and CameraInfo has not arrived.");
            return false;
        }

        try
        {
            cam_to_table = tf_buffer_.lookupTransform(table_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            base_to_table = tf_buffer_.lookupTransform(table_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));

            if (use_true_pick_plane_ && pick_plane_loaded_)
            {
                cam_to_base = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            }

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] TF lookup failed. table=%s base=%s camera=%s: %s",
                      table_frame_.c_str(), base_frame_.c_str(), camera_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool pixelToNormalizedRay(double u, double v, cv::Vec3d &ray) const
    {
        if (!camera_info_ready_)
            return false;

        if (assume_image_rectified_)
        {
            double fx = P_.at<double>(0, 0);
            double fy = P_.at<double>(1, 1);
            double cx = P_.at<double>(0, 2);
            double cy = P_.at<double>(1, 2);
            if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9)
                return false;
            ray = cv::Vec3d((u - cx) / fx, (v - cy) / fy, 1.0);
            return true;
        }

        std::vector<cv::Point2d> src;
        src.emplace_back(u, v);
        std::vector<cv::Point2d> undistorted;
        cv::undistortPoints(src, undistorted, K_, D_);
        if (undistorted.empty())
            return false;
        ray = cv::Vec3d(undistorted[0].x, undistorted[0].y, 1.0);
        return true;
    }

    bool pixelToTablePoint(const PixelPoint &px,
                           const geometry_msgs::TransformStamped &cam_to_table,
                           double plane_z,
                           geometry_msgs::Point &out) const
    {
        cv::Vec3d ray_cam;
        if (!pixelToNormalizedRay(px.u, px.v, ray_cam))
        {
            ROS_ERROR("[CTRL] Cannot project pixel (%d,%d): CameraInfo unavailable or invalid.", px.u, px.v);
            return false;
        }

        tf2::Quaternion q;
        tf2::fromMsg(cam_to_table.transform.rotation, q);
        tf2::Matrix3x3 R(q);

        tf2::Vector3 ray_table;
        ray_table.setX(R[0][0] * ray_cam[0] + R[0][1] * ray_cam[1] + R[0][2] * ray_cam[2]);
        ray_table.setY(R[1][0] * ray_cam[0] + R[1][1] * ray_cam[1] + R[1][2] * ray_cam[2]);
        ray_table.setZ(R[2][0] * ray_cam[0] + R[2][1] * ray_cam[1] + R[2][2] * ray_cam[2]);

        tf2::Vector3 cam_pos(
            cam_to_table.transform.translation.x,
            cam_to_table.transform.translation.y,
            cam_to_table.transform.translation.z);

        double denom = ray_table.z();
        if (std::abs(denom) < 1e-9)
        {
            ROS_ERROR("[CTRL] Camera ray is parallel to plane Z=%.6f.", plane_z);
            return false;
        }

        double t = (plane_z - cam_pos.z()) / denom;
        if (t < 0.0)
        {
            ROS_ERROR("[CTRL] Pixel ray intersects behind camera. Check TF/camera frame.");
            return false;
        }

        tf2::Vector3 p = cam_pos + ray_table * t;
        out.x = p.x();
        out.y = p.y();
        out.z = p.z();
        return true;
    }

    bool pixelToTruePickPlaneTablePoint(const PixelPoint &px,
                                        const geometry_msgs::TransformStamped &cam_to_base,
                                        const geometry_msgs::TransformStamped &base_to_table,
                                        geometry_msgs::Point &out_table) const
    {
        cv::Vec3d ray_cam;
        if (!pixelToNormalizedRay(px.u, px.v, ray_cam))
        {
            ROS_ERROR("[CTRL] Cannot project pixel (%d,%d): CameraInfo unavailable or invalid.", px.u, px.v);
            return false;
        }

        tf2::Quaternion q;
        tf2::fromMsg(cam_to_base.transform.rotation, q);
        tf2::Matrix3x3 R(q);

        tf2::Vector3 ray_base;
        ray_base.setX(R[0][0] * ray_cam[0] + R[0][1] * ray_cam[1] + R[0][2] * ray_cam[2]);
        ray_base.setY(R[1][0] * ray_cam[0] + R[1][1] * ray_cam[1] + R[1][2] * ray_cam[2]);
        ray_base.setZ(R[2][0] * ray_cam[0] + R[2][1] * ray_cam[1] + R[2][2] * ray_cam[2]);

        tf2::Vector3 cam_pos_base(
            cam_to_base.transform.translation.x,
            cam_to_base.transform.translation.y,
            cam_to_base.transform.translation.z);

        double denom = pick_plane_normal_base_.dot(ray_base);
        if (std::abs(denom) < 1e-9)
        {
            ROS_ERROR("[CTRL] Camera ray is parallel to true pick plane.");
            return false;
        }

        double t = pick_plane_normal_base_.dot(pick_plane_point_base_ - cam_pos_base) / denom;
        if (t < 0.0)
        {
            ROS_ERROR("[CTRL] Pixel ray intersects true pick plane behind camera. Check TF/camera frame.");
            return false;
        }

        tf2::Vector3 hit_base = cam_pos_base + ray_base * t;

        geometry_msgs::PointStamped p_base, p_table;
        p_base.header.frame_id = base_frame_;
        p_base.header.stamp = ros::Time(0);
        p_base.point.x = hit_base.x();
        p_base.point.y = hit_base.y();
        p_base.point.z = hit_base.z();

        try
        {
            tf2::doTransform(p_base, p_table, base_to_table);
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[CTRL] Failed to transform true pick hit base->table: %s", ex.what());
            return false;
        }

        out_table = p_table.point;

        ROS_INFO("[CTRL] true pick plane hit: base=(%.6f, %.6f, %.6f) -> table=(%.6f, %.6f, %.6f)",
                 hit_base.x(), hit_base.y(), hit_base.z(),
                 out_table.x, out_table.y, out_table.z);

        return true;
    }

    bool pixelToPickPointTable(const PixelPoint &px, geometry_msgs::Point &out_table) const
    {
        if (use_true_pick_plane_ && pick_plane_loaded_)
        {
            return pixelToTruePickPlaneTablePoint(px, observation_cam_to_base_, observation_base_to_table_, out_table);
        }

        return pixelToTablePoint(px, observation_cam_to_table_, PICK_Z_, out_table);
    }

    void applyPickAffineCorrection(const PixelPoint &px, geometry_msgs::Point &p) const
    {
        if (!use_pick_affine_correction_ || !pick_affine_loaded_)
            return;

        double du = static_cast<double>(px.u) - pick_affine_cx_;
        double dv = static_cast<double>(px.v) - pick_affine_cy_;

        double dx = pick_affine_x_[0] + pick_affine_x_[1] * du + pick_affine_x_[2] * dv;
        double dy = pick_affine_y_[0] + pick_affine_y_[1] * du + pick_affine_y_[2] * dv;

        p.x += dx;
        p.y += dy;

        ROS_INFO("[CTRL] pick affine correction for pixel=(%d,%d): du=%.1f dv=%.1f => dX=%.3f mm dY=%.3f mm",
                 px.u, px.v, du, dv, dx * 1000.0, dy * 1000.0);
    }

    geometry_msgs::Pose buildPickPose(const PixelPoint &px, int angle_deg) const
    {
        geometry_msgs::Point p;
        if (!pixelToPickPointTable(px, p))
        {
            throw std::runtime_error("pixelToPickPointTable failed");
        }

        applyPickAffineCorrection(px, p);

        double yaw = angle_deg * M_PI / 180.0;
        geometry_msgs::Pose pose = makeDownPose(p.x + tcp_pick_offset_x_, p.y + tcp_pick_offset_y_, p.z + tcp_pick_offset_z_, yaw);
        ROS_INFO("[CTRL] pick pixel=(%d,%d), angle=%d => table pick=(%.6f, %.6f, %.6f), yaw=%.1f deg",
                 px.u, px.v, angle_deg, pose.position.x, pose.position.y, pose.position.z, angle_deg * 1.0);
        return pose;
    }

    geometry_msgs::Pose buildPlacePose(const TaskGoal &goal) const
    {
        double center_row = goal.place_grid_center.row;
        double center_col = goal.place_grid_center.col;
        int way = goal.way;

        geometry_msgs::Point center = bilinearBoardCenter(center_row, center_col);
        double z = placeZFromTargetCells(goal);
        double yaw = way * M_PI / 2.0;

        geometry_msgs::Pose pose = makeDownPose(
            center.x + tcp_place_offset_x_,
            center.y + tcp_place_offset_y_,
            z,
            yaw);

        ROS_INFO("[CTRL] place grid center=(%.3f, %.3f), way=%d => table place=(%.6f, %.6f, %.6f), yaw=%.1f deg, has_cells=%s",
                 center_row, center_col, way, pose.position.x, pose.position.y, pose.position.z,
                 way * 90.0, goal.has_target_cells ? "true" : "false");

        if (goal.has_target_cells)
        {
            std::string cells = "";
            for (const auto &cell : goal.target_cells)
            {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "(%d,%d) ", cell.row, cell.col);
                cells += buf;
            }
            ROS_INFO("[CTRL] target cells: %s", cells.c_str());
        }

        return pose;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty())
            return;

        if (!camera_info_ready_)
        {
            ROS_WARN("[CTRL] Received plan but CameraInfo is not ready yet. Ignoring this plan.");
            return;
        }

        geometry_msgs::TransformStamped cam_to_table;
        geometry_msgs::TransformStamped cam_to_base;
        geometry_msgs::TransformStamped base_to_table;
        if (!getCurrentCameraTransforms(cam_to_table, cam_to_base, base_to_table))
        {
            ROS_WARN("[CTRL] Received plan but required TF is not ready. Ignoring this plan.");
            return;
        }

        observation_cam_to_table_ = cam_to_table;
        observation_cam_to_base_ = cam_to_base;
        observation_base_to_table_ = base_to_table;
        observation_tf_ready_ = true;

        while (!tasks_.empty())
            tasks_.pop();

        int total = msg->data[0];
        int expected_old = 1 + total * 7;
        int expected_ext = 1 + total * 15;
        bool extended_plan = false;

        if (static_cast<int>(msg->data.size()) >= expected_ext)
        {
            extended_plan = true;
        }
        else if (static_cast<int>(msg->data.size()) >= expected_old)
        {
            extended_plan = false;
        }
        else
        {
            ROS_ERROR("[CTRL] Invalid plan length: got %lu, expected at least %d old-format or %d extended-format.",
                      msg->data.size(), expected_old, expected_ext);
            return;
        }

        ROS_INFO("[CTRL] Received plan with %d steps. format=%s. Captured observation camera TF.",
                 total, extended_plan ? "extended-15" : "old-7");

        int idx = 1;
        for (int i = 0; i < total; ++i)
        {
            TaskGoal goal;
            goal.shape_type = msg->data[idx++];
            goal.way = msg->data[idx++];

            int sum_r = msg->data[idx++];
            int sum_c = msg->data[idx++];
            goal.place_grid_center.row = sum_r / 4.0;
            goal.place_grid_center.col = sum_c / 4.0;

            goal.pick_pixel.u = msg->data[idx++];
            goal.pick_pixel.v = msg->data[idx++];
            goal.pick_angle_deg = msg->data[idx++];

            if (extended_plan)
            {
                goal.has_target_cells = true;
                goal.target_cells.clear();
                for (int k = 0; k < 4; ++k)
                {
                    GridCell cell;
                    cell.row = msg->data[idx++];
                    cell.col = msg->data[idx++];
                    goal.target_cells.push_back(cell);
                }
            }

            try
            {
                int corrected_pick_angle = correctedPickAngleDeg(goal.shape_type, goal.way, goal.pick_angle_deg);
                if (corrected_pick_angle != normalizeDeg360(goal.pick_angle_deg))
                {
                    ROS_WARN("[CTRL] pick yaw corrected: shape=%d way=%d raw=%d corrected=%d",
                             goal.shape_type, goal.way, goal.pick_angle_deg, corrected_pick_angle);
                }
                goal.pick_pose = buildPickPose(goal.pick_pixel, corrected_pick_angle);
                goal.place_pose = buildPlacePose(goal);
            }
            catch (const std::exception &e)
            {
                ROS_WARN("[CTRL] Skip task %d: %s", i, e.what());
                continue;
            }

            tasks_.push(goal);
        }

        ROS_INFO("[CTRL] Queued %lu executable tasks.", tasks_.size());
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

    bool setSuction(bool on)
    {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = suction_io_num_;
        srv.request.value = on ? 1 : 0;
        if (!io_client_.call(srv))
        {
            ROS_ERROR("[CTRL] Failed to call suction IO service.");
            return false;
        }
        ros::Duration(on ? suction_on_wait_ : suction_off_wait_).sleep();
        return true;
    }

    bool moveToPose(const geometry_msgs::Pose &pose, const std::string &label)
    {
        move_group_.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_.plan(plan);

        if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("[CTRL] Plan failed for %s at (%.6f, %.6f, %.6f).",
                      label.c_str(), pose.position.x, pose.position.y, pose.position.z);
            return false;
        }

        auto exec_result = move_group_.execute(plan);
        if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_ERROR("[CTRL] Execute failed for %s.", label.c_str());
            return false;
        }

        return true;
    }

    geometry_msgs::Pose hoverFrom(const geometry_msgs::Pose &pose) const
    {
        geometry_msgs::Pose h = pose;
        h.position.z = HOVER_Z_;
        return h;
    }

    bool applyOptionalPreciseCorrection()
    {
        if (!use_precise_service_)
            return true;

        tly::GetPrecisePose srv;
        srv.request.target_shape_type = current_task_.shape_type;

        if (!precise_client_.call(srv) || !srv.response.success)
        {
            ROS_WARN("[CTRL] Precise service failed; keeping original pick pose.");
            return true;
        }

        if (!camera_info_ready_)
            return true;

        // Compatibility mode with the old service:
        // vision returns equivalent pixel dx/dy, controller converts them back to meters
        // around the current hover height.
        double fx = assume_image_rectified_ ? P_.at<double>(0, 0) : K_.at<double>(0, 0);
        double fy = assume_image_rectified_ ? P_.at<double>(1, 1) : K_.at<double>(1, 1);
        double z_dist = std::max(std::abs(HOVER_Z_ - PICK_Z_), 1e-4);

        double dX = srv.response.dy * z_dist / fy;
        double dY = srv.response.dx * z_dist / fx;

        current_task_.pick_pose.position.x += dX;
        current_task_.pick_pose.position.y += dY;

        double yaw = srv.response.angle * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, yaw);
        current_task_.pick_pose.orientation = tf2::toMsg(q);

        ROS_INFO("[CTRL] Optional precise correction: dx_pix=%d dy_pix=%d => dX=%.5f dY=%.5f angle=%d",
                 srv.response.dx, srv.response.dy, dX, dY, srv.response.angle);
        return true;
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
            ROS_INFO("[CTRL] Next task: shape=%d way=%d pick_px=(%d,%d) pick_angle=%d remaining=%lu",
                     current_task_.shape_type, current_task_.way,
                     current_task_.pick_pixel.u, current_task_.pick_pixel.v,
                     current_task_.pick_angle_deg, tasks_.size());
            state_ = State::MOVE_TO_PICK_HOVER;
            return;

        case State::MOVE_TO_PICK_HOVER:
        {
            geometry_msgs::Pose hover = hoverFrom(current_task_.pick_pose);
            if (!moveToPose(hover, "pick_hover"))
            {
                failOrFinish("Move to pick hover failed.");
                return;
            }
            state_ = State::OPTIONAL_PRECISE_CORRECTION;
            return;
        }

        case State::OPTIONAL_PRECISE_CORRECTION:
            applyOptionalPreciseCorrection();
            state_ = State::EXECUTE_PICK;
            return;

        case State::EXECUTE_PICK:
        {
            if (!moveToPose(current_task_.pick_pose, "pick"))
            {
                failOrFinish("Move to pick pose failed.");
                return;
            }
            setSuction(true);

            geometry_msgs::Pose hover = hoverFrom(current_task_.pick_pose);
            if (!moveToPose(hover, "lift_after_pick"))
            {
                failOrFinish("Lift after pick failed.");
                return;
            }
            state_ = State::MOVE_TO_PLACE_HOVER;
            return;
        }

        case State::MOVE_TO_PLACE_HOVER:
        {
            geometry_msgs::Pose hover = hoverFrom(current_task_.place_pose);
            if (!moveToPose(hover, "place_hover"))
            {
                failOrFinish("Move to place hover failed.");
                return;
            }
            state_ = State::EXECUTE_PLACE;
            return;
        }

        case State::EXECUTE_PLACE:
        {
            if (!moveToPose(current_task_.place_pose, "place"))
            {
                failOrFinish("Move to place pose failed.");
                return;
            }
            setSuction(false);

            geometry_msgs::Pose hover = hoverFrom(current_task_.place_pose);
            if (!moveToPose(hover, "lift_after_place"))
            {
                failOrFinish("Lift after place failed.");
                return;
            }
            state_ = State::TAKE_NEXT_TASK;
            return;
        }

        case State::FINISH:
            ROS_INFO("[CTRL] All tasks finished.");
            publishBusy(false);
            state_ = State::IDLE;
            return;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_controller_node");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    XarmTetrisController controller;
    ros::waitForShutdown();
    return 0;
}