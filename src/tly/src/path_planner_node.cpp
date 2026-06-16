// 路径规划节点 (Path Planner)
//
// 职责（步骤5a，附加式）：消费 /tetris_plan，把每个任务的「抓取像素 / 放置格子」
// 解算成 base 系可直接 move_line 的抓放位姿，发布 /motion_cmds。
//   - 抓取 XY：单应性矩阵（与控制节点一致）；抓取 Z：RealSense 对齐深度（新），
//     深度无效时回退到平面拟合 Z。
//   - 放置 XY/Z：白板格心双线性插值 + PLACE_Z_MAP（与控制节点一致）。
//   - 顺序：沿用 /tetris_plan 原有顺序（直通）；最短路径排序留待后续步骤。
//
// 本步骤不改动控制节点：它仍消费 /tetris_plan。/motion_cmds 供 rostopic echo 对照
// 验证（日志同时打印 z_plane 与 z_depth）。坐标解算逻辑从 xarm_controller_node.cpp
// 原样移植，保证与现有行为一致；yaw 的 0/180° 关节翻转选择属控制侧逻辑，未移植。

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <XmlRpcValue.h>

#include <tly/MotionPlan.h>
#include <tly/MotionTask.h>
#include <tly/depth_sampler.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
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
};

class PathPlanner
{
public:
    PathPlanner() : pnh_("~"), tf_listener_(tf_buffer_)
    {
        loadParams();

        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &PathPlanner::cameraInfoCallback, this);

        ROS_INFO("[PATH] Waiting for CameraInfo on %s ...", camera_info_topic_.c_str());
        sensor_msgs::CameraInfoConstPtr cam_msg =
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));
        if (cam_msg)
            cameraInfoCallback(cam_msg);
        else
            ROS_WARN("[PATH] CameraInfo not received within 5 seconds.");

        if (use_depth_pick_z_)
            depth_sampler_.init(nh_, depth_topic_);

        plan_sub_ = nh_.subscribe(plan_topic_, 1, &PathPlanner::planCallback, this);
        motion_pub_ = nh_.advertise<tly::MotionPlan>(motion_topic_, 1, true);

        ROS_INFO("[PATH] ready: in=%s out=%s depth_pick_z=%s xy_source=%s",
                 plan_topic_.c_str(), motion_topic_.c_str(),
                 use_depth_pick_z_ ? "on" : "off",
                 pick_xy_source_depth_ ? "depth" : "homography");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber plan_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher motion_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string plan_topic_ = "/tetris_plan";
    std::string motion_topic_ = "/motion_cmds";
    std::string camera_info_topic_ = "/camera/color/camera_info";
    std::string table_frame_ = "table_frame";
    std::string base_frame_ = "link_base";
    std::string camera_frame_ = "camera_color_optical_frame";

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
    double pick_homography_H_[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

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
    double yaw_homography_v_sign_ = 1.0;
    double tcp_place_offset_x_ = 0.0;
    double tcp_place_offset_y_ = 0.0;
    double tcp_place_offset_z_ = 0.0;
    double place_release_z_margin_ = 0.004;

    double fixed_roll_rad_ = M_PI;
    double fixed_pitch_rad_ = 0.0;

    int max_tasks_per_plan_ = 0; // 0 = 不限制，发布策略给的全部任务

    // 深度采样
    tly::DepthSampler depth_sampler_;
    bool use_depth_pick_z_ = true;
    std::string depth_topic_ = "/camera/aligned_depth_to_color/image_raw";
    int depth_sample_radius_px_ = 4;
    bool depth_sample_in_raw_color_ = true;
    bool pick_xy_source_depth_ = false; // false=homography(默认), true=深度反投影

    bool camera_info_ready_ = false;
    cv::Mat K_, D_, P_;

    geometry_msgs::TransformStamped observation_cam_to_table_;
    geometry_msgs::TransformStamped observation_cam_to_base_;
    geometry_msgs::TransformStamped observation_base_to_table_;

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
        pnh_.param("motion_topic", motion_topic_, motion_topic_);
        pnh_.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
        pnh_.param("table_frame", table_frame_, table_frame_);
        pnh_.param("base_frame", base_frame_, base_frame_);
        pnh_.param("camera_frame", camera_frame_, camera_frame_);

        pnh_.param("use_true_pick_plane", use_true_pick_plane_, use_true_pick_plane_);
        pnh_.param("use_pick_homography", use_pick_homography_, use_pick_homography_);
        pnh_.param("use_board_map", use_board_map_, use_board_map_);
        pnh_.param("use_cell_max_place_z", use_cell_max_place_z_, use_cell_max_place_z_);
        pnh_.param("require_board_map", require_board_map_, require_board_map_);
        pnh_.param("require_place_z_map", require_place_z_map_, require_place_z_map_);

        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        pnh_.param("tcp_pick_offset_z", tcp_pick_offset_z_, tcp_pick_offset_z_);

        double pick_yaw_offset_deg = 0.0;
        pnh_.param("pick_yaw_offset_deg", pick_yaw_offset_deg, 0.0);
        pick_yaw_offset_rad_ = pick_yaw_offset_deg * M_PI / 180.0;
        pnh_.param("pick_yaw_offset_rad", pick_yaw_offset_rad_, pick_yaw_offset_rad_);
        pnh_.param("yaw_homography_probe_px", yaw_homography_probe_px_, yaw_homography_probe_px_);
        pnh_.param("yaw_homography_v_sign", yaw_homography_v_sign_, yaw_homography_v_sign_);
        yaw_homography_v_sign_ = (yaw_homography_v_sign_ >= 0.0) ? 1.0 : -1.0;

        pnh_.param("tcp_place_offset_x", tcp_place_offset_x_, tcp_place_offset_x_);
        pnh_.param("tcp_place_offset_y", tcp_place_offset_y_, tcp_place_offset_y_);
        pnh_.param("tcp_place_offset_z", tcp_place_offset_z_, tcp_place_offset_z_);
        pnh_.param("place_release_z_margin", place_release_z_margin_, place_release_z_margin_);

        pnh_.param("fixed_roll_rad", fixed_roll_rad_, fixed_roll_rad_);
        pnh_.param("fixed_pitch_rad", fixed_pitch_rad_, fixed_pitch_rad_);

        pnh_.param("max_tasks_per_plan", max_tasks_per_plan_, max_tasks_per_plan_);

        pnh_.param("use_depth_pick_z", use_depth_pick_z_, use_depth_pick_z_);
        pnh_.param("depth_topic", depth_topic_, depth_topic_);
        pnh_.param("depth_sample_radius_px", depth_sample_radius_px_, depth_sample_radius_px_);
        pnh_.param("depth_sample_in_raw_color", depth_sample_in_raw_color_, depth_sample_in_raw_color_);
        std::string xy_source = "homography";
        pnh_.param("pick_xy_source", xy_source, xy_source);
        pick_xy_source_depth_ = (xy_source == "depth");

        nh_.param("/tetris/GRID_SIZE", GRID_SIZE_, GRID_SIZE_);
        nh_.param("/tetris/BOARD_ORIGIN_X", BOARD_ORIGIN_X_, BOARD_ORIGIN_X_);
        nh_.param("/tetris/BOARD_ORIGIN_Y", BOARD_ORIGIN_Y_, BOARD_ORIGIN_Y_);
        nh_.param("/tetris/PICK_Z", PICK_Z_, PICK_Z_);
        nh_.param("/tetris/PLACE_Z", PLACE_Z_, PLACE_Z_);
        nh_.param("/tetris/HOVER_Z", HOVER_Z_, HOVER_Z_);

        board_centers_loaded_ = loadBoardCenters14x10();
        place_z_map_loaded_ = loadPlaceZMap14x10();
        pick_plane_loaded_ = loadPickPlaneIfAvailable();
        pick_homography_loaded_ = loadPickHomographyIfAvailable();

        ROS_INFO("[PATH] calib: board=%s zmap=%s pick_plane=%s pick_homography=%s",
                 board_centers_loaded_ ? "loaded" : "MISSING",
                 place_z_map_loaded_ ? "loaded" : "MISSING",
                 pick_plane_loaded_ ? "loaded" : "flat_PICK_Z",
                 pick_homography_loaded_ ? "loaded" : "off");
    }

    bool loadBoardCenters14x10()
    {
        if (!use_board_map_)
            return false;
        XmlRpc::XmlRpcValue centers;
        if (!nh_.getParam("/tetris/BOARD_CENTERS_14x10_TABLE", centers) ||
            centers.getType() != XmlRpc::XmlRpcValue::TypeArray || centers.size() <= 0)
        {
            ROS_WARN("[PATH] /tetris/BOARD_CENTERS_14x10_TABLE not found.");
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
            ROS_WARN("[PATH] /tetris/PLACE_Z_MAP_14x10 not found.");
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
            ROS_WARN("[PATH] /tetris/PICK_SURFACE_PLANE_BASE not found; fallback to flat PICK_Z.");
            return false;
        }
        tf2::Vector3 p, n;
        if (!readVector3List(plane["point"], p) || !readVector3List(plane["normal"], n) || n.length() < 1e-9)
        {
            ROS_WARN("[PATH] Invalid PICK_SURFACE_PLANE_BASE; fallback to flat PICK_Z.");
            return false;
        }
        pick_plane_point_base_ = p;
        pick_plane_normal_base_ = n.normalized();
        return true;
    }

    bool loadPickHomographyIfAvailable()
    {
        if (!use_pick_homography_)
            return false;
        XmlRpc::XmlRpcValue hg;
        if (!nh_.getParam("/tetris/PICK_HOMOGRAPHY", hg) || hg.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[PATH] /tetris/PICK_HOMOGRAPHY not found; raw true-plane XY will be used.");
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
        return true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        P_ = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 12; ++i)
            P_.at<double>(i / 4, i % 4) = msg->P[i];
        K_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i)
            K_.at<double>(i / 3, i % 3) = msg->K[i];
        D_ = msg->D.empty() ? cv::Mat::zeros(5, 1, CV_64F) : cv::Mat(msg->D).clone();
        if (!msg->header.frame_id.empty())
            camera_frame_ = msg->header.frame_id;
        camera_info_ready_ = true;
    }

    bool getObservationTransforms()
    {
        try
        {
            observation_cam_to_table_ = tf_buffer_.lookupTransform(table_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            if (use_true_pick_plane_ && pick_plane_loaded_)
            {
                observation_cam_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
                observation_base_to_table_ = tf_buffer_.lookupTransform(table_frame_, base_frame_, ros::Time(0), ros::Duration(1.0));
            }
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[PATH] TF lookup failed: %s", ex.what());
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
            ROS_ERROR("[PATH] base->table transform for pick point failed: %s", ex.what());
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

    // 抓取点(table)：单应性 XY + 平面 Z + 2.5D 视差补偿（与控制节点一致）。
    bool pixelToPickPointTable(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        bool ok = false;
        if (use_true_pick_plane_ && pick_plane_loaded_)
            ok = pixelToTruePickPlaneTablePoint(px, out);
        else
            ok = pixelToFlatPickZTablePoint(px, out);
        if (!ok)
            return false;
        if (!use_pick_homography_ || !pick_homography_loaded_)
            return true;

        double u_flat = px.u;
        double v_flat = px.v;
        if (camera_info_ready_ && !P_.empty())
        {
            double cx = P_.at<double>(0, 2);
            double cy = P_.at<double>(1, 2);
            double cam_z = std::abs(observation_cam_to_table_.transform.translation.z);
            double block_z = std::abs(out.z);
            if (cam_z > block_z + 0.05)
            {
                double ratio = (cam_z - block_z) / cam_z;
                u_flat = cx + (px.u - cx) * ratio;
                v_flat = cy + (px.v - cy) * ratio;
            }
        }
        geometry_msgs::Point hom_p;
        if (applyPickHomographyXY(u_flat, v_flat, hom_p))
        {
            out.x = hom_p.x;
            out.y = hom_p.y;
            return true;
        }
        return false;
    }

    // 把 rectified 像素映射回 raw(带畸变)彩色像素，用于对齐深度采样（与视觉节点一致）。
    cv::Point2f rectifiedToRaw(double u, double v) const
    {
        if (!camera_info_ready_ || !depth_sample_in_raw_color_ || P_.empty() || K_.empty())
            return cv::Point2f((float)u, (float)v);
        double fx = P_.at<double>(0, 0), fy = P_.at<double>(1, 1);
        double cx = P_.at<double>(0, 2), cy = P_.at<double>(1, 2);
        if (std::abs(fx) < 1e-9 || std::abs(fy) < 1e-9)
            return cv::Point2f((float)u, (float)v);
        double x = (u - cx) / fx, y = (v - cy) / fy;
        std::vector<cv::Point3f> obj = {cv::Point3f((float)x, (float)y, 1.0f)};
        std::vector<cv::Point2f> img;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F), tvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::projectPoints(obj, rvec, tvec, K_, D_, img);
        return img[0];
    }

    // 用对齐深度求抓取点的 table 系坐标：在 raw 像素采样 Z(相机系)，
    // 用 rectified 射线得到相机系 3D 点，再经 cam->table 变换到 table 系。
    bool depthPickPointTable(const PixelPoint &px, geometry_msgs::Point &out) const
    {
        if (!depth_sampler_.ready() || !camera_info_ready_)
            return false;
        cv::Point2f raw = rectifiedToRaw(px.u, px.v);
        double z_cam = -1.0;
        if (!depth_sampler_.sampleZ((int)std::round(raw.x), (int)std::round(raw.y), depth_sample_radius_px_, z_cam))
            return false;
        cv::Vec3d ray;
        if (!pixelToNormalizedRay(px, ray))
            return false;
        tf2::Vector3 p_cam(ray[0] * z_cam, ray[1] * z_cam, z_cam);
        tf2::Quaternion q;
        tf2::fromMsg(observation_cam_to_table_.transform.rotation, q);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 T(observation_cam_to_table_.transform.translation.x,
                       observation_cam_to_table_.transform.translation.y,
                       observation_cam_to_table_.transform.translation.z);
        tf2::Vector3 p_t = R * p_cam + T;
        out.x = p_t.x();
        out.y = p_t.y();
        out.z = p_t.z();
        return true;
    }

    // 综合抓取点：XY 默认单应性，Z 优先深度（无效回退平面）；记录两种 Z 供对照。
    bool computePickTable(const PixelPoint &px, geometry_msgs::Point &out, double &z_plane, double &z_depth) const
    {
        z_plane = std::numeric_limits<double>::quiet_NaN();
        z_depth = std::numeric_limits<double>::quiet_NaN();

        geometry_msgs::Point plane_pt;
        bool plane_ok = pixelToPickPointTable(px, plane_pt);
        if (plane_ok)
            z_plane = plane_pt.z;

        geometry_msgs::Point dpt;
        bool depth_ok = use_depth_pick_z_ && depthPickPointTable(px, dpt);
        if (depth_ok)
            z_depth = dpt.z;

        if (depth_ok)
        {
            out.z = dpt.z;
            if (pick_xy_source_depth_)
            {
                out.x = dpt.x;
                out.y = dpt.y;
            }
            else if (plane_ok)
            {
                out.x = plane_pt.x;
                out.y = plane_pt.y;
            }
            else
            {
                out.x = dpt.x;
                out.y = dpt.y;
            }
            return true;
        }
        if (plane_ok)
        {
            out = plane_pt;
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

    bool tableToBasePose(const geometry_msgs::Pose &table_pose, geometry_msgs::Pose &base_pose)
    {
        geometry_msgs::PoseStamped ps_t, ps_b;
        ps_t.header.frame_id = table_frame_;
        ps_t.header.stamp = ros::Time(0);
        ps_t.pose = table_pose;
        try
        {
            tf_buffer_.transform(ps_t, ps_b, base_frame_, ros::Duration(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[PATH] table->base transform failed: %s", ex.what());
            return false;
        }
        base_pose = ps_b.pose;
        return true;
    }

    // 移植自控制节点 buildTask 的几何部分；不含 yaw 0/180° 关节翻转选择（属控制侧）。
    bool buildTask(const TaskGoal &goal, tly::MotionTask &out_task)
    {
        geometry_msgs::Point pick_p;
        double z_plane = std::numeric_limits<double>::quiet_NaN();
        double z_depth = std::numeric_limits<double>::quiet_NaN();
        if (!computePickTable(goal.pick_pixel, pick_p, z_plane, z_depth))
            return false;

        double pick_yaw = normalizeAngleRad(yawFromHomography(goal.pick_pixel.u, goal.pick_pixel.v,
                                                              goal.pick_angle_deg * M_PI / 180.0) +
                                            pick_yaw_offset_rad_);
        double place_yaw = normalizeAngleRad(-goal.way * M_PI / 2.0);

        double pick_x = pick_p.x + std::cos(pick_yaw) * tcp_pick_offset_x_ - std::sin(pick_yaw) * tcp_pick_offset_y_;
        double pick_y = pick_p.y + std::sin(pick_yaw) * tcp_pick_offset_x_ + std::cos(pick_yaw) * tcp_pick_offset_y_;
        double pick_z = pick_p.z + tcp_pick_offset_z_;
        geometry_msgs::Pose pick_table = makePoseTable(pick_x, pick_y, pick_z, pick_yaw);

        double pmgx = 0.0, pmgy = 0.0;
        bool has_off = goal.has_geom_pixel && computePickToGeomOffset(goal.pick_pixel, goal.geom_pixel, pmgx, pmgy);

        geometry_msgs::Point bc = bilinearBoardCenter(goal.place_grid_center.row, goal.place_grid_center.col);
        double place_x = bc.x, place_y = bc.y;
        if (has_off)
        {
            double delta = place_yaw - pick_yaw;
            double dx_rot = pmgx * std::cos(delta) - pmgy * std::sin(delta);
            double dy_rot = pmgx * std::sin(delta) + pmgy * std::cos(delta);
            place_x += dx_rot;
            place_y += dy_rot;
        }
        place_x += std::cos(place_yaw) * tcp_place_offset_x_ - std::sin(place_yaw) * tcp_place_offset_y_;
        place_y += std::sin(place_yaw) * tcp_place_offset_x_ + std::cos(place_yaw) * tcp_place_offset_y_;
        double place_z = placeZFromTargetCells(goal) + place_release_z_margin_ + tcp_place_offset_z_;
        geometry_msgs::Pose place_table = makePoseTable(place_x, place_y, place_z, place_yaw);

        geometry_msgs::Pose pick_base, place_base;
        if (!tableToBasePose(pick_table, pick_base) || !tableToBasePose(place_table, place_base))
            return false;

        out_task.shape = goal.shape_type;
        out_task.way = goal.way;
        out_task.pick_pose = pick_base;
        out_task.place_pose = place_base;

        char zdbuf[32];
        if (std::isfinite(z_depth))
            std::snprintf(zdbuf, sizeof(zdbuf), "%.3f", z_depth);
        else
            std::snprintf(zdbuf, sizeof(zdbuf), "NA");
        ROS_INFO("[PATH][TASK] shape=%d pick_table=(%.3f,%.3f,%.3f) place_table=(%.3f,%.3f,%.3f) "
                 "yaw_pick=%.1f yaw_place=%.1f z_plane=%.3f z_depth=%s",
                 goal.shape_type, pick_x, pick_y, pick_z, place_x, place_y, place_z,
                 pick_yaw * 180.0 / M_PI, place_yaw * 180.0 / M_PI, z_plane, zdbuf);
        return true;
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty() || !camera_info_ready_)
        {
            ROS_WARN("[PATH] Drop plan: empty or CameraInfo not ready.");
            return;
        }
        if (require_board_map_ && !board_centers_loaded_)
        {
            ROS_ERROR("[PATH] Refuse plan: required BOARD_CENTERS_14x10_TABLE missing.");
            return;
        }
        if (require_place_z_map_ && !place_z_map_loaded_)
        {
            ROS_ERROR("[PATH] Refuse plan: required PLACE_Z_MAP_14x10 missing.");
            return;
        }
        if (!getObservationTransforms())
            return;

        int total = msg->data[0];
        if (total <= 0)
            return;
        int stride = (msg->data.size() - 1) / total;
        if (stride < 7)
        {
            ROS_ERROR("[PATH] Invalid plan stride=%d", stride);
            return;
        }

        int exec_total = total;
        if (max_tasks_per_plan_ > 0 && exec_total > max_tasks_per_plan_)
            exec_total = max_tasks_per_plan_;

        tly::MotionPlan plan;
        plan.header.stamp = ros::Time::now();
        plan.header.frame_id = base_frame_;

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

            tly::MotionTask task;
            if (buildTask(goal, task))
                plan.tasks.push_back(task);
            else
                ROS_WARN("[PATH] Skip task %d: failed to build pick/place pose.", i + 1);
        }

        motion_pub_.publish(plan);
        ROS_INFO("[PATH] Published /motion_cmds with %lu task(s) (from %d plan task(s)).",
                 plan.tasks.size(), total);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    PathPlanner node;
    ros::spin();
    return 0;
}
