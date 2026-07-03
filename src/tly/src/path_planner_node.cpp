// 路径规划节点 (Path Planner) —— 全部坐标解算 + 腕部翻转决策 + 关节空间路程优化。
//
// 职责：消费 strategy 的 /tetris_plan，把每个任务的「抓取像素 / 放置格子」解算成 base 系
// 抓放位姿（含沿 board 法向的悬停），决定腕部 180° 翻转，并对「抓取-放置」序列做全局优化，
// 输出可直接执行的 MotionPlan 给控制节点（纯执行器）。
//   - 抓取 XY：单应性矩阵；抓取 Z：RealSense 对齐深度（无效回退平面拟合）。
//   - 放置 XY/Z：白板格心双线性插值 + PLACE_Z_MAP。
//   - 悬停：在 table 系抬到 HOVER_Z（table +Z 即 board 法向），再换算 base。
//
// 关节空间代价（替代旧 XY 直线距离）：用 xArm6 闭式 FK + seeded DLS IK（见
// tly/xarm6_kinematics.hpp）把每个 TCP 位姿转成 6 关节角 q，代价
//   Cost = Σ wᵢ (q_B,i − q_A,i)²
// 联合优化：放置顺序（DAG 内重排）× 同形状抓取分配 × 腕部 A/B 180° 翻转，统一最小化。
// 翻转并入该代价：A(不翻)/B(翻180°) 两套各算关节代价取小，J6 软限位作排除约束。
// 起点关节 q 取自 /xarm/joint_states；启动用 fk/ik 对照 base→link_tcp TF 自检，失败则
// 回退到旧的 XY 直线代价（安全网）。
//
// 输出：MotionPlan 发到 motion_topic（默认 /motion_cmds，latched），控制节点消费。
// 另把重排+抓取分配后的 17-int 计划发到 optimized_plan_topic（/tetris_plan_opt）仅作调试对照。

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <XmlRpcValue.h>

#include <tly/MotionPlan.h>
#include <tly/MotionTask.h>
#include <tly/depth_sampler.hpp>
#include <tly/xarm6_kinematics.hpp>
#include <xarm_msgs/RobotMsg.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <utility>
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

    bool flip = false; // 腕部 180° 翻转（由优化决定）

    // 原始 17-int 任务块（用于无损重排/回填抓取字段后再发布 Int32 调试计划）。
    std::vector<int> raw;
};

// /vision/board_state 里的一个检测块（候选抓取目标）。
struct PoolBlock
{
    int u = 0;
    int v = 0;
    int ang = 0;
    int geom_u = 0;
    int geom_v = 0;
    bool has_geom = false;
};

// 一个任务的 table 系四位姿（pick/place + 各自悬停）。
struct TablePoses
{
    geometry_msgs::Pose pick;
    geometry_msgs::Pose pick_hover;
    geometry_msgs::Pose place;
    geometry_msgs::Pose place_hover;
    double pick_yaw = 0.0;
    double place_yaw = 0.0;
};

class PathPlanner
{
public:
    PathPlanner() : pnh_("~"), tf_listener_(tf_buffer_)
    {
        loadParams();

        camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &PathPlanner::cameraInfoCallback, this);
        joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &PathPlanner::jointStateCallback, this);
        // 初始位姿工具朝向改用固件姿态时，订阅 xArm 原生状态话题缓存 base<-eef 朝向。
        if (tool_tilt_from_firmware_)
            robot_state_sub_ = nh_.subscribe(robot_state_topic_, 1, &PathPlanner::robotStateCallback, this);

        ROS_INFO("[PATH] Waiting for CameraInfo on %s ...", camera_info_topic_.c_str());
        sensor_msgs::CameraInfoConstPtr cam_msg =
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_, ros::Duration(5.0));
        if (cam_msg)
            cameraInfoCallback(cam_msg);
        else
            ROS_WARN("[PATH] CameraInfo not received within 5 seconds.");

        if (use_depth_pick_z_)
            depth_sampler_.init(nh_, depth_topic_);

        if (optimize_order_)
            board_state_sub_ = nh_.subscribe(board_state_topic_, 1, &PathPlanner::boardStateCallback, this);

        // 二选一订阅，避免对同一计划重复执行：候选集模式只订阅候选话题，单计划模式只订阅 /tetris_plan。
        if (use_plan_candidates_)
            plans_sub_ = nh_.subscribe(plan_candidates_topic_, 1, &PathPlanner::plansCallback, this);
        else
            plan_sub_ = nh_.subscribe(plan_topic_, 1, &PathPlanner::planCallback, this);
        motion_pub_ = nh_.advertise<tly::MotionPlan>(motion_topic_, 1, true);
        opt_plan_pub_ = nh_.advertise<std_msgs::Int32MultiArray>(optimized_plan_topic_, 1, true);

        ROS_INFO("[PATH] ready: in=%s motion_out=%s opt_out=%s optimize=%s joint_cost=%s pool=%s depth_z=%s",
                 use_plan_candidates_ ? plan_candidates_topic_.c_str() : plan_topic_.c_str(),
                 motion_topic_.c_str(), optimized_plan_topic_.c_str(),
                 optimize_order_ ? "on" : "off", use_joint_cost_ ? "on" : "off",
                 board_state_topic_.c_str(), use_depth_pick_z_ ? "on" : "off");
        if (use_plan_candidates_)
            ROS_INFO("[PATH] multi-candidate selection ON (parallel=%s): pick min joint-cost plan.",
                     parallel_candidate_eval_ ? "on" : "off");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber plan_sub_;
    ros::Subscriber plans_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber board_state_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber robot_state_sub_;
    ros::Publisher motion_pub_;
    ros::Publisher opt_plan_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string plan_topic_ = "/tetris_plan";
    std::string plan_candidates_topic_ = "/tetris_plan_candidates";
    std::string motion_topic_ = "/motion_cmds";
    std::string optimized_plan_topic_ = "/tetris_plan_opt";
    std::string board_state_topic_ = "/vision/board_state";

    // 多候选：消费 strategy 的同分候选集，逐个求关节代价后取最小者执行。
    // true → 订阅 plan_candidates_topic_ 走 plansCallback；false → 订阅 plan_topic_ 走 planCallback。
    bool use_plan_candidates_ = false;
    bool parallel_candidate_eval_ = true; // 候选评估用节点内多线程(共享 const 运动学/标定)
    std::string camera_info_topic_ = "/camera/color/camera_info";
    std::string joint_state_topic_ = "/xarm/joint_states";
    std::string base_frame_ = "link_base";
    std::string camera_frame_ = "camera_color_optical_frame";
    std::string eef_frame_ = "link_tcp";

    // 路程优化：开关 + board_state 候选池缓存（按形状分组）。
    bool optimize_order_ = true;
    // 是否允许重排放置顺序。true=在 DAG 内按代价重排；false=保持策略节点给的顺序（比赛可能有
    // 方块相邻等 DAG 之外的约束），仅为每个槽的指定形状选代价最小的物理块 + 腕部翻转。
    bool allow_reorder_ = true;
    bool pool_ready_ = false;
    std::vector<PoolBlock> pool_by_shape_[7];

    // board_frame 在 base 的常量位姿（由 /tetris/BOARD_POSE_BASE 派生），取代旧 table_frame TF。
    bool board_pose_loaded_ = false;
    tf2::Transform board_to_base_;   // board_frame -> base
    tf2::Transform base_to_board_;   // 逆

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

    // 抓放下压的 roll/pitch。默认在每次收到 plan 时取自"初始位姿"(臂此刻所在=单应性标定位姿)的
    // 工具朝向，换算到 board 系后固定；yaw 仍每任务计算。关掉则用下面 fixed_* 常量(π,0)。
    double fixed_roll_rad_ = M_PI;
    double fixed_pitch_rad_ = 0.0;
    bool read_tool_tilt_from_initial_pose_ = true;
    // 初始位姿工具朝向的来源：true=固件上报姿态 /xarm/xarm_states.pose（与 move_line 执行同一运动学系，
    // 消除 URDF↔固件 ~1° 差），缺失/过旧回退 ROS TF；false=一律用 ROS TF(URDF 系)。
    bool tool_tilt_from_firmware_ = true;
    std::string robot_state_topic_ = "/xarm/xarm_states";
    double fw_pose_max_age_s_ = 0.5;

    int max_tasks_per_plan_ = 0; // 0 = 不限制，发布策略给的全部任务

    // 深度采样
    tly::DepthSampler depth_sampler_;
    bool use_depth_pick_z_ = true;
    std::string depth_topic_ = "/camera/aligned_depth_to_color/image_raw";
    int depth_sample_radius_px_ = 4;
    bool depth_sample_in_raw_color_ = true;
    bool pick_xy_source_depth_ = false; // false=homography(默认), true=深度反投影

    // 关节空间运动学 / 代价
    tly::XArm6Kinematics kin_;
    using Joints = tly::XArm6Kinematics::Joints;
    // 代价 = 沿 move_line 直线路径积分的"真实时间"：基准笛卡尔时间 = max(Δd/v_lin, Δθ/ω)（位置线/姿态角
    // 取瓶颈），每段再对关节饱和取 max(Δt_nominal, maxᵢ|Δq|/v_max,i)。单位秒，仅作候选排序代理。
    std::array<double, 6> joint_max_vel_ = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14}; // 各轴关节限速 rad/s（轨迹关节饱和阈值）
    double transit_lin_speed_m_s_ = 0.06;    // 空载转移 TCP 线速度（≈控制器 transit 60mm/s）
    double loaded_lin_speed_m_s_ = 0.045;    // 载料转移 TCP 线速度（≈控制器 loaded_transit 45mm/s）
    // TCP 姿态角速度上限 ω 随 mvvelo 线性变（Move.srv：mvvelo 0~1000 → 角速度 0~3.14 rad/s），实测
    // ω=0.645rad/s@200mm/s → ω/v_lin≈3.14 rad/m≈π 且与档位无关。故按比值逐段自适应：ω=该值×v_lin。
    // 后果：转动主导阈值 Δθ>ω/v_lin·Δd=π·Δd，与速度无关；一个 180° 翻转≈1m 平移的时间代价。
    double omega_per_v_lin_rad_per_m_ = 3.14; // ω/v_lin，即"多少 rad 转动 ≈ 1m 平移"的时间当量
    double wrist_soft_penalty_s_per_rad_ = 1000.0; // 越软限位每 rad 的时间罚（秒尺度，least-bad 仍有序）
    double tie_break_weight_s_per_rad_ = 1e-3;     // 平局次级项：Σ|Δq| 极小权重（不饱和时区分等价候选，偏好腕部少甩）
    // J6 限位：软限位为优化偏好（越界加罚但仍可用）；硬限位为绝对拒发阈值（留余量到 ±2π 真硬限）。
    // 转移段沿直线笛卡尔路径采样 J6（J6≈heading−J1，J1 沿直线非线性摆动，中途可越过两端点值）。
    double wrist_soft_limit_rad_ = 4.7;     // ≈269°，偏好上界
    double wrist_hard_limit_rad_ = 6.10;    // ≈349°，拒发阈值（到 ±2π=360° 留 ~11° 余量）
    int transit_j6_samples_ = 8;            // 每段转移采样点数
    double ik_selfcheck_tol_rad_ = 0.05;    // 自检：max|ik(fk)−q| 阈值
    bool use_joint_cost_ = true;            // 自检通过/IK 可用才为 true
    bool selfcheck_done_ = false;
    std::array<double, 6> nominal_seed_ = {0.0, -0.2, -1.0, 0.0, 1.2, 0.0}; // 工具朝下分支种子

    bool joint_state_ready_ = false;
    std::array<double, 6> q_meas_ = {0, 0, 0, 0, 0, 0};
    std::mutex joint_mutex_;

    // 固件上报的 base<-eef 姿态（/xarm/xarm_states.pose 的 rpy；TCP 旋转偏置为 0，故=法兰/link_tcp 朝向）。
    bool fw_pose_ready_ = false;
    tf2::Quaternion fw_base_eef_q_;
    ros::Time fw_pose_stamp_;
    std::mutex fw_mutex_;

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

    static double unwrapAngle(double current, double target)
    {
        double diff = target - current;
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff < -M_PI)
            diff += 2.0 * M_PI;
        return current + diff;
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
        pnh_.param("plan_candidates_topic", plan_candidates_topic_, plan_candidates_topic_);
        pnh_.param("use_plan_candidates", use_plan_candidates_, use_plan_candidates_);
        pnh_.param("parallel_candidate_eval", parallel_candidate_eval_, parallel_candidate_eval_);
        pnh_.param("motion_topic", motion_topic_, motion_topic_);
        pnh_.param("optimized_plan_topic", optimized_plan_topic_, optimized_plan_topic_);
        pnh_.param("board_state_topic", board_state_topic_, board_state_topic_);
        pnh_.param("joint_state_topic", joint_state_topic_, joint_state_topic_);
        pnh_.param("optimize_order", optimize_order_, optimize_order_);
        pnh_.param("allow_reorder", allow_reorder_, allow_reorder_);
        pnh_.param("camera_info_topic", camera_info_topic_, camera_info_topic_);
        pnh_.param("base_frame", base_frame_, base_frame_);
        pnh_.param("camera_frame", camera_frame_, camera_frame_);
        pnh_.param("eef_frame", eef_frame_, eef_frame_);

        pnh_.param("use_true_pick_plane", use_true_pick_plane_, use_true_pick_plane_);
        pnh_.param("use_pick_homography", use_pick_homography_, use_pick_homography_);
        pnh_.param("use_board_map", use_board_map_, use_board_map_);
        pnh_.param("use_cell_max_place_z", use_cell_max_place_z_, use_cell_max_place_z_);
        pnh_.param("require_board_map", require_board_map_, require_board_map_);
        pnh_.param("require_place_z_map", require_place_z_map_, require_place_z_map_);

        pnh_.param("tcp_pick_offset_x", tcp_pick_offset_x_, tcp_pick_offset_x_);
        pnh_.param("tcp_pick_offset_y", tcp_pick_offset_y_, tcp_pick_offset_y_);
        nh_.getParam("/tetris/TCP_PICK_OFFSET_Z", tcp_pick_offset_z_);
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
        nh_.getParam("/tetris/TCP_PLACE_OFFSET_Z", tcp_place_offset_z_);
        pnh_.param("tcp_place_offset_z", tcp_place_offset_z_, tcp_place_offset_z_);
        pnh_.param("place_release_z_margin", place_release_z_margin_, place_release_z_margin_);

        pnh_.param("read_tool_tilt_from_initial_pose", read_tool_tilt_from_initial_pose_, read_tool_tilt_from_initial_pose_);
        pnh_.param("fixed_roll_rad", fixed_roll_rad_, fixed_roll_rad_);
        pnh_.param("fixed_pitch_rad", fixed_pitch_rad_, fixed_pitch_rad_);
        pnh_.param("tool_tilt_from_firmware", tool_tilt_from_firmware_, tool_tilt_from_firmware_);
        pnh_.param("robot_state_topic", robot_state_topic_, robot_state_topic_);
        pnh_.param("fw_pose_max_age_s", fw_pose_max_age_s_, fw_pose_max_age_s_);

        pnh_.param("max_tasks_per_plan", max_tasks_per_plan_, max_tasks_per_plan_);

        pnh_.param("use_depth_pick_z", use_depth_pick_z_, use_depth_pick_z_);
        pnh_.param("depth_topic", depth_topic_, depth_topic_);
        pnh_.param("depth_sample_radius_px", depth_sample_radius_px_, depth_sample_radius_px_);
        pnh_.param("depth_sample_in_raw_color", depth_sample_in_raw_color_, depth_sample_in_raw_color_);
        std::string xy_source = "homography";
        pnh_.param("pick_xy_source", xy_source, xy_source);
        pick_xy_source_depth_ = (xy_source == "depth");

        // 关节代价参数
        pnh_.param("wrist_soft_limit_rad", wrist_soft_limit_rad_, wrist_soft_limit_rad_);
        pnh_.param("wrist_hard_limit_rad", wrist_hard_limit_rad_, wrist_hard_limit_rad_);
        pnh_.param("transit_j6_samples", transit_j6_samples_, transit_j6_samples_);
        pnh_.param("ik_selfcheck_tol_rad", ik_selfcheck_tol_rad_, ik_selfcheck_tol_rad_);
        loadJointCostParams();

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

        ROS_INFO("[PATH] calib: board_pose=%s board=%s zmap=%s pick_plane=%s pick_homography=%s",
                 board_pose_loaded_ ? "loaded" : "MISSING",
                 board_centers_loaded_ ? "loaded" : "MISSING",
                 place_z_map_loaded_ ? "loaded" : "MISSING",
                 pick_plane_loaded_ ? "loaded" : "flat_PICK_Z",
                 pick_homography_loaded_ ? "loaded" : "off");
    }

    void loadJointCostParams()
    {
        // 关节代价不再是"端点 Δq 的二次和"，而是沿 move_line 直线路径积分的真实时间（见 evalTransit）。
        // 各轴关节限速 v_max：作为轨迹的关节饱和阈值（Δt_k' = max(基准笛卡尔时间, maxᵢ|Δq|/v_max,i)）。
        // xArm6 六轴限速默认 3.14 rad/s；实测不同用 joint_max_vel_rad_s 覆盖。
        XmlRpc::XmlRpcValue vv;
        if (pnh_.getParam("joint_max_vel_rad_s", vv) && vv.getType() == XmlRpc::XmlRpcValue::TypeArray && vv.size() == 6)
            for (int i = 0; i < 6; ++i)
            {
                double x = joint_max_vel_[i];
                if (xmlToDouble(vv[i], x) && x > 1e-6)
                    joint_max_vel_[i] = x;
            }
        pnh_.param("transit_lin_speed_m_s", transit_lin_speed_m_s_, transit_lin_speed_m_s_);
        pnh_.param("loaded_lin_speed_m_s", loaded_lin_speed_m_s_, loaded_lin_speed_m_s_);
        pnh_.param("omega_per_v_lin_rad_per_m", omega_per_v_lin_rad_per_m_, omega_per_v_lin_rad_per_m_);
        pnh_.param("wrist_soft_penalty_s_per_rad", wrist_soft_penalty_s_per_rad_, wrist_soft_penalty_s_per_rad_);
        pnh_.param("tie_break_weight_s_per_rad", tie_break_weight_s_per_rad_, tie_break_weight_s_per_rad_);

        ROS_INFO("[PATH] joint cost=path-time: v_max=[%.2f..] v_lin(empty/loaded)=%.3f/%.3f m/s omega/v_lin=%.2f rad/m "
                 "wrist_soft=%.0f deg",
                 joint_max_vel_[0], transit_lin_speed_m_s_, loaded_lin_speed_m_s_, omega_per_v_lin_rad_per_m_,
                 wrist_soft_limit_rad_ * 180.0 / M_PI);
    }

    bool loadBoardPose()
    {
        XmlRpc::XmlRpcValue bp;
        if (!nh_.getParam("/tetris/BOARD_POSE_BASE", bp) ||
            bp.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !bp.hasMember("origin") || !bp.hasMember("rpy"))
        {
            ROS_ERROR("[PATH] /tetris/BOARD_POSE_BASE not found; run calibrate_board first.");
            return false;
        }
        tf2::Vector3 origin, rpy;
        if (!readVector3List(bp["origin"], origin) || !readVector3List(bp["rpy"], rpy))
        {
            ROS_ERROR("[PATH] Invalid BOARD_POSE_BASE.");
            return false;
        }
        tf2::Quaternion q;
        q.setRPY(rpy.x(), rpy.y(), rpy.z());
        board_to_base_.setRotation(q);
        board_to_base_.setOrigin(origin);
        base_to_board_ = board_to_base_.inverse();
        ROS_INFO("[PATH] BOARD_POSE_BASE loaded: origin=(%.4f,%.4f,%.4f) rpy=(%.4f,%.4f,%.4f)",
                 origin.x(), origin.y(), origin.z(), rpy.x(), rpy.y(), rpy.z());
        return true;
    }

    bool loadBoardCenters14x10()
    {
        if (!use_board_map_)
            return false;
        XmlRpc::XmlRpcValue centers;
        if (!nh_.getParam("/tetris/BOARD_CENTERS_14x10_BOARD", centers) ||
            centers.getType() != XmlRpc::XmlRpcValue::TypeArray || centers.size() <= 0)
        {
            ROS_WARN("[PATH] /tetris/BOARD_CENTERS_14x10_BOARD not found.");
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

    // 缓存当前关节角（按名字 joint1..joint6 取，避免 gripper 等干扰顺序）。
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        std::array<double, 6> q = {0, 0, 0, 0, 0, 0};
        int found = 0;
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
        {
            const std::string &nm = msg->name[i];
            if (nm.size() == 6 && nm.compare(0, 5, "joint") == 0)
            {
                int idx = nm[5] - '1';
                if (idx >= 0 && idx < 6)
                {
                    q[idx] = msg->position[i];
                    ++found;
                }
            }
        }
        if (found < 6 && msg->position.size() >= 6)
        {
            for (int i = 0; i < 6; ++i)
                q[i] = msg->position[i];
            found = 6;
        }
        if (found >= 6)
        {
            std::lock_guard<std::mutex> lk(joint_mutex_);
            q_meas_ = q;
            joint_state_ready_ = true;
        }
    }

    // 启动自检：用测量关节做 fk → 与 base→link_tcp TF 比对，再 ik 回去比关节误差。
    // 通过则信任关节代价；失败回退 XY 代价。需要 joint_states + TF 同时可用。
    void runSelfCheckIfPossible()
    {
        if (selfcheck_done_)
            return;
        std::array<double, 6> q;
        {
            std::lock_guard<std::mutex> lk(joint_mutex_);
            if (!joint_state_ready_)
                return;
            q = q_meas_;
        }
        geometry_msgs::TransformStamped tf_msg;
        try
        {
            tf_msg = tf_buffer_.lookupTransform(base_frame_, eef_frame_, ros::Time(0), ros::Duration(0.5));
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "[PATH] self-check TF %s->%s unavailable: %s",
                              base_frame_.c_str(), eef_frame_.c_str(), ex.what());
            return;
        }
        selfcheck_done_ = true;

        tf2::Transform tf_pose;
        tf2::fromMsg(tf_msg.transform, tf_pose);

        // FK 误差（位姿层面）
        tf2::Transform fk_pose = kin_.fk(q);
        double dpos = (fk_pose.getOrigin() - tf_pose.getOrigin()).length();

        // IK 往返误差（关节层面）
        Joints q_chk;
        bool ik_ok = kin_.ik(tf_pose, q, q_chk);
        double dq_max = 0.0;
        if (ik_ok)
            for (int i = 0; i < 6; ++i)
                dq_max = std::max(dq_max, std::fabs(q_chk[i] - q[i]));

        if (ik_ok && dpos < 0.01 && dq_max < ik_selfcheck_tol_rad_)
        {
            use_joint_cost_ = true;
            ROS_INFO("[PATH] IK self-check PASS: fk_pos_err=%.4f m, ik_dq_max=%.4f rad. Using joint-space cost.",
                     dpos, dq_max);
        }
        else
        {
            use_joint_cost_ = false;
            ROS_WARN("[PATH] IK self-check FAIL (ik_ok=%d fk_pos_err=%.4f m dq_max=%.4f rad > tol=%.3f). "
                     "Falling back to XY-distance cost.",
                     (int)ik_ok, dpos, dq_max, ik_selfcheck_tol_rad_);
        }
    }

    bool getObservationTransforms()
    {
        if (!board_pose_loaded_)
        {
            ROS_ERROR_THROTTLE(2.0, "[PATH] BOARD_POSE_BASE not loaded; cannot compute observations.");
            return false;
        }
        try
        {
            observation_cam_to_base_ = tf_buffer_.lookupTransform(base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
            tf2::Transform cam_to_base;
            {
                tf2::Quaternion q;
                tf2::fromMsg(observation_cam_to_base_.transform.rotation, q);
                cam_to_base.setRotation(q);
                const auto &tr = observation_cam_to_base_.transform.translation;
                cam_to_base.setOrigin(tf2::Vector3(tr.x, tr.y, tr.z));
            }
            tf2::Transform cam_to_board = base_to_board_ * cam_to_base;
            observation_cam_to_table_.transform = tf2::toMsg(cam_to_board);
            observation_base_to_table_.transform = tf2::toMsg(base_to_board_);
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_ERROR("[PATH] TF lookup failed: %s", ex.what());
            return false;
        }
    }

    // 抓放下压的 roll/pitch 取自"初始位姿"(臂此刻所在 = 单应性标定位姿)的工具朝向，
    // 换算到 board 系后固定；yaw 仍每任务计算。读不到 base<-eef TF 则保留现 fixed_*(默认π,0)。
    // 注意：依赖处理 plan 时臂确在初始/观测位姿；生产单发流程满足，手动喂 plan 的测试需关闭本项。
    // 缓存固件上报的 base<-eef 朝向。pose=[x,y,z(mm),roll,pitch,yaw(rad)]；TCP 旋转偏置=0，
    // 故 pose 的 rpy 即法兰/link_tcp 在固件 base 系的朝向（与 move_line 执行同一运动学系）。
    void robotStateCallback(const xarm_msgs::RobotMsg::ConstPtr &msg)
    {
        if (msg->pose.size() < 6)
            return;
        tf2::Quaternion q;
        q.setRPY(msg->pose[3], msg->pose[4], msg->pose[5]);
        std::lock_guard<std::mutex> lk(fw_mutex_);
        fw_base_eef_q_ = q;
        fw_pose_stamp_ = ros::Time::now();
        fw_pose_ready_ = true;
    }

    void updateToolTiltFromInitialPose()
    {
        if (!read_tool_tilt_from_initial_pose_ || !board_pose_loaded_)
            return;

        // 优先用固件上报姿态（/xarm/xarm_states.pose 的 rpy），与 move_line 执行同一运动学系，
        // 消除 URDF↔固件 ~1° 差；缺失/过旧回退 ROS TF(URDF 系)。
        tf2::Quaternion q_be; // base <- eef
        const char *src = nullptr;
        if (tool_tilt_from_firmware_)
        {
            std::lock_guard<std::mutex> lk(fw_mutex_);
            if (fw_pose_ready_ && (ros::Time::now() - fw_pose_stamp_).toSec() <= fw_pose_max_age_s_)
            {
                q_be = fw_base_eef_q_;
                src = "firmware";
            }
        }
        if (src == nullptr)
        {
            try
            {
                geometry_msgs::TransformStamped tf =
                    tf_buffer_.lookupTransform(base_frame_, eef_frame_, ros::Time(0), ros::Duration(0.5));
                tf2::fromMsg(tf.transform.rotation, q_be);
                src = tool_tilt_from_firmware_ ? "tf(firmware stale)" : "tf";
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_WARN("[PATH] read initial-pose tool tilt failed (%s); keep roll=%.1f pitch=%.1f deg.",
                         ex.what(), fixed_roll_rad_ * 180.0 / M_PI, fixed_pitch_rad_ * 180.0 / M_PI);
                return;
            }
        }

        tf2::Quaternion q_board = base_to_board_.getRotation() * q_be; // board <- eef
        double r, p, y;
        tf2::Matrix3x3(q_board).getRPY(r, p, y);
        fixed_roll_rad_ = r;
        fixed_pitch_rad_ = p;
        ROS_INFO("[PATH] tool tilt from initial pose (board, src=%s): roll=%.2f pitch=%.2f deg (yaw per-task).",
                 src, r * 180.0 / M_PI, p * 180.0 / M_PI);
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

    // 抓取点(table)：单应性 XY + 平面 Z + 2.5D 视差补偿（不含深度）。
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

    // 综合抓取点：XY 默认单应性，Z 优先深度（无效回退平面）。
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

    // table 系悬停：抬到 HOVER_Z（table +Z = board 法向），至少高于目标 10mm。
    geometry_msgs::Pose hoverFromTable(const geometry_msgs::Pose &target) const
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

    static tf2::Transform poseToTf(const geometry_msgs::Pose &p)
    {
        tf2::Transform t;
        tf2::Quaternion q;
        tf2::fromMsg(p.orientation, q);
        t.setRotation(q);
        t.setOrigin(tf2::Vector3(p.position.x, p.position.y, p.position.z));
        return t;
    }

    bool tableToBasePose(const geometry_msgs::Pose &table_pose, geometry_msgs::Pose &base_pose) const
    {
        if (!board_pose_loaded_)
        {
            ROS_ERROR("[PATH] BOARD_POSE_BASE not loaded; cannot convert pose.");
            return false;
        }
        tf2::Transform t_base = board_to_base_ * poseToTf(table_pose);
        base_pose.position.x = t_base.getOrigin().x();
        base_pose.position.y = t_base.getOrigin().y();
        base_pose.position.z = t_base.getOrigin().z();
        base_pose.orientation = tf2::toMsg(t_base.getRotation());
        return true;
    }

    double baseYawOfTable(double table_yaw) const
    {
        geometry_msgs::Pose base;
        if (!tableToBasePose(makePoseTable(0, 0, 0, table_yaw), base))
            return normalizeAngleRad(table_yaw);
        double r, p, y;
        tf2::Matrix3x3(poseToTf(base).getRotation()).getRPY(r, p, y);
        return normalizeAngleRad(y);
    }

    // 计算一个任务（给定抓取像素 + 翻转）的 table 系四位姿。useDepth=true 时抓取 Z 用深度。
    bool computeTablePoses(const TaskGoal &goal, const PixelPoint &pick_px, int pick_ang,
                           const PixelPoint &geom_px, bool has_geom, bool flip, bool use_depth,
                           TablePoses &out) const
    {
        double pick_yaw_base = normalizeAngleRad(
            yawFromHomography(pick_px.u, pick_px.v, pick_ang * M_PI / 180.0) + pick_yaw_offset_rad_);
        double place_yaw_base = normalizeAngleRad(-goal.way * M_PI / 2.0);
        double pick_yaw = flip ? normalizeAngleRad(pick_yaw_base + M_PI) : pick_yaw_base;
        double place_yaw = flip ? normalizeAngleRad(place_yaw_base + M_PI) : place_yaw_base;

        geometry_msgs::Point pick_p;
        if (use_depth)
        {
            double zp, zd;
            if (!computePickTable(pick_px, pick_p, zp, zd))
                return false;
        }
        else if (!pixelToPickPointTable(pick_px, pick_p))
            return false;

        double pick_x = pick_p.x + std::cos(pick_yaw) * tcp_pick_offset_x_ - std::sin(pick_yaw) * tcp_pick_offset_y_;
        double pick_y = pick_p.y + std::sin(pick_yaw) * tcp_pick_offset_x_ + std::cos(pick_yaw) * tcp_pick_offset_y_;
        double pick_z = pick_p.z + tcp_pick_offset_z_;
        out.pick = makePoseTable(pick_x, pick_y, pick_z, pick_yaw);

        double pmgx = 0.0, pmgy = 0.0;
        bool has_off = has_geom && computePickToGeomOffset(pick_px, geom_px, pmgx, pmgy);

        geometry_msgs::Point bc = bilinearBoardCenter(goal.place_grid_center.row, goal.place_grid_center.col);
        double place_x = bc.x, place_y = bc.y;
        if (has_off)
        {
            double delta = place_yaw - pick_yaw; // 不受 180° 翻转影响
            double dx_rot = pmgx * std::cos(delta) - pmgy * std::sin(delta);
            double dy_rot = pmgx * std::sin(delta) + pmgy * std::cos(delta);
            place_x += dx_rot;
            place_y += dy_rot;
        }
        place_x += std::cos(place_yaw) * tcp_place_offset_x_ - std::sin(place_yaw) * tcp_place_offset_y_;
        place_y += std::sin(place_yaw) * tcp_place_offset_x_ + std::cos(place_yaw) * tcp_place_offset_y_;
        double place_z = placeZFromTargetCells(goal) + place_release_z_margin_ + tcp_place_offset_z_;
        out.place = makePoseTable(place_x, place_y, place_z, place_yaw);

        out.pick_hover = hoverFromTable(out.pick);
        out.place_hover = hoverFromTable(out.place);
        out.pick_yaw = pick_yaw;
        out.place_yaw = place_yaw;
        return true;
    }

    geometry_msgs::Pose fkPose(const Joints &q) const
    {
        tf2::Transform t = kin_.fk(q);
        geometry_msgs::Pose p;
        p.position.x = t.getOrigin().x();
        p.position.y = t.getOrigin().y();
        p.position.z = t.getOrigin().z();
        p.orientation = tf2::toMsg(t.getRotation());
        return p;
    }

    // base 系两位姿的直线插值：位置线性、姿态 slerp。
    static geometry_msgs::Pose interpPoseBase(const geometry_msgs::Pose &A, const geometry_msgs::Pose &B, double t)
    {
        geometry_msgs::Pose P;
        P.position.x = A.position.x + t * (B.position.x - A.position.x);
        P.position.y = A.position.y + t * (B.position.y - A.position.y);
        P.position.z = A.position.z + t * (B.position.z - A.position.z);
        tf2::Quaternion qa, qb;
        tf2::fromMsg(A.orientation, qa);
        tf2::fromMsg(B.orientation, qb);
        tf2::Quaternion q = qa.slerp(qb, t);
        q.normalize();
        P.orientation = tf2::toMsg(q);
        return P;
    }

    struct TransitEval
    {
        bool ok = false;
        double time_cost = 0.0;    // Σ Δt_k'（秒，排序代理，非绝对耗时）
        double joint_travel = 0.0; // Σ|Δq|（平局次级项）
        double max_abs_j6 = 0.0;   // 路径上 max|J6|（限位校验用）
        Joints q_end{};            // 终点关节（作下段转移起点 seed）
    };

    // 沿 A->B 直线笛卡尔路径（move_line 的几何）逐点 IK（连续 seed，复刻固件就近解），积分"真实时间"代价。
    // 每段基准笛卡尔时间 = max(Δd/v_lin, Δθ/ω)（位置线速度/姿态角速度取瓶颈——翻转造成的姿态差异经此
    // 计入），实际时间再对关节饱和取 max：Δt_k' = max(Δt_nominal, maxᵢ|Δq_{k,i}|/v_max,i)。这既抓端点式
    // 代价看不见的中途关节速度尖峰（J6≈heading−J1，直线平移里 J1 非线性摆动、过基座尤甚，中途 J6 可越两
    // 端点值撞 ±2π），也让姿态扫掠正确进入代价。任一采样 IK 失败 → ok=false。
    TransitEval evalTransit(const geometry_msgs::Pose &A, const geometry_msgs::Pose &B,
                            const Joints &seed, double v_lin) const
    {
        TransitEval e;
        e.max_abs_j6 = std::fabs(seed[5]);
        const int N = std::max(1, transit_j6_samples_);

        // 直线均匀采样：每段位置/姿态增量相同，基准时间逐段相等。
        const double dx = B.position.x - A.position.x;
        const double dy = B.position.y - A.position.y;
        const double dz = B.position.z - A.position.z;
        const double total_d = std::sqrt(dx * dx + dy * dy + dz * dz);
        tf2::Quaternion qa, qb;
        tf2::fromMsg(A.orientation, qa);
        tf2::fromMsg(B.orientation, qb);
        double dotq = std::min(1.0, std::max(-1.0, std::fabs(qa.dot(qb))));
        const double total_theta = 2.0 * std::acos(dotq); // 两姿态夹角
        const double v = std::max(v_lin, 1e-6);
        const double w = std::max(omega_per_v_lin_rad_per_m_ * v, 1e-6); // ω 随该段线速度自适应（实测比值恒定）
        const double dt_nominal = std::max(total_d / N / v, total_theta / N / w);

        Joints q = seed;
        for (int i = 1; i <= N; ++i)
        {
            const double t = static_cast<double>(i) / N;
            geometry_msgs::Pose P = interpPoseBase(A, B, t);
            Joints qi;
            if (!kin_.ik(poseToTf(P), q, qi))
                return e; // ok 保持 false
            double joint_time = 0.0;
            for (int j = 0; j < 6; ++j)
            {
                const double dq = std::fabs(qi[j] - q[j]);
                e.joint_travel += dq;
                joint_time = std::max(joint_time, dq / std::max(joint_max_vel_[j], 1e-6));
            }
            e.time_cost += std::max(dt_nominal, joint_time); // Δt_k'
            q = qi;
            e.max_abs_j6 = std::max(e.max_abs_j6, std::fabs(q[5]));
        }
        e.q_end = q;
        e.ok = true;
        return e;
    }

    // 评估一对 (放置槽, 抓取候选) 的两套翻转。可行性 = 端点 + 两段大转移（cur->pick_hover、
    // pick_hover->place_hover）整条路径的 max|J6| ≤ 硬限位；越软限位加罚（仍可用，least-bad）；
    // 越硬限位直接排除。返回最小可行代价、翻转、终点关节(place_hover)与终点位姿(供下段转移起点)。
    struct PairEval
    {
        bool ok = false;
        double cost = std::numeric_limits<double>::max();
        bool flip = false;
        Joints q_place{};
        geometry_msgs::Pose end_pose;
        double max_j6 = 0.0;
    };
    PairEval evalPairJoint(const TaskGoal &goal, const PixelPoint &pick_px, int pick_ang,
                           const PixelPoint &geom_px, bool has_geom,
                           const Joints &q_cur, const geometry_msgs::Pose &cur_pose) const
    {
        PairEval best;
        for (int f = 0; f < 2; ++f)
        {
            bool flip = (f == 1);
            TablePoses tp;
            if (!computeTablePoses(goal, pick_px, pick_ang, geom_px, has_geom, flip, /*use_depth=*/false, tp))
                continue;
            geometry_msgs::Pose pick_base, pick_hover_base, place_base, place_hover_base;
            if (!tableToBasePose(tp.pick, pick_base) || !tableToBasePose(tp.pick_hover, pick_hover_base) ||
                !tableToBasePose(tp.place, place_base) || !tableToBasePose(tp.place_hover, place_hover_base))
                continue;

            // 端点关节（pick/place 下压点，供 J6 限位校验）。
            Joints q_p, q_pl;
            if (!kin_.ik(poseToTf(pick_base), q_cur, q_p))
                continue;
            if (!kin_.ik(poseToTf(place_base), q_p, q_pl))
                continue;

            // 转移段沿直线积分真实时间代价 + J6 路径校验：cur_pose -> pick_hover（接近，空载）
            // -> place_hover（载料转移）。
            TransitEval trA = evalTransit(cur_pose, pick_hover_base, q_cur, transit_lin_speed_m_s_);
            if (!trA.ok)
                continue;
            TransitEval trB = evalTransit(pick_hover_base, place_hover_base, trA.q_end, loaded_lin_speed_m_s_);
            if (!trB.ok)
                continue;
            double path_max_j6 = std::max(std::max(trA.max_abs_j6, trB.max_abs_j6),
                                          std::max(std::fabs(q_p[5]), std::fabs(q_pl[5])));

            if (path_max_j6 > wrist_hard_limit_rad_)
                continue; // 硬限位：直接排除该方案

            // 代价 = 两段 move_line 真实时间（基准笛卡尔时间 × 关节超速延时）+ 平局次级项（不饱和时偏好腕部少甩）。
            double c = trA.time_cost + trB.time_cost +
                       tie_break_weight_s_per_rad_ * (trA.joint_travel + trB.joint_travel);
            if (path_max_j6 > wrist_soft_limit_rad_)
                c += wrist_soft_penalty_s_per_rad_ * (path_max_j6 - wrist_soft_limit_rad_); // 软限位：秒尺度加罚，least-bad
            if (c < best.cost)
            {
                best.ok = true;
                best.cost = c;
                best.flip = flip;
                best.q_place = trB.q_end; // place_hover 的关节作下段转移起点 seed
                best.end_pose = place_hover_base;
                best.max_j6 = path_max_j6;
            }
        }
        return best;
    }

    // 优化用的抓取 XY（board 系）：用于 XY 回退代价。
    bool pickXYForOpt(int u, int v, double &x, double &y) const
    {
        geometry_msgs::Point p;
        if (use_pick_homography_ && pick_homography_loaded_ &&
            applyPickHomographyXY(static_cast<double>(u), static_cast<double>(v), p))
        {
            x = p.x;
            y = p.y;
            return true;
        }
        PixelPoint px;
        px.u = u;
        px.v = v;
        geometry_msgs::Point tp;
        if (pixelToPickPointTable(px, tp))
        {
            x = tp.x;
            y = tp.y;
            return true;
        }
        return false;
    }

    bool getStartBoardXY(double &x, double &y)
    {
        if (!board_pose_loaded_)
            return false;
        try
        {
            geometry_msgs::TransformStamped tf =
                tf_buffer_.lookupTransform(base_frame_, eef_frame_, ros::Time(0), ros::Duration(0.5));
            tf2::Vector3 p_base(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            tf2::Vector3 p_board = base_to_board_ * p_base;
            x = p_board.x();
            y = p_board.y();
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "[PATH] start TCP TF lookup failed: %s", ex.what());
            return false;
        }
    }

    // 缓存最新候选池：board_state 末段是每个检测块 [shape,u,v,ang(,geom_u,geom_v)]。
    void boardStateCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if ((int)msg->data.size() <= 147)
            return;
        int idx = 147;
        int num_blocks = msg->data[idx++];
        int remaining = (int)msg->data.size() - idx;
        bool has_geom = (num_blocks > 0 && remaining >= num_blocks * 6);
        int stride = has_geom ? 6 : 4;

        std::vector<PoolBlock> tmp[7];
        for (int i = 0; i < num_blocks && idx + stride - 1 < (int)msg->data.size(); ++i)
        {
            PoolBlock b;
            int shape = msg->data[idx++];
            b.u = msg->data[idx++];
            b.v = msg->data[idx++];
            b.ang = msg->data[idx++];
            if (has_geom)
            {
                b.geom_u = msg->data[idx++];
                b.geom_v = msg->data[idx++];
                b.has_geom = true;
            }
            else
            {
                b.geom_u = b.u;
                b.geom_v = b.v;
                b.has_geom = false;
            }
            if (shape >= 0 && shape < 7)
                tmp[shape].push_back(b);
        }
        for (int s = 0; s < 7; ++s)
            pool_by_shape_[s] = tmp[s];
        pool_ready_ = true;
    }

    // 联合优化：放置顺序（DAG 内重排）+ 抓取分配（同形状互换）+ 腕部翻转，最小化总代价。
    // 关节代价可用时用关节空间 + 翻转 + 转移段 J6 路径校验；否则回退 XY 直线距离 + 事后 yaw 启发式。
    // 返回 false 表示规划失败（关节模式下某就绪槽无任何 J6 可行方案）——调用方应放弃发布、待重规划。
    bool optimizePlan(std::vector<TaskGoal> &goals, double &total_cost)
    {
        total_cost = 0.0;
        const int n = static_cast<int>(goals.size());
        if (n == 0)
            return true;

        // 放置点（board XY，用于 XY 回退代价）。
        std::vector<double> place_x(n), place_y(n);
        for (int i = 0; i < n; ++i)
        {
            geometry_msgs::Point bc = bilinearBoardCenter(goals[i].place_grid_center.row, goals[i].place_grid_center.col);
            place_x[i] = bc.x;
            place_y[i] = bc.y;
        }

        // 每形状需求数。
        int need[7] = {0};
        for (const auto &g : goals)
            if (g.shape_type >= 0 && g.shape_type < 7)
                need[g.shape_type]++;

        // 候选池（含闲置块），过滤掉无法投影的，再判可行性。
        std::vector<PoolBlock> cand[7];
        std::vector<double> cand_x[7], cand_y[7];
        std::vector<char> cand_used[7];
        bool use_pool = pool_ready_;
        if (use_pool)
        {
            for (int s = 0; s < 7; ++s)
            {
                for (const auto &b : pool_by_shape_[s])
                {
                    double x, y;
                    if (!pickXYForOpt(b.u, b.v, x, y))
                        continue;
                    cand[s].push_back(b);
                    cand_x[s].push_back(x);
                    cand_y[s].push_back(y);
                    cand_used[s].push_back(0);
                }
            }
            for (int s = 0; s < 7; ++s)
                if (static_cast<int>(cand[s].size()) < need[s])
                    use_pool = false;
        }
        if (!use_pool)
            ROS_WARN("[PATH] board_state pool unavailable/insufficient; reorder-only (keep strategy picks).");

        // 关节起点 q_cur：测量关节优先，否则标称工具朝下种子。
        Joints q_cur = nominal_seed_;
        {
            std::lock_guard<std::mutex> lk(joint_mutex_);
            if (joint_state_ready_)
                q_cur = q_meas_;
        }
        const bool joint_mode = use_joint_cost_;

        // 关节代价路点起点位姿（转移段 J6 采样的起点）：初始 = 当前关节的 FK 位姿（≈初始/观测位姿）。
        geometry_msgs::Pose cur_pose = fkPose(q_cur);

        // XY 回退代价用的起点 TCP（board 系）。
        double cur_x = 0.0, cur_y = 0.0;
        bool has_cur = (!joint_mode) && getStartBoardXY(cur_x, cur_y);

        // 不重排模式：保持策略节点给的放置顺序（比赛可能有 DAG 之外的约束，如方块相邻），
        // 仅按当前顺序为每个槽的指定形状选代价最小的物理块 + 腕部翻转。策略给的具体抓取块仅作
        // 占位/回退（候选池可用时由本节点重选）。
        if (!allow_reorder_)
        {
            for (int i = 0; i < n; ++i)
            {
                int s = goals[i].shape_type;
                double best = std::numeric_limits<double>::max();
                int best_cand = -1;
                bool best_flip = false;
                bool feasible = false;
                Joints best_qplace = q_cur;
                geometry_msgs::Pose best_endpose = cur_pose;
                double best_maxj6 = 0.0;

                if (use_pool && s >= 0 && s < 7)
                {
                    for (int k = 0; k < static_cast<int>(cand[s].size()); ++k)
                    {
                        if (cand_used[s][k])
                            continue;
                        const PoolBlock &b = cand[s][k];
                        if (joint_mode)
                        {
                            PixelPoint pk{b.u, b.v}, gm{b.geom_u, b.geom_v};
                            PairEval pe = evalPairJoint(goals[i], pk, b.ang, gm, b.has_geom, q_cur, cur_pose);
                            if (pe.ok && pe.cost < best)
                            {
                                best = pe.cost;
                                best_cand = k;
                                best_flip = pe.flip;
                                best_qplace = pe.q_place;
                                best_endpose = pe.end_pose;
                                best_maxj6 = pe.max_j6;
                                feasible = true;
                            }
                        }
                        else
                        {
                            double d = (has_cur ? std::hypot(cur_x - cand_x[s][k], cur_y - cand_y[s][k]) : 0.0) +
                                       std::hypot(cand_x[s][k] - place_x[i], cand_y[s][k] - place_y[i]);
                            if (d < best)
                            {
                                best = d;
                                best_cand = k;
                            }
                        }
                    }
                }
                else if (joint_mode)
                {
                    // 池不可用：保留策略抓取，仍按关节代价选翻转。
                    PairEval pe = evalPairJoint(goals[i], goals[i].pick_pixel, goals[i].pick_angle_deg,
                                                goals[i].geom_pixel, goals[i].has_geom_pixel, q_cur, cur_pose);
                    if (pe.ok)
                    {
                        best = pe.cost;
                        best_flip = pe.flip;
                        best_qplace = pe.q_place;
                        best_endpose = pe.end_pose;
                        best_maxj6 = pe.max_j6;
                        feasible = true;
                    }
                }

                // 不重排模式下没有重排自由度：该槽若无任何 J6 可行 (块,翻转) → 规划失败。
                if (joint_mode && !feasible)
                {
                    ROS_ERROR("[PATH] PLANNING FAILED: task %d (shape %d) has no J6-feasible pick/flip "
                              "(every transit exceeds hard limit %.0f deg). NOT publishing — re-plan needed.",
                              i, s, wrist_hard_limit_rad_ * 180.0 / M_PI);
                    return false;
                }

                if (use_pool && best_cand >= 0 && s >= 0 && s < 7)
                {
                    const PoolBlock &b = cand[s][best_cand];
                    cand_used[s][best_cand] = 1;
                    goals[i].pick_pixel.u = b.u;
                    goals[i].pick_pixel.v = b.v;
                    goals[i].pick_angle_deg = b.ang;
                    goals[i].geom_pixel.u = b.geom_u;
                    goals[i].geom_pixel.v = b.geom_v;
                    goals[i].has_geom_pixel = b.has_geom;
                }
                if (joint_mode)
                {
                    if (best_maxj6 > wrist_soft_limit_rad_)
                        ROS_WARN("[PATH] task %d J6 path peak %.0f deg exceeds soft %.0f deg (within hard); margin low.",
                                 i, best_maxj6 * 180.0 / M_PI, wrist_soft_limit_rad_ * 180.0 / M_PI);
                    goals[i].flip = best_flip;
                    q_cur = best_qplace;
                    cur_pose = best_endpose;
                }
                if (best < std::numeric_limits<double>::max())
                    total_cost += best;
                cur_x = place_x[i];
                cur_y = place_y[i];
                has_cur = true;
            }
            if (!joint_mode)
                assignFlipsByYaw(goals);
            ROS_INFO("[PATH] kept strategy order; assigned picks+flip: %d task(s), pool=%s, cost=%s.",
                     n, use_pool ? "on" : "off", joint_mode ? "joint" : "xy");
            return true;
        }

        // 放置依赖 DAG：某格 (r,c) 之下 (r+1,c) 是别的块，则下面的块先放。
        std::vector<int> indeg(n, 0);
        std::vector<std::vector<int>> succ(n);
        std::map<std::pair<int, int>, int> cell2goal;
        for (int i = 0; i < n; ++i)
            if (goals[i].has_target_cells)
                for (const auto &c : goals[i].target_cells)
                    cell2goal[{c.row, c.col}] = i;
        for (int i = 0; i < n; ++i)
        {
            if (!goals[i].has_target_cells)
                continue;
            for (const auto &c : goals[i].target_cells)
            {
                auto it = cell2goal.find({c.row + 1, c.col});
                if (it == cell2goal.end() || it->second == i)
                    continue;
                int h = it->second;
                bool dup = false;
                for (int s : succ[h])
                    if (s == i)
                    {
                        dup = true;
                        break;
                    }
                if (!dup)
                {
                    succ[h].push_back(i);
                    indeg[i]++;
                }
            }
        }

        std::vector<char> placed(n, 0), in_frontier(n, 0);
        std::vector<int> frontier;
        for (int i = 0; i < n; ++i)
            if (indeg[i] == 0)
            {
                frontier.push_back(i);
                in_frontier[i] = 1;
            }

        std::vector<int> order;
        order.reserve(n);
        for (int step = 0; step < n; ++step)
        {
            double best = std::numeric_limits<double>::max();
            int best_goal = -1, best_cand = -1;
            bool best_flip = false;
            Joints best_qplace = q_cur;
            geometry_msgs::Pose best_endpose = cur_pose;
            double best_maxj6 = 0.0;

            for (int g : frontier)
            {
                if (placed[g])
                    continue;
                int s = goals[g].shape_type;
                if (use_pool && s >= 0 && s < 7)
                {
                    for (int k = 0; k < static_cast<int>(cand[s].size()); ++k)
                    {
                        if (cand_used[s][k])
                            continue;
                        const PoolBlock &b = cand[s][k];
                        if (joint_mode)
                        {
                            PixelPoint pk{b.u, b.v}, gm{b.geom_u, b.geom_v};
                            PairEval pe = evalPairJoint(goals[g], pk, b.ang, gm, b.has_geom, q_cur, cur_pose);
                            if (pe.ok && pe.cost < best)
                            {
                                best = pe.cost;
                                best_goal = g;
                                best_cand = k;
                                best_flip = pe.flip;
                                best_qplace = pe.q_place;
                                best_endpose = pe.end_pose;
                                best_maxj6 = pe.max_j6;
                            }
                        }
                        else
                        {
                            double d = (has_cur ? std::hypot(cur_x - cand_x[s][k], cur_y - cand_y[s][k]) : 0.0) +
                                       std::hypot(cand_x[s][k] - place_x[g], cand_y[s][k] - place_y[g]);
                            if (d < best)
                            {
                                best = d;
                                best_goal = g;
                                best_cand = k;
                            }
                        }
                    }
                }
                else
                {
                    if (joint_mode)
                    {
                        PairEval pe = evalPairJoint(goals[g], goals[g].pick_pixel, goals[g].pick_angle_deg,
                                                    goals[g].geom_pixel, goals[g].has_geom_pixel, q_cur, cur_pose);
                        if (pe.ok && pe.cost < best)
                        {
                            best = pe.cost;
                            best_goal = g;
                            best_cand = -1;
                            best_flip = pe.flip;
                            best_qplace = pe.q_place;
                            best_endpose = pe.end_pose;
                            best_maxj6 = pe.max_j6;
                        }
                    }
                    else
                    {
                        double px, py;
                        if (!pickXYForOpt(goals[g].pick_pixel.u, goals[g].pick_pixel.v, px, py))
                        {
                            px = place_x[g];
                            py = place_y[g];
                        }
                        double d = (has_cur ? std::hypot(cur_x - px, cur_y - py) : 0.0) +
                                   std::hypot(px - place_x[g], py - place_y[g]);
                        if (d < best)
                        {
                            best = d;
                            best_goal = g;
                            best_cand = -1;
                        }
                    }
                }
            }
            // 关节模式：DAG 就绪槽里没有任何 J6 可行的 (槽,块,翻转) → 重排也救不了 → 规划失败。
            // 这里已经遍历了所有就绪槽×候选块×翻转（即"先试翻转/换块/重排"），仍无解才报错停。
            if (joint_mode && best_goal < 0)
            {
                ROS_ERROR("[PATH] PLANNING FAILED at step %d/%d: no J6-feasible (transit <= %.0f deg) "
                          "pick/flip for any DAG-ready slot. NOT publishing — re-plan needed.",
                          step, n, wrist_hard_limit_rad_ * 180.0 / M_PI);
                return false;
            }
            if (best_goal < 0)
            {
                for (int g : frontier)
                    if (!placed[g])
                    {
                        best_goal = g;
                        break;
                    }
            }
            if (best_goal < 0)
                break;

            int s = goals[best_goal].shape_type;
            if (use_pool && best_cand >= 0 && s >= 0 && s < 7)
            {
                const PoolBlock &b = cand[s][best_cand];
                cand_used[s][best_cand] = 1;
                goals[best_goal].pick_pixel.u = b.u;
                goals[best_goal].pick_pixel.v = b.v;
                goals[best_goal].pick_angle_deg = b.ang;
                goals[best_goal].geom_pixel.u = b.geom_u;
                goals[best_goal].geom_pixel.v = b.geom_v;
                goals[best_goal].has_geom_pixel = b.has_geom;
            }
            if (joint_mode)
            {
                if (best_maxj6 > wrist_soft_limit_rad_)
                    ROS_WARN("[PATH] task(goal=%d) J6 path peak %.0f deg exceeds soft %.0f deg (within hard); margin low.",
                             best_goal, best_maxj6 * 180.0 / M_PI, wrist_soft_limit_rad_ * 180.0 / M_PI);
                goals[best_goal].flip = best_flip;
                q_cur = best_qplace;
                cur_pose = best_endpose;
            }
            if (best < std::numeric_limits<double>::max())
                total_cost += best;
            placed[best_goal] = 1;
            order.push_back(best_goal);
            cur_x = place_x[best_goal];
            cur_y = place_y[best_goal];
            has_cur = true;
            for (int v : succ[best_goal])
                if (--indeg[v] == 0 && !in_frontier[v])
                {
                    frontier.push_back(v);
                    in_frontier[v] = 1;
                }
        }

        if (static_cast<int>(order.size()) != n)
        {
            ROS_WARN("[PATH] optimize produced %lu/%d tasks (cycle?); keeping original order.",
                     order.size(), n);
            return true;
        }

        std::vector<TaskGoal> reordered;
        reordered.reserve(n);
        for (int idx : order)
            reordered.push_back(goals[idx]);
        goals.swap(reordered);

        if (!joint_mode)
            assignFlipsByYaw(goals); // XY 回退路径：事后用 yaw 启发式选翻转

        ROS_INFO("[PATH] optimized order+assign: %d task(s), pool=%s, cost=%s.",
                 n, use_pool ? "on" : "off", joint_mode ? "joint" : "xy");
        return true;
    }

    // XY 回退时的翻转选择：沿用旧 A/B 腕角 travel 启发式（不需 IK，仅 base-yaw）。
    void assignFlipsByYaw(std::vector<TaskGoal> &goals)
    {
        double sim_yaw = 0.0;
        {
            std::lock_guard<std::mutex> lk(joint_mutex_);
            if (joint_state_ready_)
                sim_yaw = q_meas_[5];
        }
        for (auto &g : goals)
        {
            double pick_yaw_base = normalizeAngleRad(
                yawFromHomography(g.pick_pixel.u, g.pick_pixel.v, g.pick_angle_deg * M_PI / 180.0) + pick_yaw_offset_rad_);
            double place_yaw_base = normalizeAngleRad(-g.way * M_PI / 2.0);

            double pick_A = baseYawOfTable(pick_yaw_base);
            double place_A = baseYawOfTable(place_yaw_base);
            double pick_B = baseYawOfTable(normalizeAngleRad(pick_yaw_base + M_PI));
            double place_B = baseYawOfTable(normalizeAngleRad(place_yaw_base + M_PI));

            double pu_A = unwrapAngle(sim_yaw, pick_A);
            double pl_A = unwrapAngle(pu_A, place_A);
            double travel_A = std::fabs(pu_A - sim_yaw) + std::fabs(pl_A - pu_A);
            double max_A = std::max(std::fabs(pu_A), std::fabs(pl_A));

            double pu_B = unwrapAngle(sim_yaw, pick_B);
            double pl_B = unwrapAngle(pu_B, place_B);
            double travel_B = std::fabs(pu_B - sim_yaw) + std::fabs(pl_B - pu_B);
            double max_B = std::max(std::fabs(pu_B), std::fabs(pl_B));

            bool feas_A = max_A <= wrist_soft_limit_rad_;
            bool feas_B = max_B <= wrist_soft_limit_rad_;
            bool choose_B;
            if (feas_A && feas_B)
                choose_B = travel_B < travel_A;
            else if (feas_A)
                choose_B = false;
            else if (feas_B)
                choose_B = true;
            else
                choose_B = max_B < max_A;

            g.flip = choose_B;
            sim_yaw = choose_B ? pl_B : pl_A;
        }
    }

    // 由最终顺序 + 已定翻转，构建可执行的 MotionTask（含深度 Z 与悬停位姿）。
    bool buildMotionTask(const TaskGoal &goal, tly::MotionTask &task)
    {
        TablePoses tp;
        if (!computeTablePoses(goal, goal.pick_pixel, goal.pick_angle_deg, goal.geom_pixel,
                               goal.has_geom_pixel, goal.flip, /*use_depth=*/true, tp))
            return false;

        if (!tableToBasePose(tp.pick, task.pick_pose) ||
            !tableToBasePose(tp.pick_hover, task.pick_hover_pose) ||
            !tableToBasePose(tp.place, task.place_pose) ||
            !tableToBasePose(tp.place_hover, task.place_hover_pose))
            return false;

        task.shape = goal.shape_type;
        task.way = goal.way;

        ROS_INFO("[PATH][TASK] shape=%d flip=%d pick_table=(%.3f,%.3f,%.3f) place_table=(%.3f,%.3f,%.3f) "
                 "yaw_pick=%.1f yaw_place=%.1f",
                 goal.shape_type, (int)goal.flip,
                 tp.pick.position.x, tp.pick.position.y, tp.pick.position.z,
                 tp.place.position.x, tp.place.position.y, tp.place.position.z,
                 tp.pick_yaw * 180.0 / M_PI, tp.place_yaw * 180.0 / M_PI);
        return true;
    }

    // 解析一份 17-int(或兼容长度) 计划为 TaskGoal 列表。纯解析、无成员状态，可并发调用。
    static bool parseGoals(const std::vector<int> &data, std::vector<TaskGoal> &goals,
                           int &total, int &stride)
    {
        goals.clear();
        if (data.empty())
            return false;
        total = data[0];
        if (total <= 0)
            return false;
        stride = (static_cast<int>(data.size()) - 1) / total;
        if (stride < 7)
            return false;
        goals.reserve(total);
        for (int i = 0; i < total; ++i)
        {
            int base = 1 + i * stride;
            if (base + stride > static_cast<int>(data.size()))
                return false;
            TaskGoal goal;
            goal.shape_type = data[base + 0];
            goal.way = data[base + 1];
            goal.place_grid_center.row = data[base + 2] / 4.0;
            goal.place_grid_center.col = data[base + 3] / 4.0;
            goal.pick_pixel.u = data[base + 4];
            goal.pick_pixel.v = data[base + 5];
            goal.pick_angle_deg = data[base + 6];

            if (stride >= 9)
            {
                goal.geom_pixel.u = data[base + 7];
                goal.geom_pixel.v = data[base + 8];
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
                    cell.row = data[cell_start + 2 * k];
                    cell.col = data[cell_start + 2 * k + 1];
                    goal.target_cells.push_back(cell);
                }
            }

            goal.raw.assign(data.begin() + base, data.begin() + base + stride);
            goals.push_back(goal);
        }
        return true;
    }

    // 计划处理前置：就绪检查 + 观测 TF / 工具 tilt / IK 自检。必须在并行评估前完成
    // （它会写入 fixed_roll/pitch_、use_joint_cost_ 等共享成员，之后被各候选只读使用）。
    bool preparePlanContext()
    {
        if (!camera_info_ready_)
        {
            ROS_WARN("[PATH] Drop plan: CameraInfo not ready.");
            return false;
        }
        if (require_board_map_ && !board_centers_loaded_)
        {
            ROS_ERROR("[PATH] Refuse plan: required BOARD_CENTERS_14x10_BOARD missing.");
            return false;
        }
        if (require_place_z_map_ && !place_z_map_loaded_)
        {
            ROS_ERROR("[PATH] Refuse plan: required PLACE_Z_MAP_14x10 missing.");
            return false;
        }
        if (!getObservationTransforms())
            return false;
        updateToolTiltFromInitialPose();
        runSelfCheckIfPossible();
        return true;
    }

    // 由优化后的 goals 构建并发布调试 17-int 计划 + 可执行 MotionPlan。
    void buildAndPublishMotion(std::vector<TaskGoal> &goals, int total, int stride)
    {
        // 1) 调试用 17-int 计划（回填抓取字段后无损重排），不含翻转信息。
        std_msgs::Int32MultiArray opt;
        opt.data.reserve(1 + total * stride);
        opt.data.push_back(total);
        for (const auto &g : goals)
        {
            std::vector<int> blk = g.raw;
            if (blk.size() >= 7)
            {
                blk[4] = g.pick_pixel.u;
                blk[5] = g.pick_pixel.v;
                blk[6] = g.pick_angle_deg;
            }
            if (blk.size() >= 9)
            {
                blk[7] = g.geom_pixel.u;
                blk[8] = g.geom_pixel.v;
            }
            opt.data.insert(opt.data.end(), blk.begin(), blk.end());
        }
        opt_plan_pub_.publish(opt);

        // 2) MotionPlan（控制路径，全集；截断交给执行器）。
        int exec_total = static_cast<int>(goals.size());
        if (max_tasks_per_plan_ > 0 && exec_total > max_tasks_per_plan_)
            exec_total = max_tasks_per_plan_;

        tly::MotionPlan plan;
        plan.header.stamp = ros::Time::now();
        plan.header.frame_id = base_frame_;
        for (int i = 0; i < exec_total; ++i)
        {
            tly::MotionTask task;
            if (buildMotionTask(goals[i], task))
                plan.tasks.push_back(task);
            else
                ROS_WARN("[PATH] Skip task %d: failed to build pick/place pose.", i + 1);
        }
        motion_pub_.publish(plan);

        ROS_INFO("[PATH] Published %s with %lu task(s) + %s Int32(%d) [%s cost].",
                 motion_topic_.c_str(), plan.tasks.size(), optimized_plan_topic_.c_str(),
                 total, use_joint_cost_ ? "joint" : "xy");
    }

    void planCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            ROS_WARN("[PATH] Drop plan: empty.");
            return;
        }
        if (!preparePlanContext())
            return;

        std::vector<TaskGoal> goals;
        int total = 0, stride = 0;
        if (!parseGoals(std::vector<int>(msg->data.begin(), msg->data.end()), goals, total, stride))
        {
            ROS_ERROR("[PATH] Invalid plan (parse failed).");
            return;
        }

        if (optimize_order_)
        {
            double cost = 0.0;
            if (!optimizePlan(goals, cost))
            {
                ROS_ERROR("[PATH] Plan ABORTED (J6 infeasible). No MotionPlan published; awaiting re-plan.");
                return; // fail-stop：不发布任何计划，控制器保持空闲，等待重规划
            }
        }
        else
            assignFlipsByYaw(goals); // 不重排也要定翻转（用 yaw 启发式，不跟踪关节序列）

        buildAndPublishMotion(goals, total, stride);
    }

    // 候选集回调：解析 [K, len0, plan0..., len1, plan1...]，对每个候选做坐标解算 + 路程优化，
    // 取「总运动代价」最小的可行候选执行。候选间相互独立(各自局部 goals/池占用)，故可节点内多线程
    // 并行评估——并行期间主 spin 线程阻塞在 future.get() 上，无其它 ROS 回调改写共享成员；运动学
    // ik/fk 为无状态 const，标定/观测均在 preparePlanContext 后只读，安全。
    void plansCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.empty())
        {
            ROS_WARN("[PATH] Drop candidate set: empty.");
            return;
        }
        if (!preparePlanContext())
            return;

        const std::vector<int> &d = msg->data;
        int idx = 0;
        int K = d[idx++];
        if (K <= 0)
        {
            ROS_WARN("[PATH] Candidate set K=%d invalid.", K);
            return;
        }
        std::vector<std::vector<int>> raw_plans;
        raw_plans.reserve(K);
        for (int k = 0; k < K; ++k)
        {
            if (idx >= static_cast<int>(d.size()))
                break;
            int len = d[idx++];
            if (len <= 0 || idx + len > static_cast<int>(d.size()))
            {
                ROS_ERROR("[PATH] Candidate set framing broken at k=%d (len=%d).", k, len);
                return;
            }
            raw_plans.emplace_back(d.begin() + idx, d.begin() + idx + len);
            idx += len;
        }
        if (raw_plans.empty())
        {
            ROS_WARN("[PATH] Candidate set parsed 0 plans.");
            return;
        }

        struct Eval
        {
            bool ok = false;
            double cost = std::numeric_limits<double>::max();
            std::vector<TaskGoal> goals;
            int total = 0;
            int stride = 0;
        };
        std::vector<Eval> evals(raw_plans.size());

        auto work = [&](int k)
        {
            Eval &e = evals[k];
            std::vector<TaskGoal> goals;
            int total = 0, stride = 0;
            if (!parseGoals(raw_plans[k], goals, total, stride))
                return;
            double cost = 0.0;
            if (optimize_order_)
            {
                if (!optimizePlan(goals, cost))
                    return; // 该候选 J6 不可行，淘汰
            }
            else
            {
                assignFlipsByYaw(goals);
                cost = 0.0; // 无重排/代价：所有候选并列，取第一个可行
            }
            e.ok = true;
            e.cost = cost;
            e.goals = std::move(goals);
            e.total = total;
            e.stride = stride;
        };

        const int M = static_cast<int>(raw_plans.size());
        if (parallel_candidate_eval_ && M > 1)
        {
            std::vector<std::future<void>> futs;
            futs.reserve(M);
            for (int k = 0; k < M; ++k)
                futs.push_back(std::async(std::launch::async, work, k));
            for (auto &f : futs)
                f.get();
        }
        else
        {
            for (int k = 0; k < M; ++k)
                work(k);
        }

        int best = -1;
        double best_cost = std::numeric_limits<double>::max();
        int feasible = 0;
        for (int k = 0; k < M; ++k)
        {
            if (!evals[k].ok)
                continue;
            ++feasible;
            if (evals[k].cost < best_cost)
            {
                best_cost = evals[k].cost;
                best = k;
            }
        }
        if (best < 0)
        {
            ROS_ERROR("[PATH] All %d candidate plan(s) infeasible (J6). No MotionPlan published; awaiting re-plan.", M);
            return;
        }

        ROS_INFO("[PATH] Candidate selection: %d/%d feasible; chose #%d cost=%.4f (%s cost).",
                 feasible, M, best, best_cost, use_joint_cost_ ? "joint" : "xy");
        buildAndPublishMotion(evals[best].goals, evals[best].total, evals[best].stride);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");
    PathPlanner node;
    ros::spin();
    return 0;
}
