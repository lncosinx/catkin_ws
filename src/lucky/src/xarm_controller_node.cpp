// xArm 控制节点 —— 纯执行器 (Pure Executor)。
//
// 职责：消费 path_planner 发布的 /motion_cmds (lucky::MotionPlan)，每个 MotionTask 已是 base 系、
// 可直接 move_line 执行的四位姿（pick/place 下压 + 各自悬停）。本节点不做任何坐标换算、不加载
// 任何标定、不做腕部翻转决策——这些全部由 path_planner 完成。
//
// 仅通过 xArm 原生服务驱动臂（set_mode/set_state/move_line + 吸盘数字 IO），跑逐任务抓放状态机：
//   IDLE → TAKE_NEXT_TASK → MOVE_TO_PICK_HOVER → EXECUTE_PICK → MOVE_TO_PLACE_HOVER
//        → EXECUTE_PLACE → FINISH
// 绝不使用 MoveIt。发布 /robot_status (Bool, latched) 作忙/闲标志。
//
// move_line 的 RPY 相对当前上报位姿解绕 (unwrapAngle) 以保证连续；J6 圈数/翻转由 planner 预先
// 决策，固件按就近解执行（见 CLAUDE.md 腕部踩坑）。

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <xarm_msgs/SetDigitalIO.h>
#include <xarm_msgs/SetInt16.h>
#include <xarm_msgs/Move.h>
#include <xarm_msgs/RobotMsg.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Pose.h>

#include <lucky/MotionPlan.h>
#include <lucky/MotionTask.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

// 一个已解算好的抓放任务：全部为 base 系、可直接 move_line 的位姿。
struct ExecTask
{
    int shape = -1;
    int way = 0;
    geometry_msgs::Pose pick_pose;
    geometry_msgs::Pose pick_hover_pose;
    geometry_msgs::Pose place_pose;
    geometry_msgs::Pose place_hover_pose;
};

class XarmTetrisController
{
public:
    XarmTetrisController()
        : pnh_("~")
    {
        loadParams();

        robot_state_sub_ = nh_.subscribe("/xarm/xarm_states", 10, &XarmTetrisController::robotStateCallback, this);

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
        continue_sub_ = nh_.subscribe(debug_continue_topic_, 1, &XarmTetrisController::continueCallback, this);
        ready_sub_ = nh_.subscribe(ready_topic_, 1, &XarmTetrisController::readyCallback, this);
        status_pub_ = nh_.advertise<std_msgs::Bool>(status_topic_, 1, true);
        publishBusy(false);

        control_timer_ = nh_.createTimer(ros::Duration(control_period_), &XarmTetrisController::controlLoop, this);

        ROS_INFO("[CTRL] PURE EXECUTOR ready: in=%s, hover from plan, max_tasks=%d",
                 plan_topic_.c_str(), max_tasks_per_plan_);
        ROS_INFO("[CTRL] ready gate: %s (topic=%s)",
                 require_ready_ ? "ON — will wait for run=true before executing" : "OFF (execute on plan)",
                 ready_topic_.c_str());
        if (debug_step_)
            ROS_WARN("[CTRL] DEBUG STEP MODE ON: pauses at pick_hover/pick/place_hover/place. "
                     "Continue with: rostopic pub -1 %s std_msgs/Empty \"{}\"",
                     debug_continue_topic_.c_str());
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
        WAIT_FOR_READY,
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
    ros::Subscriber robot_state_sub_;
    ros::Subscriber continue_sub_;
    ros::Subscriber ready_sub_;
    ros::Publisher status_pub_;
    ros::Timer control_timer_;

    ros::ServiceClient io_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient set_state_client_;
    ros::ServiceClient move_line_client_;

    State state_ = State::IDLE;
    std::queue<ExecTask> tasks_;
    ExecTask current_task_;

    std::string plan_topic_ = "/motion_cmds";
    std::string status_topic_ = "/robot_status";
    std::string io_service_ = "/xarm/set_controller_dout";

    double control_period_ = 0.05;
    bool xarm_wait_for_finish_ = true;
    bool verify_native_xyz_after_motion_ = true;
    double native_xyz_tolerance_mm_ = 6.0;
    double native_verify_timeout_s_ = 1.0;
    double native_verify_poll_hz_ = 50.0;

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

    // 放置前可选的预释放抖动（base 系 X/Y 微扰；吸盘仍 ON，回中后再 OFF）。
    bool enable_place_shake_ = false;
    int place_shake_cycles_ = 1;
    double place_shake_dx_m_ = 0.0055;
    double place_shake_dy_m_ = 0.0060;
    double place_shake_speed_mm_s_ = 15.0;
    double place_shake_acc_mm_s2_ = 50.0;

    int max_tasks_per_plan_ = 1;
    bool stop_on_motion_failure_ = true;

    int suction_io_num_ = 1;
    double suction_on_wait_ = 0.25;
    double suction_off_wait_ = 0.20;

    // Debug 单步：到达每个路点(pick_hover/pick/place_hover/place)后暂停，待用户在 continue 话题
    // 发布一条 std_msgs/Empty 才继续。continue_requested_ 由 spinner 线程写、控制线程读，用 atomic。
    bool debug_step_ = false;
    std::string debug_continue_topic_ = "/xarm_controller/continue";
    std::atomic<bool> continue_requested_{false};

    // /ready 执行门闸：所有准备工作就绪后，外部在 /ready 发布 Bool(run=true) 才允许开始执行已排队的
    // 计划。run=false 关闸（仅阻止启动新计划，不打断进行中的动作）。require_ready_=false 可整体旁路，
    // 保持旧的“收到计划即执行”行为（供不发布 /ready 的测试 launch 使用）。run_gate_ 由 spinner 线程
    // 写、控制线程读，用 atomic。
    bool require_ready_ = true;
    std::string ready_topic_ = "/ready";
    std::atomic<bool> run_gate_{false};

    bool robot_state_ready_ = false;
    std::vector<float> native_pose_{0, 0, 0, 3.14159f, 0, 0};
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

    static double unwrapAngle(double current, double target)
    {
        double diff = target - current;
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff < -M_PI)
            diff += 2.0 * M_PI;
        return current + diff;
    }

    void loadParams()
    {
        pnh_.param("plan_topic", plan_topic_, plan_topic_);
        pnh_.param("status_topic", status_topic_, status_topic_);
        pnh_.param("io_service", io_service_, io_service_);

        pnh_.param("control_period", control_period_, control_period_);
        pnh_.param("xarm_wait_for_finish", xarm_wait_for_finish_, xarm_wait_for_finish_);
        pnh_.param("verify_native_xyz_after_motion", verify_native_xyz_after_motion_, verify_native_xyz_after_motion_);
        pnh_.param("native_xyz_tolerance_mm", native_xyz_tolerance_mm_, native_xyz_tolerance_mm_);
        pnh_.param("native_verify_timeout_s", native_verify_timeout_s_, native_verify_timeout_s_);
        pnh_.param("native_verify_poll_hz", native_verify_poll_hz_, native_verify_poll_hz_);

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

        pnh_.param("debug_step", debug_step_, debug_step_);
        pnh_.param("debug_continue_topic", debug_continue_topic_, debug_continue_topic_);
        pnh_.param("require_ready", require_ready_, require_ready_);
        pnh_.param("ready_topic", ready_topic_, ready_topic_);
        pnh_.param("suction_io_num", suction_io_num_, suction_io_num_);
        pnh_.param("suction_on_wait", suction_on_wait_, suction_on_wait_);
        pnh_.param("suction_off_wait", suction_off_wait_, suction_off_wait_);
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

    void continueCallback(const std_msgs::Empty::ConstPtr &)
    {
        continue_requested_ = true;
    }

    // /ready 执行门闸回调：外部准备就绪后发布 run=true 开闸，run=false 关闸。仅置位原子标志，
    // 实际的状态推进由控制线程在 WAIT_FOR_READY 状态轮询完成（不跨线程直接改 state_）。
    void readyCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        const bool prev = run_gate_.exchange(msg->data);
        if (msg->data && !prev)
            ROS_INFO("[CTRL] /ready run=true: execution gate OPEN.");
        else if (!msg->data && prev)
            ROS_WARN("[CTRL] /ready run=false: execution gate CLOSED (won't start new plans; "
                     "in-progress task keeps running).");
    }

    // Debug 单步：到达某路点后暂停，直到用户在 continue 话题发一条消息才返回。
    // 在控制线程（control_timer 回调）里阻塞等待；continue 回调由 spinner 另一线程置位。
    // 返回 false 表示 ros 正在关闭（控制循环应直接退出，不再推进）。
    bool waitForConfirm(const std::string &label)
    {
        if (!debug_step_)
            return true;
        continue_requested_ = false; // 丢弃陈旧确认，要求新的一次发布
        ROS_WARN("[CTRL][DEBUG] PAUSED at '%s'. Continue: rostopic pub -1 %s std_msgs/Empty \"{}\"",
                 label.c_str(), debug_continue_topic_.c_str());
        ros::Rate r(50);
        while (ros::ok() && !continue_requested_.load())
        {
            ROS_INFO_THROTTLE(5.0, "[CTRL][DEBUG] waiting at '%s' for continue on %s ...",
                              label.c_str(), debug_continue_topic_.c_str());
            r.sleep();
        }
        if (!ros::ok())
            return false;
        continue_requested_ = false;
        ROS_INFO("[CTRL][DEBUG] continue from '%s'.", label.c_str());
        return true;
    }

    void planCallback(const lucky::MotionPlan::ConstPtr &msg)
    {
        while (!tasks_.empty())
            tasks_.pop();

        int total = static_cast<int>(msg->tasks.size());
        if (total <= 0)
        {
            ROS_WARN("[CTRL] Received empty MotionPlan.");
            return;
        }

        int exec_total = total;
        if (max_tasks_per_plan_ > 0 && exec_total > max_tasks_per_plan_)
        {
            ROS_WARN("[CTRL][SAFETY] Received %d tasks; executing first %d task(s).", total, max_tasks_per_plan_);
            exec_total = max_tasks_per_plan_;
        }

        for (int i = 0; i < exec_total; ++i)
        {
            const lucky::MotionTask &t = msg->tasks[i];
            ExecTask et;
            et.shape = t.shape;
            et.way = t.way;
            et.pick_pose = t.pick_pose;
            et.pick_hover_pose = t.pick_hover_pose;
            et.place_pose = t.place_pose;
            et.place_hover_pose = t.place_hover_pose;
            tasks_.push(et);
        }

        ROS_INFO("[CTRL] Queued %d task(s) from MotionPlan (%d received).", exec_total, total);
        if (!tasks_.empty())
        {
            publishBusy(true);
            if (!require_ready_ || run_gate_.load())
            {
                state_ = State::TAKE_NEXT_TASK;
            }
            else
            {
                ROS_INFO("[CTRL] Plan queued; holding until /ready publishes run=true on %s.",
                         ready_topic_.c_str());
                state_ = State::WAIT_FOR_READY;
            }
        }
    }

    // base 位姿 -> native [x_mm,y_mm,z_mm, r,p,y]，RPY 相对当前上报解绕后下发。
    bool moveLineNative(const geometry_msgs::Pose &target, double speed_mm_s, double acc_mm_s2, const std::string &label)
    {
        tf2::Quaternion q(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);
        double r = 0.0, p = 0.0, y = 0.0;
        tf2::Matrix3x3(q).getRPY(r, p, y);

        std::vector<float> tgt = {
            static_cast<float>(target.position.x * 1000.0),
            static_cast<float>(target.position.y * 1000.0),
            static_cast<float>(target.position.z * 1000.0),
            static_cast<float>(r),
            static_cast<float>(p),
            static_cast<float>(normalizeAngleRad(y))};

        {
            std::lock_guard<std::mutex> lk(native_pose_mutex_);
            if (robot_state_ready_)
            {
                tgt[3] = static_cast<float>(unwrapAngle(native_pose_[3], tgt[3]));
                tgt[4] = static_cast<float>(unwrapAngle(native_pose_[4], tgt[4]));
                tgt[5] = static_cast<float>(unwrapAngle(native_pose_[5], tgt[5]));
            }
        }

        xarm_msgs::Move srv;
        srv.request.pose = tgt;
        srv.request.mvvelo = static_cast<float>(speed_mm_s);
        srv.request.mvacc = static_cast<float>(acc_mm_s2);
        srv.request.mvtime = 0;
        srv.request.mvradii = 0;

        ROS_INFO("[CTRL][MOVE_LINE] %s target_base=(%.1f, %.1f, %.1f, %.3f, %.3f, %.3f), v=%.1f, a=%.1f",
                 label.c_str(), tgt[0], tgt[1], tgt[2], tgt[3], tgt[4], tgt[5], speed_mm_s, acc_mm_s2);

        if (!move_line_client_.call(srv))
        {
            ROS_ERROR("[CTRL] Failed to call /xarm/move_line for %s", label.c_str());
            return false;
        }
        if (srv.response.ret != 0)
        {
            ROS_ERROR("[CTRL] /xarm/move_line failed for %s: ret=%d msg=%s", label.c_str(),
                      srv.response.ret, srv.response.message.c_str());
            return false;
        }
        return verifyReachedXYZ(tgt, label);
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
            if (stamp > t_enter)
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
            geometry_msgs::Pose p_out = center_pose;
            p_out.position.x += place_shake_dx_m_;
            p_out.position.y += place_shake_dy_m_;
            if (!moveLineNative(p_out, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_diag_out"))
                return false;

            geometry_msgs::Pose p_in = center_pose;
            p_in.position.x -= place_shake_dx_m_;
            p_in.position.y -= place_shake_dy_m_;
            if (!moveLineNative(p_in, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_diag_in"))
                return false;

            if (!moveLineNative(center_pose, place_shake_speed_mm_s_, place_shake_acc_mm_s2_, "shake_center"))
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

        case State::WAIT_FOR_READY:
            if (tasks_.empty())
            {
                publishBusy(false);
                state_ = State::IDLE;
                return;
            }
            if (!require_ready_ || run_gate_.load())
            {
                ROS_INFO("[CTRL] Ready gate open; starting execution of queued plan.");
                state_ = State::TAKE_NEXT_TASK;
            }
            else
            {
                ROS_INFO_THROTTLE(5.0, "[CTRL] Waiting for /ready run=true on %s ...", ready_topic_.c_str());
            }
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
            if (!moveLineNative(current_task_.pick_hover_pose, transit_speed_mm_s_, transit_acc_mm_s2_, "pick_hover"))
            {
                failOrFinish("move_line to pick hover failed.");
                return;
            }
            if (!waitForConfirm("pick_hover"))
                return;
            state_ = State::EXECUTE_PICK;
            return;
        }

        case State::EXECUTE_PICK:
        {
            if (!moveLineNative(current_task_.pick_pose, pick_down_speed_mm_s_, pick_down_acc_mm_s2_, "pick_down"))
            {
                failOrFinish("move_line pick down failed; suction will NOT turn on.");
                return;
            }
            if (!waitForConfirm("pick"))
                return;
            if (!setSuction(true))
            {
                failOrFinish("suction ON failed.");
                return;
            }
            if (!moveLineNative(current_task_.pick_hover_pose, lift_speed_mm_s_, lift_acc_mm_s2_, "pick_lift"))
            {
                failOrFinish("move_line pick lift failed.");
                return;
            }
            state_ = State::MOVE_TO_PLACE_HOVER;
            return;
        }

        case State::MOVE_TO_PLACE_HOVER:
        {
            if (!moveLineNative(current_task_.place_hover_pose, loaded_transit_speed_mm_s_, loaded_transit_acc_mm_s2_, "loaded_place_hover"))
            {
                failOrFinish("move_line loaded transit to place hover failed.");
                return;
            }
            if (!waitForConfirm("place_hover"))
                return;
            state_ = State::EXECUTE_PLACE;
            return;
        }

        case State::EXECUTE_PLACE:
        {
            if (!moveLineNative(current_task_.place_pose, place_down_speed_mm_s_, place_down_acc_mm_s2_, "place_down"))
            {
                failOrFinish("move_line place down failed; suction remains ON.");
                return;
            }
            if (!waitForConfirm("place"))
                return;
            if (!executePlaceShake(current_task_.place_pose))
            {
                failOrFinish("place shake failed; suction remains ON.");
                return;
            }
            if (!setSuction(false))
            {
                failOrFinish("suction OFF failed.");
                return;
            }
            if (!moveLineNative(current_task_.place_hover_pose, lift_speed_mm_s_, lift_acc_mm_s2_, "place_lift"))
            {
                failOrFinish("move_line place lift failed.");
                return;
            }
            state_ = State::TAKE_NEXT_TASK;
            return;
        }

        case State::FINISH:
            ROS_INFO("[CTRL] All tasks finished (pure executor).");
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
