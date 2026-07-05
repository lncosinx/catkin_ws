#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测量 xArm move_line 的 TCP 姿态角速度 ω，并换算成 path_planner 代价用的比值 ω/v_lin（omega_per_v_lin_rad_per_m）。

原理：move_line 命令一个"原地纯转姿态"（位置不变、只改一个 RPY 分量）的运动，用 /xarm/xarm_states 的
state 场（1=RUNNING → 2=SLEEPING）在固件端时间戳（msg.header.stamp）上界定运动起止，得到该角度的运动
耗时 T。对两个（或多个）不同角度重复，用斜率法消掉加减速斜坡与命令/检测的固定延迟：

    ω ≈ (θ2 − θ1) / (T2 − T1)

输出：除绝对 ω 外，重点给 **ω/v_lin 比值 = ω·1000/mvvelo (rad/m)**——这才是 path_planner 代价直接吃的量
（ω 随 mvvelo 线性变，比值与档位无关，见下）。把它填进 tly.launch 的 omega_per_v_lin_rad_per_m 即可。

关键点：
  - move_line 服务默认"立刻返回"（/xarm/wait_for_finish=false），故不能用服务耗时计时，必须轮询状态；
    本脚本用固件上报的 state 边沿 + header.stamp 计时，抗客户端排队抖动。
  - Move.srv 文档：mvvelo 对角度运动按 0~1000 → 0~3.14 rad/s 映射，实测 ω=0.645@200mm/s 印证 ω 随 mvvelo
    线性变、ω/v_lin≈π 恒定。故用比值标定，与档位无关；换个 mvvelo 复测比值应稳定。
  - Δθ 用起止姿态四元数的测地夹角精确计算，与选哪个 RPY 轴无关。
  - **改了 TCP 或关节的加速度/加加速度(jerk)后，加减速斜坡/角速度上限可能变，建议重测本项。**

安全：这是真机脚本。请先在 UF Studio 把 J6（及 J1/J4）摇到中段再跑，避免大角度旋转把腕部推过 ±180°/限位。
默认只改 yaw、角度适中；起始姿态取当前上报位姿。仅 set_mode(0)/set_state(0)，不改其它。
"""

import math
import threading
import rospy
from xarm_msgs.srv import Move, MoveRequest, SetInt16
from xarm_msgs.msg import RobotMsg


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    # ZYX (yaw*pitch*roll)，与 tf2 setRPY 一致
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    z = cr * cp * sy - sr * sp * cy
    yq = cr * sp * cy + sr * cp * sy
    return (x, yq, z, w)


def geodesic_angle(qa, qb):
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


class OmegaMeasurer:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None          # 1=RUNNING, 2=SLEEPING
        self.cmdnum = None
        self.pose = None           # [x_mm,y_mm,z_mm, r,p,y]
        self.stamp = None          # msg.header.stamp（固件端时间）
        self.ready = False

        self.mvvelo = float(rospy.get_param("~mvvelo", 45.0))       # mm/s，测生产档位
        self.mvacc = float(rospy.get_param("~mvacc", 80.0))         # mm/s^2
        self.axis = str(rospy.get_param("~axis", "yaw"))            # roll/pitch/yaw
        angles = rospy.get_param("~angles_deg", [20.0, 60.0, 100.0, 140.0])
        self.angles_deg = [float(a) for a in angles]
        self.repeats = int(rospy.get_param("~repeats", 3))
        self.ori_tol_rad = float(rospy.get_param("~ori_tol_rad", 0.004))
        self.start_timeout = float(rospy.get_param("~start_timeout_s", 3.0))
        self.finish_timeout = float(rospy.get_param("~finish_timeout_s", 40.0))
        self.settle_pause = float(rospy.get_param("~settle_pause_s", 0.8))

        # move_line 默认立刻返回，正是我们要的（靠状态轮询计时）。
        rospy.set_param("/xarm/wait_for_finish", False)

        self.sub = rospy.Subscriber("/xarm/xarm_states", RobotMsg, self.cb, queue_size=50)
        rospy.loginfo("[OMEGA] waiting for /xarm/xarm_states ...")
        while not rospy.is_shutdown() and not self.ready:
            rospy.sleep(0.05)

        rospy.wait_for_service("/xarm/set_mode")
        rospy.wait_for_service("/xarm/set_state")
        rospy.wait_for_service("/xarm/move_line")
        self.set_mode = rospy.ServiceProxy("/xarm/set_mode", SetInt16)
        self.set_state = rospy.ServiceProxy("/xarm/set_state", SetInt16)
        self.move_line = rospy.ServiceProxy("/xarm/move_line", Move)

    def cb(self, msg):
        with self.lock:
            self.state = msg.state
            self.cmdnum = msg.cmdnum
            if len(msg.pose) >= 6:
                self.pose = list(msg.pose[:6])
            self.stamp = msg.header.stamp
            self.ready = True

    def snap(self):
        with self.lock:
            return self.state, self.cmdnum, list(self.pose) if self.pose else None, self.stamp

    def native_mode(self):
        try:
            self.set_mode(0); self.set_state(0)  # rospy 由位置参数构造 SetInt16Request(data=0)
        except rospy.ServiceException as e:
            rospy.logwarn("[OMEGA] set_mode/state failed: %s", e)

    def target_quat(self, base_pose, delta_rad):
        r, p, y = base_pose[3], base_pose[4], base_pose[5]
        if self.axis == "roll":
            r += delta_rad
        elif self.axis == "pitch":
            p += delta_rad
        else:
            y += delta_rad
        return quat_from_rpy(r, p, y), (r, p, y)

    def send(self, rpy_target, xyz_mm):
        req = MoveRequest()
        req.pose = [xyz_mm[0], xyz_mm[1], xyz_mm[2], rpy_target[0], rpy_target[1], rpy_target[2]]
        req.mvvelo = self.mvvelo
        req.mvacc = self.mvacc
        req.mvtime = 0
        req.mvradii = 0
        return self.move_line(req)

    def wait_running(self):
        """等到 state==1(RUNNING)，返回该帧 header.stamp（运动开始）。超时返回 None。"""
        deadline = rospy.Time.now() + rospy.Duration(self.start_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            st, cmd, _, stamp = self.snap()
            if st == 1 or (cmd is not None and cmd > 0):
                return stamp
            rospy.sleep(0.002)
        return None

    def wait_done(self, target_quat):
        """等到 state 回到 2(SLEEPING)、cmdnum==0 且姿态到位，返回该帧 header.stamp（运动结束）。"""
        deadline = rospy.Time.now() + rospy.Duration(self.finish_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            st, cmd, pose, stamp = self.snap()
            if pose is not None and st == 2 and (cmd == 0 or cmd is None):
                q = quat_from_rpy(pose[3], pose[4], pose[5])
                if geodesic_angle(q, target_quat) < self.ori_tol_rad:
                    return stamp
            rospy.sleep(0.002)
        return None

    def move_and_time(self, rpy_target, xyz_mm):
        """发一次 move_line，返回 (T_run 秒, 实际 Δθ 弧度) 或 None。"""
        tq = quat_from_rpy(*rpy_target)
        # 已在目标姿态（如连续回 home）→ 不发指令，避免 wait_running 空等超时。
        _, _, pose_now, _ = self.snap()
        if pose_now is not None and geodesic_angle(quat_from_rpy(*pose_now[3:6]), tq) < self.ori_tol_rad:
            return 0.0
        resp = self.send(rpy_target, xyz_mm)
        if resp.ret != 0:
            rospy.logerr("[OMEGA] move_line ret=%d msg=%s", resp.ret, resp.message)
            return None
        t_start = self.wait_running()
        if t_start is None:
            rospy.logwarn("[OMEGA] never saw RUNNING (motion too short? raise angle) — skip")
            return None
        t_end = self.wait_done(tq)
        if t_end is None:
            rospy.logerr("[OMEGA] finish timeout — aborting")
            return None
        return (t_end - t_start).to_sec()

    def run(self):
        self.native_mode()
        rospy.sleep(0.3)
        _, _, home, _ = self.snap()
        if home is None:
            rospy.logerr("[OMEGA] no pose; abort")
            return
        home_xyz = home[0:3]
        home_rpy = home[3:6]
        home_q = quat_from_rpy(*home_rpy)
        rospy.loginfo("[OMEGA] home rpy(deg)=(%.1f, %.1f, %.1f) axis=%s mvvelo=%.0f mm/s",
                      math.degrees(home_rpy[0]), math.degrees(home_rpy[1]),
                      math.degrees(home_rpy[2]), self.axis, self.mvvelo)

        # 结果：angle_deg -> [T_run,...]
        results = {a: [] for a in self.angles_deg}
        thetas = {}  # angle_deg -> 实测 Δθ(rad)

        for rep in range(self.repeats):
            for a_deg in self.angles_deg:
                if rospy.is_shutdown():
                    break
                a_rad = math.radians(a_deg)
                _, tgt_rpy = self.target_quat(home, a_rad)  # target_quat 返回 (quat, rpy)，这里要 rpy 三元组
                tq = quat_from_rpy(*tgt_rpy)
                dtheta = geodesic_angle(home_q, tq)  # 精确测地夹角（≈|a_rad|）
                thetas[a_deg] = dtheta

                # 1) 回 home（确保每次从同一起点转出）。
                self.move_and_time(home_rpy, home_xyz)
                rospy.sleep(self.settle_pause)
                if rospy.is_shutdown():
                    break

                # 2) 转到目标，计时。
                T = self.move_and_time(tgt_rpy, home_xyz)
                rospy.sleep(self.settle_pause)
                if T is not None:
                    results[a_deg].append(T)
                    rospy.loginfo("[OMEGA] rep %d/%d  angle=%.0f deg (Δθ=%.3f rad)  T_run=%.3f s  ω_naive=%.3f rad/s",
                                  rep + 1, self.repeats, a_deg, dtheta, T, dtheta / T if T > 1e-6 else float("nan"))

                # 3) 回 home 复位。
                self.move_and_time(home_rpy, home_xyz)
                rospy.sleep(self.settle_pause)

        self.report(results, thetas)

    def report(self, results, thetas):
        rospy.loginfo("========== ω 测量汇总 (mvvelo=%.0f mm/s, axis=%s) ==========", self.mvvelo, self.axis)
        pts = []  # (theta_rad, T_mean)
        for a_deg in sorted(results.keys()):
            Ts = results[a_deg]
            if not Ts:
                rospy.logwarn("  angle=%.0f deg: 无有效样本", a_deg)
                continue
            Tm = sum(Ts) / len(Ts)
            th = thetas[a_deg]
            pts.append((th, Tm))
            rospy.loginfo("  angle=%5.0f deg  Δθ=%.3f rad  T_mean=%.3f s (n=%d)  ω_naive=θ/T=%.3f rad/s",
                          a_deg, th, Tm, len(Ts), th / Tm if Tm > 1e-6 else float("nan"))

        if len(pts) < 2:
            rospy.logwarn("[OMEGA] 有效角度<2，无法用斜率法。请增大角度间距/重试。")
            return

        # 斜率法（消掉加减速斜坡 + 固定延迟）：最小二乘 θ = ω·T + b → ω = slope。
        n = len(pts)
        sT = sum(T for _, T in pts)
        sTh = sum(th for th, _ in pts)
        sTT = sum(T * T for _, T in pts)
        sThT = sum(th * T for th, T in pts)
        denom = n * sTT - sT * sT
        if abs(denom) < 1e-9:
            rospy.logwarn("[OMEGA] 斜率退化。")
            return
        omega = (n * sThT - sTh * sT) / denom
        b = (sTh - omega * sT) / n

        # 相邻两点的成对斜率，便于看一致性。
        rospy.loginfo("  --- 斜率法（推荐，已消加减速斜坡/固定延迟）---")
        ps = sorted(pts, key=lambda x: x[0])
        for i in range(1, len(ps)):
            th1, T1 = ps[i - 1]
            th2, T2 = ps[i]
            if abs(T2 - T1) > 1e-6:
                rospy.loginfo("    (%.0f→%.0f deg): ω=(θ2−θ1)/(T2−T1)=%.3f rad/s",
                              math.degrees(th1), math.degrees(th2), (th2 - th1) / (T2 - T1))
        rospy.loginfo("  ★ 最小二乘 ω = %.3f rad/s @ mvvelo=%.0f mm/s  (截距 b=%.3f s ≈ 加减速+延迟开销)",
                      omega, self.mvvelo, b)

        # path_planner 代价用的是"比值" ω/v_lin（rad/m），与速度档位无关（ω 随 mvvelo 线性变）。
        # v_lin = mvvelo/1000 (m/s)，故 ratio = ω / v_lin = ω·1000/mvvelo。
        v_lin = self.mvvelo / 1000.0
        ratio = omega / v_lin if v_lin > 1e-6 else float("nan")
        rospy.loginfo("  ★ ω/v_lin 比值 = %.3f rad/m  (v_lin=%.3f m/s；理论固件映射≈π=3.14)", ratio, v_lin)
        rospy.loginfo("  → 建议把 path_planner 的 omega_per_v_lin_rad_per_m 设为 %.3f", ratio)
        rospy.loginfo("     比值应与 mvvelo 无关；换个 mvvelo 复测若比值稳定即确认。改了 TCP/关节的加速度"
                      "/加加速度(jerk)后，斜坡/上限可能变，建议重测。")


if __name__ == "__main__":
    rospy.init_node("measure_tcp_omega")
    try:
        OmegaMeasurer().run()
    except rospy.ROSInterruptException:
        pass
