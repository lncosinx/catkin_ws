#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
吸嘴对齐白板/桌面（闭环）。

原理：相机和吸嘴都固连在 link6 上。订阅对齐深度 → RANSAC 拟出桌面法向（相机系）
→ 用手眼 TF 把法向换到 base 系（这是物理桌面法向，与当前姿态无关）→ 让 link6 的
吸嘴轴（默认 link6 +Z，可参数化）反平行于桌面法向，即吸嘴垂直板面 → 改 link6 RPY
（move_line，仅改姿态、保持 TCP 位置）→ 再测，迭代收敛。

对齐的是【吸嘴】不是相机：相机相对吸嘴可以有固定夹角，本脚本通过 TF 链把相机的
测量换算到吸嘴方向，所以即使相机没正对桌面，吸嘴也会被摆正。

精度上限：手眼【旋转】标定精度 + 深度平面拟合噪声。手眼平移误差不影响法向方向，
故对它鲁棒；但手眼旋转的常值误差迭代消不掉（详见脚本末尾说明）。

安全：
  - 默认 **dry-run**：只测量并打印当前吸嘴↔桌面夹角与一步修正量，不动机械臂。
  - `_execute:=true` 才真的发 move_line；每步 max_step_deg 限幅、慢速、保持位置；
    `_auto:=false`（默认）时每步等回车确认。
  - 起步姿态应已大致 flange 朝下（RPY≈180,0,0），让吸嘴轴朝向桌面。

用法：
  # 只测量（不动）：
  rosrun tly align_tool_to_board.py
  # 实际对齐（每步确认）：
  rosrun tly align_tool_to_board.py _execute:=true
  # 自动连跑：
  rosrun tly align_tool_to_board.py _execute:=true _auto:=true
  # 诊断手眼质量（自动转几个姿态，测桌面法向漂移；需 execute 才会动）：
  rosrun tly align_tool_to_board.py _mode:=diag _execute:=true

写回：默认把【板面法向 + 修改后的机械臂原生位姿】合并写入 tetris_config.yaml 的
      tetris.TOOL_BOARD_ALIGN（只更新这一个键，其余原样保留）。calibrate_board.py 的
      第 7 步（波纹管长度）优先用这个法向作投影轴。用 _save_to_config:=false 关闭。

需要：xArm 原生驱动（提供 /xarm/set_mode|set_state|move_line|xarm_states 与 TF）
      + RealSense（align_depth:=true）。**不要**和 tetris 控制器同时跑（抢 move_line）。
"""

import os
import sys
import yaml
import numpy as np
import rospy
import tf2_ros

from sensor_msgs.msg import Image, CameraInfo
from xarm_msgs.srv import SetInt16, Move
from xarm_msgs.msg import RobotMsg
from tf.transformations import (quaternion_matrix, euler_matrix,
                                euler_from_matrix, rotation_matrix)

DEPTH_TOPIC_DEFAULT = "/camera/aligned_depth_to_color/image_raw"
CAMINFO_TOPIC_DEFAULT = "/camera/color/camera_info"
CAMERA_FRAME_DEFAULT = "camera_color_optical_frame"

# 候选欧拉角约定（xArm RPY 通常是固定轴 XYZ = 'sxyz'，运行时自动核对）。
EULER_CANDIDATES = ["sxyz", "rxyz", "szyx", "rzyx", "sxzy", "syxz", "syzx", "szxy"]


# ----------------------------- 深度 / 几何 -----------------------------

def depth_msg_to_meters(msg):
    if msg.encoding in ("16UC1", "mono16"):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth = arr.astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = arr.astype(np.float32).copy()
    else:
        raise RuntimeError("不支持的深度编码: %s" % msg.encoding)
    depth[~np.isfinite(depth)] = 0.0
    return depth


def grab_median_depth(depth_topic, n_frames, timeout=5.0):
    stack, h, w = [], None, None
    for _ in range(max(1, n_frames)):
        try:
            msg = rospy.wait_for_message(depth_topic, Image, timeout=timeout)
        except Exception as e:
            rospy.logwarn("等待深度帧失败: %s", e)
            continue
        d = depth_msg_to_meters(msg)
        if h is None:
            h, w = d.shape
        elif d.shape != (h, w):
            continue
        stack.append(d)
    if not stack:
        raise RuntimeError("未取到任何深度帧，检查 %s 是否在线" % depth_topic)
    arr = np.stack(stack, axis=0)
    arr[arr <= 0.0] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(arr, axis=0)
    med[~np.isfinite(med)] = 0.0
    return med.astype(np.float32)


def deproject_center(depth_m, frac, z_range, intr):
    """取图像中心 frac 比例的矩形 + 深度范围门 → Nx3 相机系点。"""
    fx, fy, cx, cy = intr
    H, W = depth_m.shape
    fw, fh = int(W * frac), int(H * frac)
    u0, v0 = (W - fw) // 2, (H - fh) // 2
    u1, v1 = u0 + fw, v0 + fh
    sub = depth_m[v0:v1, u0:u1]
    us, vs = np.meshgrid(np.arange(u0, u1), np.arange(v0, v1))
    z = sub.reshape(-1).astype(np.float64)
    us = us.reshape(-1).astype(np.float64)
    vs = vs.reshape(-1).astype(np.float64)
    m = (z > max(1e-3, float(z_range[0]))) & (z < float(z_range[1]))
    z, us, vs = z[m], us[m], vs[m]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def ransac_plane(points, thresh=0.004, iters=300, min_inliers_frac=0.3, seed=0):
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < 50:
        raise RuntimeError("中心区域有效深度点太少(%d)" % n)
    rng = np.random.default_rng(seed)
    best_inliers, best_count = None, -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        inliers = np.abs((pts - p0).dot(nrm)) < thresh
        c = int(np.count_nonzero(inliers))
        if c > best_count:
            best_count, best_inliers = c, inliers
    if best_inliers is None or best_count < max(50, int(min_inliers_frac * n)):
        raise RuntimeError("RANSAC 未找到稳定平面(inliers=%d/%d)" % (best_count, n))
    inl = pts[best_inliers]
    centroid = inl.mean(axis=0)
    _, _, vh = np.linalg.svd(inl - centroid, full_matrices=False)
    normal = vh[-1] / np.linalg.norm(vh[-1])
    signed = (inl - centroid).dot(normal)
    rms = float(np.sqrt(np.mean(signed ** 2)))
    return normal, centroid, best_count, n, rms


def rot_angle(R):
    """旋转矩阵的转角（弧度）。"""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


# ----------------------------- TF / 位姿 -----------------------------

def lookup_R_t(tf_buffer, target, source, timeout=3.0):
    tr = tf_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(timeout))
    q = tr.transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    t = np.array([tr.transform.translation.x,
                  tr.transform.translation.y,
                  tr.transform.translation.z], dtype=float)
    return R, t


class Aligner(object):
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "link_base")
        self.tool_frame = rospy.get_param("~tool_frame", "link6")
        self.camera_frame = rospy.get_param("~camera_frame", CAMERA_FRAME_DEFAULT)
        self.tool_axis = np.array(rospy.get_param("~tool_axis", [0.0, 0.0, 1.0]), dtype=float)
        self.tool_axis /= np.linalg.norm(self.tool_axis)

        self.depth_topic = rospy.get_param("~depth_topic", DEPTH_TOPIC_DEFAULT)
        self.caminfo_topic = rospy.get_param("~camera_info_topic", CAMINFO_TOPIC_DEFAULT)
        self.roi_frac = float(rospy.get_param("~roi_center_frac", 0.5))
        self.roi_z = rospy.get_param("~roi_z", [0.10, 1.50])
        self.capture_frames = int(rospy.get_param("~capture_frames", 50))
        self.ransac_thresh = float(rospy.get_param("~ransac_thresh_m", 0.004))
        self.min_inliers_frac = float(rospy.get_param("~min_inliers_frac", 0.3))

        self.mode = str(rospy.get_param("~mode", "align"))   # align | diag
        self.config_path = rospy.get_param(
            "~config_path", "/root/catkin_ws/src/tly/config/tetris_config.yaml")
        self.diag_perturb_deg = float(rospy.get_param("~diag_perturb_deg", 8.0))
        # 把对齐后的板面法向 + 修改后的机械臂姿态写入 config（供 calibrate_board 第7步等使用）。
        self.save_cfg = bool(rospy.get_param("~save_to_config", True))

        self.execute = bool(rospy.get_param("~execute", False))
        self.auto = bool(rospy.get_param("~auto", False))
        self.tol_deg = float(rospy.get_param("~tol_deg", 0.5))
        self.max_step_deg = float(rospy.get_param("~max_step_deg", 6.0))
        self.damping = float(rospy.get_param("~damping", 0.8))
        self.max_iters = int(rospy.get_param("~max_iters", 12))
        self.mvvelo = float(rospy.get_param("~mvvelo", 40.0))
        self.mvacc = float(rospy.get_param("~mvacc", 200.0))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.native_pose = None
        self.robot_err = 0
        rospy.Subscriber("/xarm/xarm_states", RobotMsg, self._state_cb, queue_size=10)

        self.intr = self._get_intrinsics()
        self.euler_axes = None  # 运行时探测

    def _state_cb(self, msg):
        if len(msg.pose) >= 6:
            self.native_pose = list(msg.pose[:6])
        self.robot_err = int(msg.err)

    def _get_intrinsics(self):
        msg = rospy.wait_for_message(self.caminfo_topic, CameraInfo, timeout=5.0)
        return (msg.K[0], msg.K[4], msg.K[2], msg.K[5])  # fx, fy, cx, cy

    # ---------- 测量 ----------
    def measure_normal_base(self):
        depth = grab_median_depth(self.depth_topic, self.capture_frames)
        pts = deproject_center(depth, self.roi_frac, self.roi_z, self.intr)
        n_cam, _, inl, tot, rms = ransac_plane(pts, self.ransac_thresh,
                                               min_inliers_frac=self.min_inliers_frac)
        R_cb, _ = lookup_R_t(self.tf_buffer, self.base_frame, self.camera_frame)
        n_base = R_cb.dot(n_cam)
        n_base /= np.linalg.norm(n_base)
        if n_base[2] < 0:            # 桌面法向朝上（base +Z 大致向上）
            n_base = -n_base
        return n_base, inl, tot, rms

    # ---------- 欧拉约定探测 ----------
    def detect_euler_axes(self):
        if self.native_pose is None:
            raise RuntimeError("还没收到 /xarm/xarm_states，无法读取当前 RPY")
        r, p, y = self.native_pose[3:6]
        R_tool, _ = lookup_R_t(self.tf_buffer, self.base_frame, self.tool_frame)
        best, best_ang = None, 1e9
        for axes in EULER_CANDIDATES:
            R = euler_matrix(r, p, y, axes=axes)[:3, :3]
            ang = rot_angle(R.T.dot(R_tool))
            if ang < best_ang:
                best_ang, best = ang, axes
        return best, np.degrees(best_ang)

    # ---------- 一步对齐 ----------
    def compute_correction(self, n_base):
        """返回 (theta_deg, target_native_pose 或 None)。target 仅在需要动时给出。"""
        R_tool, _ = lookup_R_t(self.tf_buffer, self.base_frame, self.tool_frame)
        a0 = R_tool.dot(self.tool_axis)
        a0 /= np.linalg.norm(a0)
        d = -n_base                                   # 吸嘴应朝下扎入桌面
        theta = np.arccos(np.clip(a0.dot(d), -1.0, 1.0))
        theta_deg = np.degrees(theta)
        if theta_deg <= self.tol_deg:
            return theta_deg, None

        axis = np.cross(a0, d)
        na = np.linalg.norm(axis)
        if na < 1e-9:                                 # 已反平行(几乎对齐)或正好相反
            return theta_deg, None
        axis /= na
        step = min(self.damping * theta, np.radians(self.max_step_deg))
        R_corr = rotation_matrix(step, axis)[:3, :3]

        r, p, y = self.native_pose[3:6]
        R_cur = euler_matrix(r, p, y, axes=self.euler_axes)[:3, :3]
        R_new = R_corr.dot(R_cur)
        rpy_new = euler_from_matrix(R_new, axes=self.euler_axes)
        target = list(self.native_pose[:3]) + [float(rpy_new[0]),
                                               float(rpy_new[1]),
                                               float(rpy_new[2])]
        return theta_deg, target

    # ---------- 运动 ----------
    def enable_motion(self):
        rospy.wait_for_service("/xarm/set_mode", timeout=5.0)
        rospy.wait_for_service("/xarm/set_state", timeout=5.0)
        rospy.set_param("/xarm/wait_for_finish", True)
        rospy.ServiceProxy("/xarm/set_mode", SetInt16)(0)   # 0 = 位置模式
        rospy.ServiceProxy("/xarm/set_state", SetInt16)(0)  # 0 = 就绪/启动
        self.move_srv = rospy.ServiceProxy("/xarm/move_line", Move)
        rospy.wait_for_service("/xarm/move_line", timeout=5.0)

    def move_line(self, target):
        resp = self.move_srv(pose=[float(v) for v in target],
                             mvvelo=self.mvvelo, mvacc=self.mvacc,
                             mvtime=0.0, mvradii=0.0)
        if resp.ret != 0:
            raise RuntimeError("move_line 失败 ret=%d msg=%s" % (resp.ret, resp.message))
        rospy.sleep(0.4)  # 让 wait_for_finish 后状态稳定再测

    # ---------- 诊断：多姿态测法向漂移（手眼旋转质量） ----------
    def load_touch_normal(self):
        """读 config 里的触点法向（不经相机/手眼，作绝对参考）。无则 None。"""
        try:
            with open(self.config_path) as f:
                t = (yaml.safe_load(f) or {}).get("tetris", {})
            if t.get("BOARD_SURFACE_NORMAL_BASE"):
                n = np.array(t["BOARD_SURFACE_NORMAL_BASE"], dtype=float)
            else:
                tsp = t.get("TABLE_SURFACE_PLANE_BASE", {})
                if not (isinstance(tsp, dict) and tsp.get("normal")):
                    return None
                n = np.array(tsp["normal"], dtype=float)
            n = n / np.linalg.norm(n)
            return n if n[2] >= 0 else -n
        except Exception:
            return None

    def run_diag(self):
        if not self.execute:
            rospy.loginfo("diag 需要 _execute:=true 才能自动转到多个姿态测漂移；"
                          "当前仅在本姿态测了一帧（见上）。")
            return
        if self.native_pose is None:
            return
        start = list(self.native_pose)
        d = np.radians(self.diag_perturb_deg)
        plan = [("start", (0, 0, 0)),
                ("roll+", (d, 0, 0)), ("roll-", (-d, 0, 0)),
                ("pitch+", (0, d, 0)), ("pitch-", (0, -d, 0)),
                ("yaw+", (0, 0, d)), ("yaw-", (0, 0, -d))]
        rospy.loginfo("诊断扫掠：以当前姿态为中心 roll/pitch/yaw 各 ±%.1f°（共%d姿态，"
                      "位置不变），每个测一次桌面法向；理想情况下法向应恒定。",
                      self.diag_perturb_deg, len(plan))
        if not self.auto and input("  开始扫掠？[回车=开始 / q=退出]: ").strip().lower() in ("q", "n", "quit"):
            return

        self.enable_motion()
        samples = []
        try:
            for label, dl in plan:
                target = start[:3] + [start[3] + dl[0], start[4] + dl[1], start[5] + dl[2]]
                try:
                    self.move_line(target)
                    rospy.sleep(0.3)
                    n_base, inl, tot, rms = self.measure_normal_base()
                    samples.append(n_base)
                    ang_z = np.degrees(np.arccos(np.clip(abs(n_base[2]), -1, 1)))
                    rospy.loginfo("  [%-6s] n_base=[% .4f % .4f % .4f] 偏base-Z %.2f° "
                                  "内点%d/%d 残差%.2fmm",
                                  label, n_base[0], n_base[1], n_base[2],
                                  ang_z, inl, tot, rms * 1000.0)
                except Exception as e:
                    rospy.logwarn("  [%-6s] 测量失败，跳过：%s", label, e)
        finally:
            try:
                self.move_line(start)   # 回到起始姿态
            except Exception:
                pass

        if len(samples) < 2:
            rospy.logwarn("有效样本不足，无法评估漂移。")
            return
        N = np.array(samples)
        mean = N.sum(0)
        mean /= np.linalg.norm(mean)
        max_dev = max(np.degrees(np.arccos(np.clip(abs(n.dot(mean)), -1, 1))) for n in N)
        max_pair = 0.0
        for i in range(len(N)):
            for j in range(i + 1, len(N)):
                max_pair = max(max_pair, np.degrees(
                    np.arccos(np.clip(abs(N[i].dot(N[j])), -1, 1))))
        rospy.loginfo("—— 漂移评估（%d 个有效姿态）——", len(N))
        rospy.loginfo("  法向相对一致性: 最大偏均值 %.2f°, 最大两两夹角 %.2f° "
                      "（手眼旋转误差指示，越小越好）", max_dev, max_pair)
        touch = self.load_touch_normal()
        if touch is not None:
            m = mean if mean[2] >= 0 else -mean
            abs_err = np.degrees(np.arccos(np.clip(abs(m.dot(touch)), -1, 1)))
            rospy.loginfo("  vs 触点法向 %s: %.2f°（绝对手眼旋转误差估计）",
                          np.array2string(touch, precision=4), abs_err)
        verdict = "良好" if max_pair < 1.5 else ("可疑" if max_pair < 3.0 else "明显偏差(建议重标手眼)")
        rospy.loginfo("  判定: %s", verdict)

    # ---------- 写回 config ----------
    def save_to_config(self, n_base, note=""):
        """把板面法向 + 当前(修改后的)机械臂原生位姿合并写入 tetris_config.yaml。
        只更新 TOOL_BOARD_ALIGN 这一个键，其余键（手眼/单应性/标定数据）原样保留。"""
        if not self.save_cfg:
            return
        n = np.asarray(n_base, dtype=float)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            rospy.logwarn("法向无效，跳过写入 config。")
            return
        n = n / ln
        if n[2] < 0:
            n = -n
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            data = {}
        if not isinstance(data, dict):
            data = {}
        tetris = data.get("tetris")
        if not isinstance(tetris, dict):
            tetris = {}
        pose = [float(v) for v in (self.native_pose or [])][:6]
        tetris["TOOL_BOARD_ALIGN"] = {
            "normal_base": [float(n[0]), float(n[1]), float(n[2])],
            "tool_pose_native": pose,
            "tool_frame": str(self.tool_frame),
            "tool_axis": [float(v) for v in self.tool_axis],
            "executed": bool(self.execute),
            "note": str(note),
        }
        data["tetris"] = tetris
        try:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True,
                               default_flow_style=False, sort_keys=False)
        except Exception as e:
            rospy.logerr("写入 config 失败：%s", e)
            return
        rospy.loginfo("已写入 %s: TOOL_BOARD_ALIGN.normal_base=%s, tool_pose_native=%s (%s)",
                      self.config_path, np.array2string(n, precision=5),
                      ["%.4f" % v for v in pose], note)

    # ---------- 主循环 ----------
    def run(self):
        # 等当前姿态
        t0 = rospy.Time.now()
        while self.native_pose is None and (rospy.Time.now() - t0).to_sec() < 5.0:
            rospy.sleep(0.1)
        if self.native_pose is None:
            rospy.logerr("收不到 /xarm/xarm_states，确认 xArm 驱动在线。")
            return
        if self.robot_err != 0:
            rospy.logwarn("xArm err=%d（可能需要 UFactory Studio 清错/使能）", self.robot_err)

        self.euler_axes, conv_err = self.detect_euler_axes()
        rospy.loginfo("欧拉约定探测: axes=%s（与 TF %s 残差 %.2f°）",
                      self.euler_axes, self.tool_frame, conv_err)
        if conv_err > 3.0:
            rospy.logerr("RPY 约定/工具系核对残差过大(%.2f°)：native_pose 的姿态与 "
                         "base<-%s 不一致（TCP 可能带旋转偏置）。为安全只做 dry-run。",
                         conv_err, self.tool_frame)
            self.execute = False

        n_base, inl, tot, rms = self.measure_normal_base()
        theta_deg, _ = self.compute_correction(n_base)
        rospy.loginfo("桌面法向(base)=[%.4f %.4f %.4f] 内点%d/%d 残差%.2fmm | "
                      "当前吸嘴↔桌面夹角=%.3f°",
                      n_base[0], n_base[1], n_base[2], inl, tot, rms * 1000.0, theta_deg)

        if self.mode == "diag":
            self.run_diag()
            return

        if not self.execute:
            rospy.loginfo("dry-run：不动机械臂。加 _execute:=true 才执行对齐。")
            self.save_to_config(n_base, note="dry-run (arm not modified)")
            return

        self.enable_motion()
        for it in range(1, self.max_iters + 1):
            n_base, inl, tot, rms = self.measure_normal_base()
            theta_deg, target = self.compute_correction(n_base)
            rospy.loginfo("[迭代%d] 夹角=%.3f° 内点%d/%d 残差%.2fmm",
                          it, theta_deg, inl, tot, rms * 1000.0)
            if target is None:
                rospy.loginfo("✅ 已对齐（夹角 %.3f° ≤ %.2f°）。", theta_deg, self.tol_deg)
                self.save_to_config(n_base, note="converged %.3fdeg" % theta_deg)
                return
            rospy.loginfo("  目标 RPY=(%.3f %.3f %.3f)（仅改姿态，位置不变，步长≤%.1f°）",
                          target[3], target[4], target[5], self.max_step_deg)
            if not self.auto:
                ans = input("  发 move_line？[回车=执行 / q=退出]: ").strip().lower()
                if ans in ("q", "quit", "n"):
                    rospy.loginfo("用户中止。")
                    return
            try:
                self.move_line(target)
            except Exception as e:
                rospy.logerr("运动失败：%s", e)
                return
        rospy.logwarn("达到 max_iters=%d 仍未收敛到 %.2f°，检查手眼旋转标定/深度噪声。",
                      self.max_iters, self.tol_deg)
        try:
            n_base, _, _, _ = self.measure_normal_base()
            self.save_to_config(n_base, note="max_iters not converged")
        except Exception as e:
            rospy.logwarn("收尾测量/写入失败：%s", e)


def main():
    rospy.init_node("align_tool_to_board", anonymous=True)
    try:
        Aligner().run()
    except Exception as e:
        rospy.logerr("对齐脚本异常：%s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()


# 关于精度上限（重要）：
# 本脚本用 base<-camera 的手眼 TF 把相机测的法向换到 base 系。若手眼【旋转】有常值
# 误差 δR，测得法向 = δR·N（N 为真法向），吸嘴会对齐到 δR·N 而非 N，残留 δR 的倾斜，
# 迭代消不掉。手眼【平移】误差不影响法向方向，故对它鲁棒。深度拟合噪声靠多帧中值 +
# 几千像素 RANSAC 压低。要再进一步，需要用已知姿态变化观测法向来标定 camera<-link6
# 的旋转（另一个标定，超出本脚本范围）。
