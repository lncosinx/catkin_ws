#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
像素→base 预测 与实际 TCP 触点对比诊断（两条预测路径 × 各自同系的触点尺子）。

两条预测路径其实落在不同 base 系，故触点也分两把尺子记录、各比各的：
  - 深度+手眼：像素→深度反投影→相机系→TF(base<-camera) 到 base（URDF/TF 系，感知粗核对）。
  - 单应性+视差：复刻 xarm_controller 的抓取 XY（PICK_HOMOGRAPHY + BOARD_POSE_BASE）。这套标定由
    calibrate_board 用固件位姿记录、move_line 也走固件系，故预测在【固件系】（生产真正用的路径）。

流程：
  1) 在彩色图上手动点选 N 个物理点（默认 9 个）。
  2) 取多帧中值深度反投影 + 单应性两条各算一个 base 预测。
  3) 用户手动移动吸嘴尖(link_tcp)到同一物理点，按回车：同时记录 ROS TF 触点(URDF 系，比深度+手眼)
     与固件 /xarm/xarm_states.pose 触点(固件系，比单应性)。固件位姿不可用则单应性对照回退 TF。
  4) 打印并保存两条误差 + URDF↔固件系统偏差。

它不写 tetris_config.yaml，只把诊断结果保存到 /tmp。
"""

import os
import time
import yaml
import rospy
import tf2_ros
import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_matrix, euler_matrix


DEFAULT_CONFIG_PATH = "/root/catkin_ws/src/lucky/config/tetris_config.yaml"


def depth_msg_to_meters(msg):
    if msg.encoding in ("16UC1", "mono16"):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth = arr.astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    else:
        raise RuntimeError("不支持的深度编码: %s" % msg.encoding)
    depth[~np.isfinite(depth)] = 0.0
    return depth


def grab_median_depth(depth_topic, n_frames, timeout=5.0):
    stack = []
    shape = None
    print("  采集 %d 帧 aligned depth 做逐像素中值..." % n_frames)
    for _ in range(max(1, n_frames)):
        try:
            msg = rospy.wait_for_message(depth_topic, Image, timeout=timeout)
        except Exception as e:
            rospy.logwarn("等待深度帧失败: %s", e)
            continue
        d = depth_msg_to_meters(msg)
        if shape is None:
            shape = d.shape
        elif d.shape != shape:
            continue
        stack.append(d)
    if not stack:
        raise RuntimeError("未取到深度帧: %s" % depth_topic)
    arr = np.stack(stack, axis=0)
    arr[arr <= 0] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(arr, axis=0)
    med[~np.isfinite(med)] = 0.0
    print("  有效深度帧: %d/%d" % (len(stack), n_frames))
    return med.astype(np.float32)


def sample_depth(depth_m, u, v, half, min_valid):
    H, W = depth_m.shape
    ui, vi = int(round(u)), int(round(v))
    if ui < 0 or vi < 0 or ui >= W or vi >= H:
        return None
    u0, u1 = max(0, ui - half), min(W, ui + half + 1)
    v0, v1 = max(0, vi - half), min(H, vi + half + 1)
    win = depth_m[v0:v1, u0:u1].reshape(-1)
    win = win[win > 0]
    if win.size < min_valid:
        return None
    return float(np.median(win))


def camera_info_to_KD(msg):
    K = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(msg.D, dtype=np.float64).reshape(-1) if len(msg.D) else np.zeros(5)
    return K, D


def pixel_depth_to_camera(u, v, z, K, D):
    if cv2 is not None and D.size and float(np.max(np.abs(D))) > 1e-9:
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        ray = cv2.undistortPoints(pts, K, D)[0, 0]
        x, y = float(ray[0]) * z, float(ray[1]) * z
    else:
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
    return np.array([x, y, z], dtype=np.float64)


def lookup_R_t(tf_buffer, target_frame, source_frame, timeout=3.0):
    tr = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))
    q = tr.transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    t = np.array([tr.transform.translation.x,
                  tr.transform.translation.y,
                  tr.transform.translation.z], dtype=np.float64)
    return R, t


def lookup_point(tf_buffer, target_frame, source_frame, timeout=3.0):
    tr = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))
    return np.array([tr.transform.translation.x,
                     tr.transform.translation.y,
                     tr.transform.translation.z], dtype=np.float64)


def load_board_pose(config_path):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            tcfg = (yaml.safe_load(f) or {}).get("tetris", {})
        bp = tcfg.get("BOARD_POSE_BASE")
        if not bp:
            return None, None
        P = np.asarray(bp["origin"], dtype=np.float64)
        r = [float(v) for v in bp["rpy"]]
        R = euler_matrix(r[0], r[1], r[2], axes="sxyz")[:3, :3]
        return R, P
    except Exception:
        return None, None


def load_pick_homography(config_path):
    """读 PICK_HOMOGRAPHY.matrix（9 个 row-major 浮点）-> 3x3，失败返回 None。"""
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            tcfg = (yaml.safe_load(f) or {}).get("tetris", {})
        hg = tcfg.get("PICK_HOMOGRAPHY")
        if not hg:
            return None
        mat = hg.get("matrix")
        if not mat or len(mat) < 9:
            return None
        return np.asarray([float(v) for v in mat[:9]], dtype=np.float64).reshape(3, 3)
    except Exception:
        return None


def load_pick_z(config_path, default=0.0):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            tcfg = (yaml.safe_load(f) or {}).get("tetris", {})
        v = tcfg.get("PICK_Z")
        return float(v) if v is not None else float(default)
    except Exception:
        return float(default)


# ---- 固件上报位姿（xArm 原生 /xarm/xarm_states.pose，单位 mm / rad）----
# 与 calibrate_board 同源：URDF 名义运动学 ≠ 固件出厂逐台标定运动学（实测本机约 4~5mm）。
# PICK_HOMOGRAPHY / BOARD_POSE_BASE 现在都由 calibrate_board 用固件位姿记录标定、move_line 也走固件系，
# 所以「单应性+视差」预测落在固件系，touch 必须同样用固件位姿记录才是同一把尺子（否则凭空多出那 4~5mm
# 偏差→假性不合格）；「深度+手眼」预测走 TF(base<-camera) 落在 URDF 系，则仍用 ROS TF 记录 touch。
# 故本工具两条 touch 都记：固件位姿比单应性、ROS TF 比深度+手眼。固件位姿不可用时单应性对照回退 TF。
_FW = {
    "enabled": False,
    "pose": None,      # 最新 [x,y,z(mm), r,p,y(rad)]
    "offset": None,    # 固件当前 TCP 偏置 [x,y,z(mm), r,p,y(rad)]（RobotMsg.offset）
    "stamp": None,
    "max_age": 0.5,    # s；位姿过旧视为不可用，回退 TF
}


def _fw_state_cb(msg):
    if len(msg.pose) >= 6:
        _FW["pose"] = [float(v) for v in msg.pose[:6]]
        _FW["stamp"] = rospy.Time.now()
    if len(getattr(msg, "offset", [])) >= 3:
        _FW["offset"] = [float(v) for v in msg.offset[:6]]


def get_firmware_touch():
    """返回新鲜的固件 TCP 位置 [x,y,z]（m，link_base 系）；未启用/过旧/缺失返回 None。"""
    if not _FW["enabled"] or _FW["pose"] is None or _FW["stamp"] is None:
        return None
    if (rospy.Time.now() - _FW["stamp"]).to_sec() > _FW["max_age"]:
        return None
    p = _FW["pose"]
    return np.array([p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0], dtype=np.float64)


def setup_firmware_pose(tf_buffer, base_frame, eef_frame):
    """启用「用固件 /xarm/xarm_states.pose 记录单应性对照的触点」。纯被动，不命令机械臂运动、只读不改：
    订阅状态话题→等首帧→读固件 TCP 偏置(RobotMsg.offset)与吸嘴尖(TF link_eef->eef_frame)核对(仅提示)→
    打印固件位姿 vs ROS TF 偏差（即 URDF↔固件运动学差）。~use_firmware_pose:=false 可整体关闭退回 TF。
    任一环节失败不致命：enabled 保持 False，单应性对照自动回退 TF 触点。返回 True 表示已启用固件位姿。"""
    if not bool(rospy.get_param("~use_firmware_pose", True)):
        print("ℹ️ ~use_firmware_pose=false：单应性对照也用 ROS TF 记录触点（URDF 系）。")
        return False
    try:
        from xarm_msgs.msg import RobotMsg
    except Exception as e:
        print("⚠️ 无法导入 xarm_msgs（%s）；单应性对照回退 ROS TF 触点。" % e)
        return False

    rospy.Subscriber("/xarm/xarm_states", RobotMsg, _fw_state_cb)
    try:
        rospy.wait_for_message("/xarm/xarm_states", RobotMsg, timeout=6.0)
    except Exception:
        print("⚠️ 未收到 /xarm/xarm_states；单应性对照回退 ROS TF 触点。确认机器人在线、robot_ip 正确。")
        return False

    # 核对固件 TCP 偏置 = 吸嘴尖(TF link_eef->eef_frame)，一致则 xarm_states.pose 即吸嘴尖。只核对不改。
    exp = None
    try:
        tr = tf_buffer.lookup_transform("link_eef", eef_frame, rospy.Time(0), rospy.Duration(3.0))
        exp = np.array([tr.transform.translation.x, tr.transform.translation.y,
                        tr.transform.translation.z]) * 1000.0
    except Exception as e:
        print("  ⚠️ 读 TF link_eef->%s 失败（%s）；无法核对固件 TCP，请自行确认=吸嘴尖。" % (eef_frame, e))
    cur = _FW.get("offset")
    if exp is not None and cur is not None:
        d = float(np.linalg.norm(np.array(cur[:3]) - exp))
        if d <= 1.0:   # 1mm 容差
            print("  ✅ 固件当前 TCP 偏置=(%.2f,%.2f,%.2f)mm ≈ 吸嘴尖，xarm_states.pose 即吸嘴尖。"
                  % tuple(cur[:3]))
        else:
            print("  ⚠️ 固件 TCP 偏置=(%.2f,%.2f,%.2f)mm 与吸嘴尖(%.2f,%.2f,%.2f)mm 差 %.2fmm！"
                  % (cur[0], cur[1], cur[2], exp[0], exp[1], exp[2], d))
            print("     → 请在 UF Studio 选中吸嘴尖那个 TCP，否则固件位姿测的不是吸嘴尖、单应性对照会偏。")

    _FW["enabled"] = True
    print("✅ 单应性对照 touch 源=固件位姿 /xarm/xarm_states.pose（与标定/move_line 同系）。")
    # 启动自检：同一时刻固件位姿 vs ROS TF，打印偏差（即 verify 偏 4~5mm 的根源量）。
    p = get_firmware_touch()
    if p is not None:
        try:
            tr = tf_buffer.lookup_transform(base_frame, eef_frame, rospy.Time(0), rospy.Duration(2.0))
            tfp = np.array([tr.transform.translation.x, tr.transform.translation.y,
                            tr.transform.translation.z])
            d = (p - tfp) * 1000.0
            print("  🔎 固件 vs TF 偏差(mm): dx=%.2f dy=%.2f dz=%.2f | XY=%.2f mm"
                  % (d[0], d[1], d[2], float(np.hypot(d[0], d[1]))))
            print("     （即 URDF↔固件运动学差；下面单应性对照已用固件触点消掉这项）")
        except Exception:
            pass
    return True


def raw_to_rectified_px(u, v, K, D, P3x3):
    """把点选的 raw 像素去畸变到 rect 像素空间（控制器/单应性用的就是 rect 像素）。
    D≈0 时近似恒等；cv2 不可用时直接返回原值。"""
    if cv2 is None or P3x3 is None:
        return float(u), float(v)
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
    rect = cv2.undistortPoints(pts, K, D, P=P3x3)[0, 0]
    return float(rect[0]), float(rect[1])


def homography_board_xy(H, u_rect, v_rect, cx, cy, cam_z, block_z, use_parallax):
    """复刻 xarm_controller_node 的抓取 XY：先按 (cam_z-block_z)/cam_z 把像素往光心拉
    （视差补偿），再用 PICK_HOMOGRAPHY 映射到 board-XY。返回 [bx, by] 或 None。"""
    uu, vv = float(u_rect), float(v_rect)
    if use_parallax and cam_z is not None and block_z is not None and cam_z > block_z + 0.05:
        ratio = (cam_z - block_z) / cam_z
        uu = cx + (uu - cx) * ratio
        vv = cy + (vv - cy) * ratio
    res = H.dot(np.array([uu, vv, 1.0], dtype=np.float64))
    if abs(res[2]) < 1e-12:
        return None
    return np.array([res[0] / res[2], res[1] / res[2]], dtype=np.float64)


class PixelTouchCheck(object):
    def __init__(self):
        rospy.init_node("pixel_touch_check", anonymous=True)
        if cv2 is None:
            raise RuntimeError("cv2 不可用，无法弹窗点选")

        self.base_frame = rospy.get_param("~base_frame", "link_base")
        self.eef_frame = rospy.get_param("~eef_frame", "link_tcp")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.config_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
        self.num_points = max(1, int(rospy.get_param("~num_points", 9)))
        self.capture_frames = max(1, int(rospy.get_param("~capture_frames", 30)))
        self.sample_win_half = max(0, int(rospy.get_param("~sample_win_half", 4)))
        self.sample_min_valid = max(1, int(rospy.get_param("~sample_min_valid", 8)))
        self.save_path = str(rospy.get_param("~save_path", "")).strip()

        # 对照模式：除了「深度+手眼」那条感知路径，再用控制器真正在用的
        # 「单应性(+视差)」路径预测同一个物理点，二者各自与触点比对，一刀切开
        # 感知误差 vs 单应性/视差误差。
        self.compare_homography = bool(rospy.get_param("~compare_homography", True))
        self.use_parallax = bool(rospy.get_param("~use_parallax", True))

        self.bridge = CvBridge()
        self.latest_image = None
        self.points = []
        self.mouse_x = 0
        self.mouse_y = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fw_enabled = False
        self.board_R, self.board_P = load_board_pose(self.config_path)
        self.pick_H = load_pick_homography(self.config_path) if self.compare_homography else None
        self.pick_z = load_pick_z(self.config_path, 0.0)
        if self.compare_homography:
            if self.pick_H is None:
                rospy.logwarn("compare_homography 开启但未读到 PICK_HOMOGRAPHY，单应性对照将跳过。")
            elif self.board_R is None:
                rospy.logwarn("compare_homography 开启但未读到 BOARD_POSE_BASE，单应性对照将跳过。")
            else:
                rospy.loginfo("单应性对照: ON (use_parallax=%s, PICK_Z=%.4f)",
                              self.use_parallax, self.pick_z)

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("图像转换失败: %s", e)

    def wait_ready(self):
        rospy.loginfo("等待相机图像: %s", self.image_topic)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.latest_image is None:
            rate.sleep()

        rospy.loginfo("等待 TF: %s <- %s", self.base_frame, self.camera_frame)
        if not self.tf_buffer.can_transform(self.base_frame, self.camera_frame,
                                            rospy.Time(0), rospy.Duration(10.0)):
            raise RuntimeError("TF 不可用: %s <- %s" % (self.base_frame, self.camera_frame))
        rospy.loginfo("等待 TF: %s <- %s", self.base_frame, self.eef_frame)
        if not self.tf_buffer.can_transform(self.base_frame, self.eef_frame,
                                            rospy.Time(0), rospy.Duration(10.0)):
            raise RuntimeError("TF 不可用: %s <- %s" % (self.base_frame, self.eef_frame))

    def _disp_to_image(self, x, y):
        # WINDOW_NORMAL 窗口被 WM 缩放时，鼠标回调给的是“显示坐标”，须按实际显示尺寸换算回
        # 全分辨率图像坐标，否则点选像素被缩放带偏（与标定同源的隐患，会让验证假性合格）。
        img = self.latest_image
        if img is None:
            return int(round(x)), int(round(y))
        H_full, W_full = img.shape[:2]
        win = getattr(self, "_sel_win", None)
        if win is not None:
            try:
                _, _, rw, rh = cv2.getWindowImageRect(win)
                if rw > 0 and rh > 0:
                    x = x * W_full / float(rw)
                    y = y * H_full / float(rh)
            except Exception:
                pass
        ix = int(max(0, min(W_full - 1, round(x))))
        iy = int(max(0, min(H_full - 1, round(y))))
        return ix, iy

    def mouse_cb(self, event, x, y, flags, param):
        ix, iy = self._disp_to_image(x, y)
        self.mouse_x, self.mouse_y = ix, iy
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < self.num_points:
                self.points.append({"id": len(self.points) + 1, "u": ix, "v": iy})
                print("  添加 P%d pixel=(%d,%d)" % (len(self.points), ix, iy))
            else:
                print("  已选满 %d 个点，按 Enter 确认。" % self.num_points)
        elif event == cv2.EVENT_RBUTTONDOWN and self.points:
            p = self.points.pop()
            print("  撤销 P%d" % p["id"])

    def draw_zoom(self, display):
        mag_size = 28
        mag_scale = 7
        mx, my = self.mouse_x, self.mouse_y
        x0, y0 = max(0, mx - mag_size), max(0, my - mag_size)
        x1, y1 = min(display.shape[1], mx + mag_size), min(display.shape[0], my + mag_size)
        patch = self.latest_image[y0:y1, x0:x1]
        if patch.size == 0:
            return
        zoomed = cv2.resize(patch, (0, 0), fx=mag_scale, fy=mag_scale, interpolation=cv2.INTER_NEAREST)
        zx, zy = int((mx - x0) * mag_scale), int((my - y0) * mag_scale)
        cv2.drawMarker(zoomed, (zx, zy), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        zh, zw = zoomed.shape[:2]
        ox, oy = display.shape[1] - zw - 10, 10
        if ox < 0 or oy + zh >= display.shape[0]:
            return
        cv2.rectangle(display, (ox - 2, oy - 2), (ox + zw + 2, oy + zh + 2), (0, 0, 0), -1)
        display[oy:oy + zh, ox:ox + zw] = zoomed
        cv2.rectangle(display, (ox, oy), (ox + zw, oy + zh), (0, 255, 0), 2)

    def select_pixels(self):
        print("\n手动点选像素：左键添加，右键撤销，i 键入(u,v)，c 清空，Enter 确认，q 退出。")
        print("建议按固定顺序点 3x3，例如左上→中上→右上→左中→...→右下。\n")
        win = "pixel touch check: select points"
        self._sel_win = win
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, self.mouse_cb)
        while not rospy.is_shutdown():
            display = self.latest_image.copy()
            for p in self.points:
                u, v, pid = p["u"], p["v"], p["id"]
                cv2.drawMarker(display, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.circle(display, (u, v), 9, (0, 0, 255), 2)
                cv2.putText(display, "P%d" % pid, (u + 8, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            self.draw_zoom(display)
            cv2.putText(display, "Selected %d/%d  Left:add Right:undo i:type(u,v) c:clear Enter:ok q:quit" %
                        (len(self.points), self.num_points),
                        (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.imshow(win, display)
            key = cv2.waitKey(30) & 0xFF
            if key in (13, 10) and len(self.points) == self.num_points:
                cv2.destroyWindow(win)
                return True
            if key == ord("c"):
                self.points = []
                print("  已清空点选。")
            if key == ord("i") and len(self.points) < self.num_points:
                # 可选：手动键入像素坐标（如从 realsense-viewer 读数直接输入），绕开点选/窗口缩放。
                # 注意：键入时焦点要切到终端。
                H_full, W_full = self.latest_image.shape[:2]
                try:
                    s = input("  ⌨️  输入像素 u,v（逗号或空格分隔，范围 0..%d, 0..%d；空回车取消）: "
                              % (W_full - 1, H_full - 1)).strip()
                except EOFError:
                    s = ""
                if s:
                    try:
                        parts = s.replace(",", " ").split()
                        u, v = int(round(float(parts[0]))), int(round(float(parts[1])))
                        if 0 <= u < W_full and 0 <= v < H_full:
                            self.points.append({"id": len(self.points) + 1, "u": u, "v": v})
                            self.mouse_x, self.mouse_y = u, v
                            print("  ✅ 添加 P%d pixel=(%d,%d)  [手动输入]" % (len(self.points), u, v))
                        else:
                            print("  ⚠️ 越界，已忽略。")
                    except (ValueError, IndexError):
                        print("  ⚠️ 格式无效，示例：640 360 或 640,360")
            if key == ord("q"):
                cv2.destroyWindow(win)
                return False
        cv2.destroyWindow(win)
        return False

    def compute_vision_points(self):
        info = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=5.0)
        K, D = camera_info_to_KD(info)
        P3 = np.asarray(info.P, dtype=np.float64).reshape(3, 4)[:3, :3] if len(info.P) >= 12 else K
        cx_p, cy_p = float(P3[0, 2]), float(P3[1, 2])
        print("相机内参: %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f Dmax=%.6f" %
              (info.width, info.height, K[0, 0], K[1, 1], K[0, 2], K[1, 2],
               float(np.max(np.abs(D))) if D.size else 0.0))
        depth = grab_median_depth(self.depth_topic, self.capture_frames)
        R_cb, t_cb = lookup_R_t(self.tf_buffer, self.base_frame, self.camera_frame)

        # 相机光心在 board 系的高度，给单应性视差当 cam_z。
        cam_z = None
        if self.board_R is not None:
            cam_z = abs(float(self.board_R.T.dot(t_cb - self.board_P)[2]))
        hom_ok = self.pick_H is not None and self.board_R is not None

        rows = []
        for p in self.points:
            z = sample_depth(depth, p["u"], p["v"], self.sample_win_half, self.sample_min_valid)
            row = dict(p)
            row["depth_m"] = z
            row["valid_depth"] = z is not None
            p_board = None
            if z is not None:
                p_cam = pixel_depth_to_camera(p["u"], p["v"], z, K, D)
                p_base = R_cb.dot(p_cam) + t_cb
                row["camera_xyz_m"] = [float(v) for v in p_cam]
                row["vision_base_m"] = [float(v) for v in p_base]
                if self.board_R is not None:
                    p_board = self.board_R.T.dot(p_base - self.board_P)
                    row["vision_board_m"] = [float(v) for v in p_board]
                print("  P%d pixel=(%d,%d) z=%.4f -> 深度+手眼 base=[%.5f %.5f %.5f]" %
                      (p["id"], p["u"], p["v"], z, p_base[0], p_base[1], p_base[2]))
            else:
                print("  P%d pixel=(%d,%d) 深度无效（深度路径跳过；单应性路径用 PICK_Z 仍可比）。" %
                      (p["id"], p["u"], p["v"]))

            # 单应性(+视差)路径：复刻 xarm_controller_node 的抓取 XY。
            # 这些点点在发光板（board 系 z=0，单应性标定所在平面）上，不是方块顶面，
            # 所以 block_z=0（视差比例=1，无平移），回投也落在 board 平面 z=0。
            if hom_ok:
                block_z = 0.0
                bz = 0.0
                u_rect, v_rect = raw_to_rectified_px(p["u"], p["v"], K, D, P3)
                hxy = homography_board_xy(self.pick_H, u_rect, v_rect, cx_p, cy_p,
                                          cam_z, block_z, self.use_parallax)
                if hxy is not None:
                    hom_base = self.board_R.dot(np.array([hxy[0], hxy[1], bz])) + self.board_P
                    row["valid_homography"] = True
                    row["homography_board_xy_m"] = [float(hxy[0]), float(hxy[1])]
                    row["homography_base_m"] = [float(v) for v in hom_base]
                    print("       单应性 board_xy=[%.5f %.5f] -> base=[%.5f %.5f]" %
                          (hxy[0], hxy[1], hom_base[0], hom_base[1]))
            rows.append(row)
        return rows, K, D

    def collect_touch_points(self, rows):
        print("\n现在按刚才的 P1..P%d 顺序移动机械臂。" % self.num_points)
        print("每次让吸嘴尖(link_tcp)对准同一物理点，按回车：同时记录 TF 触点(比深度+手眼)与固件位姿触点(比单应性)。")
        print("  s 跳过；q 结束。\n")
        out = []
        for row in rows:
            has_depth = bool(row.get("valid_depth")) and ("vision_base_m" in row)
            has_hom = bool(row.get("valid_homography")) and ("homography_base_m" in row)
            if not has_depth and not has_hom:
                out.append(row)
                continue
            pid = row["id"]
            while not rospy.is_shutdown():
                ans = input("P%d: 移动 TCP 到该物理点后回车记录 [s=跳过 q=结束]: " % pid).strip().lower()
                if ans == "q":
                    return out
                if ans == "s":
                    row["skipped"] = True
                    out.append(row)
                    break
                # 触点两把尺子：TF(URDF 系) 比深度+手眼；固件位姿(固件系) 比单应性。
                try:
                    p_touch_tf = lookup_point(self.tf_buffer, self.base_frame, self.eef_frame)
                except Exception as e:
                    print("  读取 TCP(TF) 失败: %s" % e)
                    continue
                p_touch_fw = get_firmware_touch()
                fw_used = p_touch_fw is not None
                if not fw_used:
                    p_touch_fw = p_touch_tf
                    if has_hom:
                        print("  ⚠️ 固件位姿不可用，本点单应性对照回退 TF 触点（会带 URDF↔固件偏差）。")

                row["touch_base_tf_m"] = [float(v) for v in p_touch_tf]
                row["touch_base_fw_m"] = [float(v) for v in p_touch_fw]
                row["touch_fw_used"] = bool(fw_used)
                if self.board_R is not None:
                    row["touch_board_tf_m"] = [float(v) for v in self.board_R.T.dot(p_touch_tf - self.board_P)]
                    row["touch_board_fw_m"] = [float(v) for v in self.board_R.T.dot(p_touch_fw - self.board_P)]

                parts = []
                if has_depth:
                    # 深度+手眼预测在 URDF/TF 系 → 用 TF 触点比。
                    diff = p_touch_tf - np.asarray(row["vision_base_m"], dtype=np.float64)
                    row["error_m"] = [float(v) for v in diff]
                    row["error_norm_m"] = float(np.linalg.norm(diff))
                    row["error_xy_m"] = float(np.linalg.norm(diff[:2]))
                    if self.board_R is not None:
                        row["error_board_m"] = [float(v) for v in self.board_R.T.dot(diff)]
                    parts.append("深度+手眼[TF] dx=%+.2f dy=%+.2f dz=%+.2f |xy|=%.2f mm" %
                                 (diff[0] * 1000.0, diff[1] * 1000.0, diff[2] * 1000.0,
                                  row["error_xy_m"] * 1000.0))
                if has_hom:
                    # 单应性预测在固件系 → 用固件触点比（不可用已回退 TF）。
                    hdiff = p_touch_fw - np.asarray(row["homography_base_m"], dtype=np.float64)
                    row["homography_error_m"] = [float(v) for v in hdiff]
                    row["homography_error_xy_m"] = float(np.linalg.norm(hdiff[:2]))
                    parts.append("单应性[%s] dx=%+.2f dy=%+.2f |xy|=%.2f mm" %
                                 ("固件" if fw_used else "TF回退",
                                  hdiff[0] * 1000.0, hdiff[1] * 1000.0,
                                  row["homography_error_xy_m"] * 1000.0))
                print("  P%d  " % pid + "  |  ".join(parts))
                out.append(row)
                break
        return out

    @staticmethod
    def _err_stats(rows, key_vec):
        valid = [r for r in rows if r.get(key_vec) is not None]
        if not valid:
            return None
        E = np.asarray([r[key_vec] for r in valid], dtype=np.float64)
        norms = np.linalg.norm(E, axis=1)
        xy = np.linalg.norm(E[:, :2], axis=1)
        return {
            "count": int(len(valid)),
            "mean_error_m": [float(v) for v in E.mean(axis=0)],
            "median_abs_error_m": [float(v) for v in np.median(np.abs(E), axis=0)],
            "rms_xy_m": float(np.sqrt(np.mean(xy ** 2))),
            "rms_3d_m": float(np.sqrt(np.mean(norms ** 2))),
            "max_xy_m": float(np.max(xy)),
            "max_3d_m": float(np.max(norms)),
        }

    @staticmethod
    def _fw_tf_offset(rows):
        """固件触点 - TF 触点 的系统偏差（仅统计真正用到固件位姿的点）= URDF↔固件运动学差。"""
        pairs = [(r["touch_base_fw_m"], r["touch_base_tf_m"]) for r in rows
                 if r.get("touch_fw_used") and r.get("touch_base_fw_m") is not None
                 and r.get("touch_base_tf_m") is not None]
        if not pairs:
            return None
        D = np.asarray([np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)
                        for a, b in pairs], dtype=np.float64)
        return {
            "count": int(len(pairs)),
            "mean_m": [float(v) for v in D.mean(axis=0)],
            "rms_xy_m": float(np.sqrt(np.mean(np.linalg.norm(D[:, :2], axis=1) ** 2))),
        }

    def summarize(self, rows):
        depth_s = self._err_stats(rows, "error_m")
        hom_s = self._err_stats(rows, "homography_error_m")
        off = self._fw_tf_offset(rows)
        if depth_s is None and hom_s is None:
            print("\n没有有效对比点。")
            return {}
        print("\n========== 误差汇总（以实际 touch 为基准） ==========")
        if depth_s:
            print("[深度+手眼·TF系 ] n=%d  mean dx/dy/dz=%+.2f/%+.2f/%+.2f mm  RMS xy=%.2f mm  MAX xy=%.2f mm" %
                  (depth_s["count"], depth_s["mean_error_m"][0] * 1000.0,
                   depth_s["mean_error_m"][1] * 1000.0, depth_s["mean_error_m"][2] * 1000.0,
                   depth_s["rms_xy_m"] * 1000.0, depth_s["max_xy_m"] * 1000.0))
        if hom_s:
            src = "固件系" if getattr(self, "fw_enabled", False) else "TF回退"
            print("[单应性+视差·%s] n=%d  mean dx/dy   =%+.2f/%+.2f mm        RMS xy=%.2f mm  MAX xy=%.2f mm" %
                  (src, hom_s["count"], hom_s["mean_error_m"][0] * 1000.0,
                   hom_s["mean_error_m"][1] * 1000.0,
                   hom_s["rms_xy_m"] * 1000.0, hom_s["max_xy_m"] * 1000.0))
        elif self.compare_homography:
            print("[单应性+视差] 无（缺 PICK_HOMOGRAPHY / BOARD_POSE_BASE？）")
        if off:
            print("[URDF↔固件偏差] n=%d  mean dx/dy/dz=%+.2f/%+.2f/%+.2f mm  RMS xy=%.2f mm （固件touch - TF touch）" %
                  (off["count"], off["mean_m"][0] * 1000.0, off["mean_m"][1] * 1000.0,
                   off["mean_m"][2] * 1000.0, off["rms_xy_m"] * 1000.0))
        print("提示：单应性+视差(固件系)=控制器运行时真正用的 XY，与标定/move_line 同系→它小=生产抓放准。")
        print("单应性大→标定/位姿/视差要查；深度+手眼(TF系)只是感知粗核对(本就~cm 级)，两系差见上行。")
        print("===================================================\n")
        return {"depth_handeye": depth_s, "homography": hom_s,
                "urdf_vs_firmware_offset": off,
                "firmware_pose_used": bool(getattr(self, "fw_enabled", False))}

    def save(self, rows, summary, K, D):
        if not self.save_path:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.save_path = "/tmp/lucky_pixel_touch_check_%s.yaml" % stamp
        data = {
            "frames": {
                "base_frame": self.base_frame,
                "eef_frame": self.eef_frame,
                "camera_frame": self.camera_frame,
            },
            "topics": {
                "image_topic": self.image_topic,
                "depth_topic": self.depth_topic,
                "camera_info_topic": self.camera_info_topic,
            },
            "camera": {
                "K": [float(v) for v in K.reshape(-1)],
                "D": [float(v) for v in D.reshape(-1)],
            },
            "params": {
                "num_points": int(self.num_points),
                "capture_frames": int(self.capture_frames),
                "sample_win_half": int(self.sample_win_half),
                "sample_min_valid": int(self.sample_min_valid),
            },
            "summary": summary,
            "points": rows,
        }
        os.makedirs(os.path.dirname(self.save_path) or ".", exist_ok=True)
        with open(self.save_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        print("诊断结果已保存: %s" % self.save_path)

    def run(self):
        self.wait_ready()
        self.fw_enabled = setup_firmware_pose(self.tf_buffer, self.base_frame, self.eef_frame)
        if not self.select_pixels():
            print("已取消。")
            return
        rows, K, D = self.compute_vision_points()
        rows = self.collect_touch_points(rows)
        summary = self.summarize(rows)
        self.save(rows, summary, K, D)


if __name__ == "__main__":
    try:
        PixelTouchCheck().run()
    except Exception as e:
        rospy.logerr("pixel_touch_check 失败: %s", e)
        raise
