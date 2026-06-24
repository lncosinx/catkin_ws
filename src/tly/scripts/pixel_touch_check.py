#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
像素→深度→base 与实际 TCP 触点对比诊断。

流程：
  1) 在彩色图上手动点选 N 个物理点（默认 9 个）。
  2) 脚本从 aligned_depth_to_color 取多帧中值深度，把每个像素反投影到相机系。
  3) 用当前 TF(base <- camera_color_optical_frame) 转到 base 系。
  4) 用户手动移动 link_tcp 到同一个物理点，按回车记录。
  5) 打印并保存 touch_base - vision_base 误差。

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


DEFAULT_CONFIG_PATH = "/root/catkin_ws/src/tly/config/tetris_config.yaml"


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

        self.bridge = CvBridge()
        self.latest_image = None
        self.points = []
        self.mouse_x = 0
        self.mouse_y = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.board_R, self.board_P = load_board_pose(self.config_path)

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

    def mouse_cb(self, event, x, y, flags, param):
        self.mouse_x, self.mouse_y = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < self.num_points:
                self.points.append({"id": len(self.points) + 1, "u": int(x), "v": int(y)})
                print("  添加 P%d pixel=(%d,%d)" % (len(self.points), x, y))
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
        print("\n手动点选像素：左键添加，右键撤销，c 清空，Enter 确认，q 退出。")
        print("建议按固定顺序点 3x3，例如左上→中上→右上→左中→...→右下。\n")
        win = "pixel touch check: select points"
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
            cv2.putText(display, "Selected %d/%d  Left:add Right:undo c:clear Enter:ok q:quit" %
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
            if key == ord("q"):
                cv2.destroyWindow(win)
                return False
        cv2.destroyWindow(win)
        return False

    def compute_vision_points(self):
        info = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=5.0)
        K, D = camera_info_to_KD(info)
        print("相机内参: %dx%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f Dmax=%.6f" %
              (info.width, info.height, K[0, 0], K[1, 1], K[0, 2], K[1, 2],
               float(np.max(np.abs(D))) if D.size else 0.0))
        depth = grab_median_depth(self.depth_topic, self.capture_frames)
        R_cb, t_cb = lookup_R_t(self.tf_buffer, self.base_frame, self.camera_frame)

        rows = []
        for p in self.points:
            z = sample_depth(depth, p["u"], p["v"], self.sample_win_half, self.sample_min_valid)
            row = dict(p)
            row["depth_m"] = z
            if z is None:
                print("  P%d pixel=(%d,%d) 深度无效，后续会跳过。" % (p["id"], p["u"], p["v"]))
                row["valid_depth"] = False
                rows.append(row)
                continue
            p_cam = pixel_depth_to_camera(p["u"], p["v"], z, K, D)
            p_base = R_cb.dot(p_cam) + t_cb
            row["valid_depth"] = True
            row["camera_xyz_m"] = [float(v) for v in p_cam]
            row["vision_base_m"] = [float(v) for v in p_base]
            if self.board_R is not None:
                p_board = self.board_R.T.dot(p_base - self.board_P)
                row["vision_board_m"] = [float(v) for v in p_board]
            rows.append(row)
            print("  P%d pixel=(%d,%d) z=%.4f -> base=[%.5f %.5f %.5f]" %
                  (p["id"], p["u"], p["v"], z, p_base[0], p_base[1], p_base[2]))
        return rows, K, D

    def collect_touch_points(self, rows):
        print("\n现在按刚才的 P1..P%d 顺序移动机械臂。" % self.num_points)
        print("每次让 link_tcp/吸嘴尖对准同一个物理点，按回车记录；s 跳过；q 结束。\n")
        out = []
        for row in rows:
            if not row.get("valid_depth"):
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
                try:
                    p_touch = lookup_point(self.tf_buffer, self.base_frame, self.eef_frame)
                except Exception as e:
                    print("  读取 TCP 失败: %s" % e)
                    continue
                p_vis = np.asarray(row["vision_base_m"], dtype=np.float64)
                diff = p_touch - p_vis
                row["touch_base_m"] = [float(v) for v in p_touch]
                row["error_m"] = [float(v) for v in diff]
                row["error_norm_m"] = float(np.linalg.norm(diff))
                row["error_xy_m"] = float(np.linalg.norm(diff[:2]))
                if self.board_R is not None:
                    p_board = self.board_R.T.dot(p_touch - self.board_P)
                    row["touch_board_m"] = [float(v) for v in p_board]
                    row["error_board_m"] = [float(v) for v in self.board_R.T.dot(diff)]
                print("  P%d 误差 touch-vision: dx=%+.2f dy=%+.2f dz=%+.2f mm |xy|=%.2f |3d|=%.2f mm" %
                      (pid, diff[0] * 1000.0, diff[1] * 1000.0, diff[2] * 1000.0,
                       row["error_xy_m"] * 1000.0, row["error_norm_m"] * 1000.0))
                out.append(row)
                break
        return out

    def summarize(self, rows):
        valid = [r for r in rows if r.get("error_m") is not None]
        if not valid:
            print("\n没有有效对比点。")
            return {}
        E = np.asarray([r["error_m"] for r in valid], dtype=np.float64)
        norms = np.linalg.norm(E, axis=1)
        xy = np.linalg.norm(E[:, :2], axis=1)
        summary = {
            "count": int(len(valid)),
            "mean_error_m": [float(v) for v in E.mean(axis=0)],
            "median_abs_error_m": [float(v) for v in np.median(np.abs(E), axis=0)],
            "rms_3d_m": float(np.sqrt(np.mean(norms ** 2))),
            "rms_xy_m": float(np.sqrt(np.mean(xy ** 2))),
            "max_3d_m": float(np.max(norms)),
            "max_xy_m": float(np.max(xy)),
        }
        print("\n========== pixel/depth/hand-eye vs touch 误差汇总 ==========")
        print("有效点: %d" % summary["count"])
        print("mean dx/dy/dz: %+.2f / %+.2f / %+.2f mm" %
              tuple(np.asarray(summary["mean_error_m"]) * 1000.0))
        print("RMS xy/3d: %.2f / %.2f mm" %
              (summary["rms_xy_m"] * 1000.0, summary["rms_3d_m"] * 1000.0))
        print("MAX xy/3d: %.2f / %.2f mm" %
              (summary["max_xy_m"] * 1000.0, summary["max_3d_m"] * 1000.0))
        print("===========================================================\n")
        return summary

    def save(self, rows, summary, K, D):
        if not self.save_path:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.save_path = "/tmp/tly_pixel_touch_check_%s.yaml" % stamp
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
