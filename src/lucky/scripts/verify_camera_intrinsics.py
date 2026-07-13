#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机内参验证（RealSense / xArm 项目）。

为什么要验证：calibrate_board.py / align_tool_to_board.py / 视觉深度反投影都用
/camera/color/camera_info 的 K(fx,fy,cx,cy) + D(畸变) 把像素反投影成 3D。内参或
对齐不对，深度 Z 和 XY 全错。本脚本做三层检查：

  1) report —— 打印并体检 K / D / 分辨率，并核对【color 与 aligned_depth 的内参一致】
     （对齐深度反投影必须用同一组内参）。无需任何标定物。
  2) points —— 交互：在画面上点两个点，用对齐深度 + 内参反投影成 3D，报两点的
     深度与 3D 直线距离；你拿尺子量真实距离对比 → 同时验证 fx/fy 与深度尺度。
  3) aruco  —— 可选：检测已知边长的 ArUco，solvePnP 算重投影 RMS（与深度无关的纯
     内参质量），并对比 PnP 的 Z 与对齐深度的 Z（验证深度尺度 vs 内参）。

用法：
  rosrun lucky verify_camera_intrinsics.py                       # report + points(默认)
  rosrun lucky verify_camera_intrinsics.py _mode:=report         # 只体检内参
  rosrun lucky verify_camera_intrinsics.py _mode:=points _known_distance_m:=0.10
  rosrun lucky verify_camera_intrinsics.py _mode:=aruco _marker_length_m:=0.05
需要：RealSense 在线（align_depth:=true）。points/aruco 需要桌面有显示器（弹窗）。
"""

import sys
import numpy as np
import rospy

try:
    import cv2
except Exception:
    cv2 = None

from sensor_msgs.msg import Image, CameraInfo

COLOR_INFO_DEFAULT = "/camera/color/camera_info"
DEPTH_INFO_DEFAULT = "/camera/aligned_depth_to_color/camera_info"
COLOR_TOPIC_DEFAULT = "/camera/color/image_raw"
DEPTH_TOPIC_DEFAULT = "/camera/aligned_depth_to_color/image_raw"


# ----------------------------- 工具 -----------------------------

def wait_info(topic, timeout=5.0):
    return rospy.wait_for_message(topic, CameraInfo, timeout=timeout)


def K_of(info):
    return np.array(info.K, dtype=np.float64).reshape(3, 3)


def D_of(info):
    return np.array(info.D, dtype=np.float64) if len(info.D) else np.zeros(5)


def color_msg_to_bgr(msg):
    enc = msg.encoding
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    if enc in ("mono8",):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise RuntimeError("不支持的彩色编码: %s" % enc)


def depth_msg_to_meters(msg):
    if msg.encoding in ("16UC1", "mono16"):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).astype(np.float32).copy()
    else:
        raise RuntimeError("不支持的深度编码: %s" % msg.encoding)
    d[~np.isfinite(d)] = 0.0
    return d


def grab_median_depth(topic, n, timeout=5.0):
    stack, h, w = [], None, None
    for _ in range(max(1, n)):
        try:
            m = rospy.wait_for_message(topic, Image, timeout=timeout)
        except Exception:
            continue
        d = depth_msg_to_meters(m)
        if h is None:
            h, w = d.shape
        elif d.shape != (h, w):
            continue
        stack.append(d)
    if not stack:
        raise RuntimeError("未取到深度帧: %s" % topic)
    a = np.stack(stack, 0)
    a[a <= 0] = np.nan
    with np.errstate(all="ignore"):
        med = np.nanmedian(a, 0)
    med[~np.isfinite(med)] = 0.0
    return med.astype(np.float32)


def depth_at(depth_m, u, v, win=2):
    H, W = depth_m.shape
    u0, u1 = max(0, u - win), min(W, u + win + 1)
    v0, v1 = max(0, v - win), min(H, v + win + 1)
    patch = depth_m[v0:v1, u0:u1].reshape(-1)
    patch = patch[patch > 0]
    return float(np.median(patch)) if patch.size else 0.0


def deproject(u, v, Z, K, D):
    """像素(u,v)+深度Z → 相机系 3D，正确处理畸变（D≈0 时退化为针孔）。"""
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
    und = cv2.undistortPoints(pts, K, D)   # 归一化坐标 (x', y')
    x, y = und[0, 0]
    return np.array([x * Z, y * Z, Z], dtype=float)


# ----------------------------- 1. report -----------------------------

def check_report(color_topic, depth_topic):
    info = wait_info(color_topic)
    K, D = K_of(info), D_of(info)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    W, H = info.width, info.height
    P = np.array(info.P, dtype=np.float64).reshape(3, 4)

    print("\n========== 内参体检 (%s) ==========" % color_topic)
    print("  分辨率: %dx%d   畸变模型: %s" % (W, H, info.distortion_model or "(空)"))
    print("  fx=%.3f fy=%.3f  cx=%.3f cy=%.3f" % (fx, fy, cx, cy))
    print("  D = %s" % np.array2string(D, precision=5))

    ok = True
    if fx <= 0 or fy <= 0:
        print("  ❌ fx/fy 非正，相机内参无效。"); ok = False
    dfxy = abs(fx - fy) / max(fx, 1e-9)
    print("  fx≈fy: 偏差 %.2f%% %s" % (dfxy * 100, "✅" if dfxy < 0.02 else "⚠️(>2%, RealSense 通常很接近)"))
    ecx, ecy = abs(cx - W / 2.0) / W, abs(cy - H / 2.0) / H
    print("  主点居中: cx偏%.1f%%宽 cy偏%.1f%%高 %s" %
          (ecx * 100, ecy * 100, "✅" if (ecx < 0.05 and ecy < 0.05) else "⚠️(>5%)"))
    maxd = float(np.max(np.abs(D))) if D.size else 0.0
    if maxd < 1e-6:
        print("  畸变≈0 ✅（color 多为出厂已校正流；反投影按针孔即可）")
    else:
        print("  ⚠️ 畸变非0(max|D|=%.4f)：反投影必须 undistort（本脚本已用 undistortPoints）；"
              "另注意深度 rect→raw 映射（plan.md §11）。" % maxd)
    # K 应等于 P[:3,:3]（已校正流）
    if np.max(np.abs(K - P[:3, :3])) > 1e-3:
        print("  ⚠️ K ≠ P[:3,:3]：color_info 与投影矩阵不一致，确认订阅的是同一流。")

    # color vs aligned_depth 内参一致性（对齐反投影必须一致）
    try:
        dinfo = wait_info(depth_topic.replace("/image_raw", "/camera_info")
                          if depth_topic.endswith("/image_raw") else DEPTH_INFO_DEFAULT)
        Kd = K_of(dinfo)
        dmax = float(np.max(np.abs(Kd - K)))
        same_res = (dinfo.width == W and dinfo.height == H)
        print("  对齐深度内参 vs color: K 最大差 %.4f, 分辨率%s %s" %
              (dmax, "一致" if same_res else "不一致", "✅" if (dmax < 1e-3 and same_res) else "❌"))
        if dmax >= 1e-3 or not same_res:
            print("     → aligned_depth 与 color 内参不一致，深度反投影会错。检查 align_depth 配置。")
            ok = False
    except Exception as e:
        print("  ⚠️ 拿不到对齐深度内参(%s)：确认 align_depth:=true。" % e)

    print("=" * 44)
    return info, K, D, ok


# ----------------------------- 2. points -----------------------------

def check_points(K, D, color_topic, depth_topic, frames, known_dist):
    if cv2 is None:
        print("cv2 不可用，跳过 points。"); return
    print("\n--- 交互 2 点测距：点两个点(同一平面更准)，对比尺子量的真实距离 ---")
    print("    左键选点(满2点自动算)；r=重选；q=退出。")
    cmsg = rospy.wait_for_message(color_topic, Image, timeout=5.0)
    color = color_msg_to_bgr(cmsg)
    depth = grab_median_depth(depth_topic, frames)

    state = {"pts": []}

    def on_mouse(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN and len(state["pts"]) < 2:
            state["pts"].append((x, y))

    win = "verify intrinsics: click 2 points (r=reset q=quit)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while not rospy.is_shutdown():
        disp = color.copy()
        for p in state["pts"]:
            cv2.circle(disp, p, 5, (0, 0, 255), -1)
        if len(state["pts"]) == 2:
            cv2.line(disp, state["pts"][0], state["pts"][1], (0, 255, 0), 2)
        cv2.imshow(win, disp)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('q'):
            break
        if key == ord('r'):
            state["pts"] = []
        if len(state["pts"]) == 2:
            (u1, v1), (u2, v2) = state["pts"]
            z1, z2 = depth_at(depth, u1, v1), depth_at(depth, u2, v2)
            if z1 <= 0 or z2 <= 0:
                print("  ⚠️ 某点深度无效(z1=%.3f z2=%.3f)，换个点(避开黑/反光/孔洞)。" % (z1, z2))
                state["pts"] = []
                continue
            P1 = deproject(u1, v1, z1, K, D)
            P2 = deproject(u2, v2, z2, K, D)
            dist = float(np.linalg.norm(P1 - P2))
            print("  P1(%d,%d) Z=%.3fm  P2(%d,%d) Z=%.3fm  → 3D距离=%.4f m (%.1f mm)" %
                  (u1, v1, z1, u2, v2, z2, dist, dist * 1000))
            if known_dist > 0:
                err = dist - known_dist
                print("     已知=%.4f m, 误差=%+.1f mm (%.2f%%) %s" %
                      (known_dist, err * 1000, abs(err) / known_dist * 100,
                       "✅" if abs(err) / known_dist < 0.02 else "⚠️(>2%)"))
            state["pts"] = []
    cv2.destroyWindow(win)


# ----------------------------- 3. aruco -----------------------------

def check_aruco(K, D, color_topic, depth_topic, frames, marker_len):
    if cv2 is None or not hasattr(cv2, "aruco"):
        print("cv2.aruco 不可用，跳过 aruco。"); return
    if marker_len <= 0:
        print("aruco 模式需 _marker_length_m:=<打印的实际边长>，跳过。"); return
    print("\n--- ArUco 重投影 + 深度尺度校验 (marker_length=%.4f m) ---" % marker_len)
    cmsg = rospy.wait_for_message(color_topic, Image, timeout=5.0)
    color = color_msg_to_bgr(cmsg)
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    depth = grab_median_depth(depth_topic, frames)

    dicts = [cv2.aruco.DICT_ARUCO_ORIGINAL, cv2.aruco.DICT_4X4_50,
             cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_6X6_250, cv2.aruco.DICT_7X7_50]
    found = False
    for dflag in dicts:
        ad = cv2.aruco.getPredefinedDictionary(dflag)
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, ad)
        except Exception:
            continue
        if ids is None or len(ids) == 0:
            continue
        found = True
        h = marker_len / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        for c, mid in zip(corners, ids.flatten()):
            img_pts = c.reshape(-1, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img_pts) ** 2, axis=1))))
            z_pnp = float(tvec[2])
            cu, cv_ = img_pts.mean(0)
            z_depth = depth_at(depth, int(round(cu)), int(round(cv_)))
            print("  id=%d  重投影RMS=%.2f px %s  PnP_Z=%.3f m  深度_Z=%.3f m  ΔZ=%+.1f mm" %
                  (mid, rms, "✅" if rms < 1.0 else "⚠️(>1px)", z_pnp, z_depth,
                   (z_depth - z_pnp) * 1000 if z_depth > 0 else float('nan')))
            print("     重投影RMS小 → 内参/畸变好；ΔZ 小 → 深度尺度与内参一致。")
        break
    if not found:
        print("  未检测到 ArUco（确认画面里有 marker、字典匹配）。")


# ----------------------------- main -----------------------------

def main():
    rospy.init_node("verify_camera_intrinsics", anonymous=True)
    modes = str(rospy.get_param("~mode", "report,points")).replace(" ", "").split(",")
    color_info = rospy.get_param("~color_info_topic", COLOR_INFO_DEFAULT)
    color_topic = rospy.get_param("~color_topic", COLOR_TOPIC_DEFAULT)
    depth_topic = rospy.get_param("~depth_topic", DEPTH_TOPIC_DEFAULT)
    frames = int(rospy.get_param("~capture_frames", 30))
    known_dist = float(rospy.get_param("~known_distance_m", 0.0))
    marker_len = float(rospy.get_param("~marker_length_m", 0.0))

    try:
        info, K, D, ok = check_report(color_info, depth_topic)
    except Exception as e:
        rospy.logerr("拿不到 %s：相机没起？(%s)", color_info, e)
        sys.exit(1)

    if "points" in modes:
        try:
            check_points(K, D, color_topic, depth_topic, frames, known_dist)
        except Exception as e:
            rospy.logwarn("points 失败：%s", e)
    if "aruco" in modes:
        try:
            check_aruco(K, D, color_topic, depth_topic, frames, marker_len)
        except Exception as e:
            rospy.logwarn("aruco 失败：%s", e)

    print("\n内参体检结论: %s" % ("通过 ✅" if ok else "有问题 ❌（见上）"))


if __name__ == "__main__":
    main()
