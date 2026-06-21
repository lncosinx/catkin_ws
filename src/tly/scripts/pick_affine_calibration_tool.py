#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_affine_calibration_tool.py (Manual Magnifier + Homography)

带有高倍放大镜的手动点选标定工具。
彻底抛弃视觉节点的自动识别（因为方块物理中心难以肉眼对齐），改为在画面中手动点击标定纸上的 9 个黑点。
底层的数学模型采用单应性矩阵 (Homography)，完美消除手眼标定的物理残差和相机倾斜透视畸变。

坐标系：已全量 base 化，不再有 table_frame TF。采点时读 base<-eef，再用 config 里的
BOARD_POSE_BASE（board_frame 在 base 的常量位姿，由 calibrate_board 派生）换算到 board 系，
拟合 pixel→board-XY 的单应性。⚠️ 每次 calibrate_board 重定坐标系后，必须重跑本工具。

输出写入 tetris_config.yaml:
  tetris/PICK_HOMOGRAPHY  (pixel -> board-XY)
"""

import os
import yaml
import rospy
import tf2_ros
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf.transformations import euler_matrix

DEFAULT_CONFIG_PATH = "/root/catkin_ws/src/tly/config/tetris_config.yaml"

class PickAffineCalibrator:
    def __init__(self):
        rospy.init_node("pick_affine_calibrator", anonymous=True)

        self.config_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
        self.base_frame = rospy.get_param("~base_frame", "link_base")
        self.eef_frame = rospy.get_param("~eef_frame", "link_tcp")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_rect_color")
        self.num_points = int(rospy.get_param("~num_points", 9))

        if self.num_points < 4:
            self.num_points = 9

        # board_frame 在 base 的常量位姿（由 calibrate_board 派生）。base->board: R^T (p-P)。
        self.board_R, self.board_P = self.load_board_pose()

        self.bridge = CvBridge()
        self.latest_image = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        self.manual_points = []
        self.mouse_x = 0
        self.mouse_y = 0

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def load_board_pose(self):
        """从 config 读取 BOARD_POSE_BASE（board_frame 在 base 的位姿）。返回 (R 3x3, P 3)。"""
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                t = (yaml.safe_load(f) or {}).get("tetris", {})
            bp = t.get("BOARD_POSE_BASE")
            if not bp:
                raise RuntimeError("缺少 tetris/BOARD_POSE_BASE，请先运行 calibrate_board.py")
            P = np.asarray(bp["origin"], dtype=np.float64)
            r = [float(v) for v in bp["rpy"]]
            R = euler_matrix(r[0], r[1], r[2], axes="sxyz")[:3, :3]
            rospy.loginfo("已加载 board_frame: origin=%s rpy=%s", P.tolist(), r)
            return R, P
        except Exception as e:
            rospy.logerr("加载 BOARD_POSE_BASE 失败: %s", e)
            raise

    def wait_required_tf(self):
        rospy.loginfo("等待必要 TF: %s <- %s", self.base_frame, self.eef_frame)
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.tf_buffer.can_transform(self.base_frame, self.eef_frame, rospy.Time(0), rospy.Duration(0.2)):
                return True
            rate.sleep()
        return False

    def wait_for_image(self):
        rospy.loginfo("等待相机图像...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                return
            rate.sleep()

    def mouse_callback(self, event, x, y, flags, param):
        self.mouse_x = x
        self.mouse_y = y
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.manual_points) < self.num_points:
                self.manual_points.append({"u": x, "v": y, "calib_id": len(self.manual_points) + 1})
                print(f"🎯 已添加点 P{len(self.manual_points)}: 像素坐标 (u={x}, v={y})")
            else:
                print(f"已经选满了 {self.num_points} 个点，请按『回车键』确认。")
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.manual_points:
                removed = self.manual_points.pop()
                print(f"↩️ 已撤销点 P{removed['calib_id']}")

    def select_points_manually(self):
        print("\n" + "=" * 90)
        print("🔍 请在弹出的图像窗口中手动精确标点 (带十字放大镜)：")
        print("  - 鼠标移动：右上角会显示高倍放大镜")
        print("  - 鼠标左键：精确点选一个黑点中心")
        print("  - 鼠标右键：撤销上一个点")
        print(f"  - 选满 {self.num_points} 个点后，按『回车键 (Enter)』确认。")
        print("=" * 90 + "\n")

        cv2.namedWindow("Select Points (Zoom Enabled)", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Select Points (Zoom Enabled)", self.mouse_callback)

        while not rospy.is_shutdown():
            if self.latest_image is None:
                continue
                
            display_img = self.latest_image.copy()
            
            # 画已经选择的点
            for d in self.manual_points:
                u, v = d["u"], d["v"]
                cid = d["calib_id"]
                cv2.drawMarker(display_img, (u, v), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
                cv2.circle(display_img, (u, v), 10, (0, 0, 255), 2)
                cv2.putText(display_img, f"P{cid}", (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 渲染右上角高倍放大镜
            mag_size = 30  # 截取鼠标周围 30x30 像素
            mag_scale = 6  # 放大 6 倍
            mx, my = self.mouse_x, self.mouse_y
            
            x0, y0 = max(0, mx - mag_size), max(0, my - mag_size)
            x1, y1 = min(display_img.shape[1], mx + mag_size), min(display_img.shape[0], my + mag_size)
            patch = self.latest_image[y0:y1, x0:x1]
            
            if patch.size > 0:
                zoomed = cv2.resize(patch, (0, 0), fx=mag_scale, fy=mag_scale, interpolation=cv2.INTER_NEAREST)
                
                # 画放大镜中心的准星
                zx = (mx - x0) * mag_scale
                zy = (my - y0) * mag_scale
                cv2.drawMarker(zoomed, (int(zx), int(zy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                
                # 贴到右上角
                zh, zw = zoomed.shape[:2]
                top_left_x = display_img.shape[1] - zw - 10
                top_left_y = 10
                
                # 绘制黑框和放大图像
                cv2.rectangle(display_img, (top_left_x - 2, top_left_y - 2), (top_left_x + zw + 2, top_left_y + zh + 2), (0, 0, 0), -1)
                display_img[top_left_y:top_left_y+zh, top_left_x:top_left_x+zw] = zoomed
                cv2.rectangle(display_img, (top_left_x, top_left_y), (top_left_x + zw, top_left_y + zh), (0, 255, 0), 2)

            text = f"Selected: {len(self.manual_points)}/{self.num_points}. Left=Add, Right=Undo, Enter=Confirm"
            cv2.putText(display_img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow("Select Points (Zoom Enabled)", display_img)
            key = cv2.waitKey(30) & 0xFF
            
            if key == 13 and len(self.manual_points) == self.num_points:
                break
            elif key == ord('q'):
                cv2.destroyAllWindows()
                return False

        cv2.destroyAllWindows()
        return True

    def get_eef_board_point(self):
        """读 base<-eef，再用常量 board 位姿换算到 board 系。返回 board 系 [x,y,z]。"""
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.eef_frame, rospy.Time(0), rospy.Duration(3.0))
            p_base = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ], dtype=np.float64)
            return self.board_R.T.dot(p_base - self.board_P)
        except Exception as e:
            rospy.logerr("获取 TF 失败: %s", e)
            return None

    def fit_homography(self, rows):
        """核心：计算单应性透视变换矩阵 (pixel -> board-XY)"""
        src_pts = np.array([[r["u"], r["v"]] for r in rows], dtype=np.float32)
        dst_pts = np.array([[r["actual_board"][0], r["actual_board"][1]] for r in rows], dtype=np.float32)

        H, _ = cv2.findHomography(src_pts, dst_pts)

        rms_sq = 0.0
        for i in range(len(src_pts)):
            pt = np.array([src_pts[i][0], src_pts[i][1], 1.0])
            pred = H.dot(pt)
            pred = pred / pred[2]
            dx = dst_pts[i][0] - pred[0]
            dy = dst_pts[i][1] - pred[1]
            rms_sq += dx*dx + dy*dy
        rms = float(np.sqrt(rms_sq / len(src_pts)))

        return {
            "enabled": True,
            "model": "x = (H00*u + H01*v + H02)/W; y = (H10*u + H11*v + H12)/W; W = H20*u + H21*v + H22",
            "matrix": [float(v) for v in H.flatten()],
            "rms_m": rms,
            "sample_count": len(rows),
        }

    def save_config(self, correction, rows):
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        cfg.setdefault("tetris", {})
        cfg["tetris"]["PICK_HOMOGRAPHY"] = correction
        
        # 禁用旧的 affine 防止冲突
        if "PICK_AFFINE_CORRECTION" in cfg["tetris"]:
            cfg["tetris"]["PICK_AFFINE_CORRECTION"]["enabled"] = False

        backup = self.config_path + ".bak_before_homography"
        if os.path.exists(self.config_path):
            with open(self.config_path, "r", encoding="utf-8") as f:
                old = f.read()
            with open(backup, "w", encoding="utf-8") as f:
                f.write(old)

        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

        rospy.loginfo("✅ 已写入纯单应性矩阵 (Homography) 到: %s", self.config_path)

    def run(self):
        if not self.wait_required_tf():
            return
        self.wait_for_image()

        if not self.select_points_manually():
            return

        print("\n" + "=" * 90)
        print("🤖 单应性矩阵物理对齐采样")
        print("请按顺序手动移动吸盘中心，压在你刚才点选的【物理黑点】上，按回车记录。")
        print("提示：如果标定失误，输入 'u' 并回车可以撤销重新记录上一个点。")
        print("=" * 90 + "\n")

        rows = []
        i = 0
        while i < len(self.manual_points):
            d = self.manual_points[i]
            cid = int(d["calib_id"])
            u, v = int(d["u"]), int(d["v"])

            print(f"\n[P{cid}/{len(self.manual_points)}] 刚才选择的精确像素=({u}, {v})")
            
            prompt_str = f"  👉 请将机械臂吸盘移动到 P{cid} 的真实物理黑点位置并按回车"
            if i > 0:
                prompt_str += " [或输入 'u' 撤销]"
            
            ans = input(prompt_str + ": ").strip().lower()
            if ans in ('u', 'undo', '撤销') and i > 0:
                i -= 1
                rows.pop()
                print(f"  ↩️ 已撤销！退回上一个点 P{self.manual_points[i]['calib_id']} 重新记录。")
                continue

            actual_board = self.get_eef_board_point()
            if actual_board is None:
                continue

            print("  ✅ 已记录 board 系坐标: x={:.6f}, y={:.6f}".format(actual_board[0], actual_board[1]))
            rows.append({
                "id": cid, "u": u, "v": v, "actual_board": actual_board,
            })
            i += 1

        corr = self.fit_homography(rows)
        print("\n" + "=" * 90)
        print("🎉 单应性矩阵拟合完成！")
        print("  均方根误差 RMS (越接近0越完美): {:.3f} mm".format(corr["rms_m"] * 1000.0))
        print("=" * 90)

        if input("是否写入 tetris_config.yaml？[Y/n]: ").strip().lower() not in ("n", "no"):
            self.save_config(corr, rows)
            print("✅ 写入完毕！现在抓取绝对指哪打哪！请重新启动控制节点进行测试。")

if __name__ == "__main__":
    try:
        node = PickAffineCalibrator()
        node.run()
    except rospy.ROSInterruptException:
        pass