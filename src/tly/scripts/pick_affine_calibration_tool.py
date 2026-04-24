#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_affine_calibration_tool.py

读取 /vision/board_state 和 /camera/color/image_rect_color，从画面中的方块吸取点里自动选择 9 个分散点。
然后使用与控制节点一致的逻辑：
  rectified pixel -> camera ray -> PICK_SURFACE_PLANE_BASE true pick plane -> table_frame
输出预测吸取点坐标。

你手动把机械臂吸盘中心移动到每个方块真实吸取点后按回车，程序记录 link_eef 在 table_frame 下的实际坐标，
并拟合二维仿射补偿：
  dx = ax0 + ax1*(u-cx) + ax2*(v-cy)
  dy = ay0 + ay1*(u-cx) + ay2*(v-cy)

输出写入 tetris_config.yaml:
  tetris/PICK_AFFINE_CORRECTION
  tetris/PICK_AFFINE_SAMPLES
"""

import os
import yaml
import rospy
import tf2_ros
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf.transformations import quaternion_matrix


DEFAULT_CONFIG_PATH = "/root/catkin_ws/src/tly/config/tetris_config.yaml"


class PickAffineCalibrator:
    def __init__(self):
        rospy.init_node("pick_affine_calibrator", anonymous=True)

        self.config_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
        self.base_frame = rospy.get_param("~base_frame", "link_base")
        self.eef_frame = rospy.get_param("~eef_frame", "link_eef")
        self.table_frame = rospy.get_param("~table_frame", "table_frame")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_rect_color")
        self.board_state_topic = rospy.get_param("~board_state_topic", "/vision/board_state")
        self.num_points = int(rospy.get_param("~num_points", 9))
        self.debug_topic = rospy.get_param("~debug_topic", "/calibration/pick_affine_debug")
        self.assume_image_rectified = bool(rospy.get_param("~assume_image_rectified", True))

        if self.num_points < 3:
            self.num_points = 9

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_state = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1, latch=True)

        # 采样开始前冻结拍照时刻的相机位姿。之后你移动机械臂记录真实点，
        # 不能再用当前相机 TF 重新计算其它图像点，否则 eye-on-hand 相机会跟着机械臂移动。
        self.T_base_cam_at_capture = None
        self.T_table_base_at_capture = None

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        rospy.Subscriber(self.board_state_topic, Int32MultiArray, self.state_cb, queue_size=1)

        self.load_config_planes()

        rospy.loginfo("等待 CameraInfo: %s", self.camera_info_topic)
        self.camera_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
        self.K = np.array(self.camera_info.K, dtype=np.float64).reshape(3, 3)
        self.P = np.array(self.camera_info.P, dtype=np.float64).reshape(3, 4)
        self.D = np.array(self.camera_info.D, dtype=np.float64).reshape(1, -1)
        rospy.loginfo(
            "CameraInfo frame=%s K fx=%.2f fy=%.2f cx=%.2f cy=%.2f | P fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            self.camera_info.header.frame_id,
            self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2],
            self.P[0, 0], self.P[1, 1], self.P[0, 2], self.P[1, 2],
        )

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.latest_image = None

    def state_cb(self, msg):
        self.latest_state = list(msg.data)

    def load_config_planes(self):
        if not os.path.exists(self.config_path):
            raise RuntimeError("配置文件不存在: {}".format(self.config_path))

        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        tetris = cfg.get("tetris", {})
        plane = tetris.get("PICK_SURFACE_PLANE_BASE", None)
        if not plane:
            raise RuntimeError("配置文件中缺少 tetris/PICK_SURFACE_PLANE_BASE，请先运行新版 calibration_tool.py")

        self.plane_point_base = np.array(plane["point"], dtype=np.float64)
        self.plane_normal_base = np.array(plane["normal"], dtype=np.float64)
        n = np.linalg.norm(self.plane_normal_base)
        if n < 1e-9:
            raise RuntimeError("PICK_SURFACE_PLANE_BASE.normal 长度为 0")
        self.plane_normal_base /= n

        rospy.loginfo(
            "Loaded PICK_SURFACE_PLANE_BASE point=(%.6f, %.6f, %.6f), normal=(%.6f, %.6f, %.6f)",
            *self.plane_point_base, *self.plane_normal_base
        )

    def wait_required_tf(self):
        rospy.loginfo(
            "等待必要 TF: %s <- %s, %s <- %s, %s <- %s",
            self.base_frame, self.camera_frame,
            self.table_frame, self.base_frame,
            self.table_frame, self.eef_frame
        )

        deadline = rospy.Time.now() + rospy.Duration(20.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            ok_base_cam = self.tf_buffer.can_transform(
                self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            ok_table_base = self.tf_buffer.can_transform(
                self.table_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            ok_table_eef = self.tf_buffer.can_transform(
                self.table_frame, self.eef_frame, rospy.Time(0), rospy.Duration(0.2)
            )

            if ok_base_cam and ok_table_base and ok_table_eef:
                rospy.loginfo("必要 TF 已就绪。")
                return True

            rospy.logwarn_throttle(
                2.0,
                "TF 未就绪: %s<- %s=%s, %s<- %s=%s, %s<- %s=%s",
                self.base_frame, self.camera_frame, ok_base_cam,
                self.table_frame, self.base_frame, ok_table_base,
                self.table_frame, self.eef_frame, ok_table_eef
            )
            rate.sleep()

        rospy.logerr("等待 TF 超时。当前 TF buffer 中已有 frames:\n%s", self.tf_buffer.all_frames_as_string())
        return False

    def freeze_camera_tf_for_current_image(self):
        """
        Eye-on-hand 相机会跟着机械臂运动。
        因此必须在开始人工记录前，把“拍照时刻”的 camera->base 和 base->table TF 冻结下来。
        后续 P1..P9 的预测点全部使用这个冻结 TF。
        """
        self.T_base_cam_at_capture = self.get_transform_matrix(self.base_frame, self.camera_frame)
        self.T_table_base_at_capture = self.get_transform_matrix(self.table_frame, self.base_frame)
        rospy.loginfo("已冻结拍照时刻 TF: %s <- %s, %s <- %s",
                      self.base_frame, self.camera_frame,
                      self.table_frame, self.base_frame)

    def wait_inputs(self):
        rospy.loginfo("等待图像和视觉识别结果...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_image is not None and self.latest_state is not None:
                data = self.latest_state
                if len(data) > 148 and data[147] > 0:
                    return
            rate.sleep()

    def parse_detections(self):
        data = self.latest_state
        n = int(data[147]) if len(data) > 147 else 0
        detections = []
        idx = 148
        for i in range(n):
            if idx + 3 >= len(data):
                break
            sid = int(data[idx])
            u = int(data[idx + 1])
            v = int(data[idx + 2])
            ang = int(data[idx + 3])
            detections.append({"idx": i + 1, "shape": sid, "u": u, "v": v, "ang": ang})
            idx += 4
        return detections

    def select_spread_points(self, detections):
        if len(detections) <= self.num_points:
            selected = detections
        else:
            pts = np.array([[d["u"], d["v"]] for d in detections], dtype=np.float64)
            selected_idx = []
            center = np.mean(pts, axis=0)
            start = int(np.argmin(np.linalg.norm(pts - center, axis=1)))
            selected_idx.append(start)

            while len(selected_idx) < self.num_points:
                dist_to_sel = np.min(
                    np.linalg.norm(pts[:, None, :] - pts[np.array(selected_idx)][None, :, :], axis=2),
                    axis=1
                )
                dist_to_sel[selected_idx] = -1
                selected_idx.append(int(np.argmax(dist_to_sel)))

            selected = [detections[i] for i in selected_idx]

        for k, d in enumerate(selected, start=1):
            d["calib_id"] = k
        return selected

    def draw_debug(self, selected):
        img = self.latest_image.copy()
        for d in selected:
            u, v = int(d["u"]), int(d["v"])
            cid = int(d["calib_id"])
            cv2.drawMarker(img, (u, v), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=25, thickness=2)
            cv2.circle(img, (u, v), 20, (0, 255, 255), 2)
            cv2.putText(img, f"P{cid}", (u + 15, v - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(img, f"u={u} v={v}", (u + 15, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.camera_frame
        self.debug_pub.publish(msg)
        rospy.loginfo("已发布 debug 图像: %s", self.debug_topic)

    def get_transform_matrix(self, target, source):
        trans = self.tf_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(3.0))
        q = trans.transform.rotation
        t = trans.transform.translation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z
        return T

    def pixel_to_ray_camera(self, u, v):
        if self.assume_image_rectified:
            fx, fy = self.P[0, 0], self.P[1, 1]
            cx, cy = self.P[0, 2], self.P[1, 2]
            return np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)

        src = np.array([[[float(u), float(v)]]], dtype=np.float64)
        und = cv2.undistortPoints(src, self.K, self.D)
        return np.array([und[0, 0, 0], und[0, 0, 1], 1.0], dtype=np.float64)

    def pixel_to_pick_point_table(self, u, v):
        if self.T_base_cam_at_capture is None or self.T_table_base_at_capture is None:
            raise RuntimeError("尚未冻结拍照时刻 TF，请先调用 freeze_camera_tf_for_current_image()")

        T_base_cam = self.T_base_cam_at_capture
        T_table_base = self.T_table_base_at_capture

        ray_cam = self.pixel_to_ray_camera(u, v)
        R_base_cam = T_base_cam[:3, :3]
        cam_pos_base = T_base_cam[:3, 3]
        ray_base = R_base_cam.dot(ray_cam)

        denom = float(np.dot(self.plane_normal_base, ray_base))
        if abs(denom) < 1e-9:
            raise RuntimeError("像素射线与 pick 平面平行")

        t = float(np.dot(self.plane_normal_base, self.plane_point_base - cam_pos_base) / denom)
        if t < 0:
            raise RuntimeError("像素射线交点在相机后方")

        hit_base = cam_pos_base + t * ray_base
        hit_table_h = T_table_base.dot(np.array([hit_base[0], hit_base[1], hit_base[2], 1.0]))
        return hit_base, hit_table_h[:3]

    def get_eef_table_point(self):
        trans = self.tf_buffer.lookup_transform(self.table_frame, self.eef_frame, rospy.Time(0), rospy.Duration(3.0))
        return np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ], dtype=np.float64)

    def fit_affine(self, rows):
        cx = float(self.P[0, 2] if self.assume_image_rectified else self.K[0, 2])
        cy = float(self.P[1, 2] if self.assume_image_rectified else self.K[1, 2])

        A = []
        bx = []
        by = []
        for r in rows:
            du = r["u"] - cx
            dv = r["v"] - cy
            A.append([1.0, du, dv])
            bx.append(r["actual_table"][0] - r["pred_table"][0])
            by.append(r["actual_table"][1] - r["pred_table"][1])

        A = np.asarray(A, dtype=np.float64)
        bx = np.asarray(bx, dtype=np.float64)
        by = np.asarray(by, dtype=np.float64)

        coef_x, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
        coef_y, _, _, _ = np.linalg.lstsq(A, by, rcond=None)

        res_x = bx - A.dot(coef_x)
        res_y = by - A.dot(coef_y)
        rms = float(np.sqrt(np.mean(res_x ** 2 + res_y ** 2)))

        return {
            "enabled": True,
            "model": "dx = ax0 + ax1*(u-cx) + ax2*(v-cy); dy = ay0 + ay1*(u-cx) + ay2*(v-cy)",
            "pixel_center": [cx, cy],
            "coeff_x": [float(v) for v in coef_x],
            "coeff_y": [float(v) for v in coef_y],
            "rms_m": rms,
            "sample_count": len(rows),
        }

    def save_config(self, correction, rows):
        with open(self.config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        cfg.setdefault("tetris", {})
        cfg["tetris"]["PICK_AFFINE_CORRECTION"] = correction
        cfg["tetris"]["PICK_AFFINE_SAMPLES"] = []

        for r in rows:
            cfg["tetris"]["PICK_AFFINE_SAMPLES"].append({
                "id": int(r["id"]),
                "shape": int(r["shape"]),
                "pixel": [int(r["u"]), int(r["v"])],
                "pred_table": [float(v) for v in r["pred_table"]],
                "actual_table": [float(v) for v in r["actual_table"]],
                "delta_table": [float(v) for v in (r["actual_table"] - r["pred_table"])],
            })

        backup = self.config_path + ".bak_before_pick_affine"
        if os.path.exists(self.config_path):
            with open(self.config_path, "r", encoding="utf-8") as f:
                old = f.read()
            with open(backup, "w", encoding="utf-8") as f:
                f.write(old)
            rospy.loginfo("已备份旧配置: %s", backup)

        with open(self.config_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

        rospy.loginfo("已写入二维仿射补偿到: %s", self.config_path)

    def run(self):
        if not self.wait_required_tf():
            raise RuntimeError("必要 TF 未就绪，请确认 handeye_publisher、table_tf_broadcaster、robot_state_publisher 已启动")

        self.wait_inputs()

        # 关键：此时机械臂还应在拍照/识别位置。冻结相机 TF。
        # 后续你移动机械臂到 P1、P2... 记录真实点，不会影响其它点的预测坐标。
        self.freeze_camera_tf_for_current_image()

        detections = self.parse_detections()
        selected = self.select_spread_points(detections)
        self.draw_debug(selected)

        print("\n" + "=" * 90)
        print("二维仿射补偿采样")
        print("请打开 rqt_image_view 查看: {}".format(self.debug_topic))
        print("图中会显示 P1..P{}。".format(len(selected)))
        print("按顺序手动移动吸盘中心到每个方块真实吸取点，按回车记录。")
        print("=" * 90 + "\n")

        rows = []
        for d in selected:
            cid = int(d["calib_id"])
            u, v = int(d["u"]), int(d["v"])

            pred_base, pred_table = self.pixel_to_pick_point_table(u, v)
            print("\n[P{}] shape={} pixel=({}, {}) angle={} deg".format(cid, d["shape"], u, v, d["ang"]))
            print("  预测 base : x={:.6f}, y={:.6f}, z={:.6f}".format(*pred_base))
            print("  预测 table: x={:.6f}, y={:.6f}, z={:.6f}".format(*pred_table))
            input("  请手动移动吸盘中心到该方块真实吸取点，然后按回车记录...")

            actual_table = self.get_eef_table_point()
            delta = actual_table - pred_table
            print("  实际 table: x={:.6f}, y={:.6f}, z={:.6f}".format(*actual_table))
            print("  delta     : dx={:.3f} mm, dy={:.3f} mm, dz={:.3f} mm".format(
                delta[0] * 1000, delta[1] * 1000, delta[2] * 1000
            ))

            rows.append({
                "id": cid,
                "shape": int(d["shape"]),
                "u": u,
                "v": v,
                "pred_base": pred_base,
                "pred_table": pred_table,
                "actual_table": actual_table,
            })

        corr = self.fit_affine(rows)
        print("\n" + "=" * 90)
        print("拟合完成：")
        print("  coeff_x:", ["{:.10f}".format(v) for v in corr["coeff_x"]])
        print("  coeff_y:", ["{:.10f}".format(v) for v in corr["coeff_y"]])
        print("  pixel_center:", ["{:.3f}".format(v) for v in corr["pixel_center"]])
        print("  RMS: {:.3f} mm".format(corr["rms_m"] * 1000.0))
        print("=" * 90)

        if input("是否写入 tetris_config.yaml？[Y/n]: ").strip().lower() not in ("n", "no"):
            self.save_config(corr, rows)
        else:
            print("未写入配置文件。")


if __name__ == "__main__":
    try:
        node = PickAffineCalibrator()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("pick affine calibration failed: %s", e)
        raise