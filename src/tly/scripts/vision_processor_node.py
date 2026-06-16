#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shape-template vision processor for the Tetris xArm project.

Main idea:
  - Do NOT classify blocks by color.
  - Tune lighting/camera so blocks are dark silhouettes on a bright background.
  - Segment dark foreground, extract contours, and classify each piece by matching against
    polyomino templates over many rotation angles.

Kept ROS interfaces:
  Publishes /vision/board_state:
    inventory[7] + board_state[140] + num_blocks + [shape,u,v,angle]*n
  Publishes /vision/debug_image

Shape IDs remain compatible with strategy_node.cpp:
  0 line, 1 square, 2 T, 3 L_left, 4 L_right, 5 Z_left, 6 Z_right
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf.transformations import quaternion_matrix, quaternion_from_euler


SHAPE_NAMES = {
    0: "line",
    1: "square",
    2: "T",
    3: "L_left",
    4: "L_right",
    5: "Z_left",
    6: "Z_right",
}

# Same geometry convention as the corrected strategy node.
# Coordinates are unit cells before normalization; x=col-like, y=row-like.
BASE_SHAPES = {
    0: [(0, 0), (1, 0), (2, 0), (3, 0)],
    1: [(0, 0), (1, 0), (0, 1), (1, 1)],
    2: [(0, 0), (1, 0), (2, 0), (1, 1)],
    3: [(1, 0), (1, 1), (1, 2), (0, 2)],  # L_left
    4: [(0, 0), (0, 1), (0, 2), (1, 2)],  # L_right
    5: [(0, 0), (1, 0), (1, 1), (2, 1)],
    6: [(1, 0), (2, 0), (0, 1), (1, 1)],
}

DRAW_COLORS = {
    0: (0, 0, 255),
    1: (0, 165, 255),
    2: (42, 42, 165),
    3: (255, 0, 255),
    4: (0, 255, 255),
    5: (255, 0, 0),
    6: (0, 255, 0),
}


class CameraModel(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.ready = False
        self.frame_id = ""
        self.K = None
        self.D = None
        self.P = None
        self.use_rectified_projection = True

    def update(self, msg: CameraInfo, prefer_rectified: bool = True):
        with self.lock:
            self.frame_id = msg.header.frame_id or self.frame_id
            self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.D, dtype=np.float64).reshape(-1, 1) if len(msg.D) else np.zeros((5, 1), dtype=np.float64)
            self.P = np.array(msg.P, dtype=np.float64).reshape(3, 4)
            self.use_rectified_projection = bool(prefer_rectified and abs(self.P[0, 0]) > 1e-9 and abs(self.P[1, 1]) > 1e-9)
            self.ready = True

    def intrinsics_for_equivalent_pixel(self):
        with self.lock:
            if not self.ready:
                return None
            if self.use_rectified_projection:
                return float(self.P[0, 0]), float(self.P[1, 1]), float(self.P[0, 2]), float(self.P[1, 2])
            return float(self.K[0, 0]), float(self.K[1, 1]), float(self.K[0, 2]), float(self.K[1, 2])

    def image_center(self, fallback_shape=None):
        intr = self.intrinsics_for_equivalent_pixel()
        if intr is not None:
            _, _, cx, cy = intr
            return float(cx), float(cy)
        if fallback_shape is not None:
            h, w = fallback_shape[:2]
            return w * 0.5, h * 0.5
        return 0.0, 0.0

    def pixel_to_normalized_ray(self, u: float, v: float, assume_image_rectified: bool):
        with self.lock:
            if not self.ready:
                return None
            if assume_image_rectified and self.use_rectified_projection:
                fx, fy, cx, cy = self.P[0, 0], self.P[1, 1], self.P[0, 2], self.P[1, 2]
                x = (float(u) - cx) / fx
                y = (float(v) - cy) / fy
            else:
                pt = np.array([[[float(u), float(v)]]], dtype=np.float64)
                undistorted = cv2.undistortPoints(pt, self.K, self.D, P=None)
                x = float(undistorted[0, 0, 0])
                y = float(undistorted[0, 0, 1])
            return np.array([x, y, 1.0], dtype=np.float64)


@dataclass
class Detection:
    shape_id: int
    name: str
    contour: np.ndarray
    area: float
    geom_px: tuple
    pick_px: tuple
    angle_deg: float
    score: float
    iou: float
    stamp: float
    geom_table: object = None
    pick_table: object = None


class BlockTrack(object):
    def __init__(self, track_id: int, det: Detection, history_len: int):
        self.track_id = track_id
        self.shape_id = det.shape_id
        self.name = det.name
        self.history = deque(maxlen=history_len)
        self.missed = 0
        self.update(det)

    def update(self, det: Detection):
        self.shape_id = det.shape_id
        self.name = det.name
        self.history.append(det)
        self.missed = 0

    def mark_missed(self):
        self.missed += 1

    def count(self):
        return len(self.history)

    @staticmethod
    def _median_pair(values):
        arr = np.array(values, dtype=np.float64)
        return float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))

    @staticmethod
    def _median_vec3(values):
        return np.median(np.array(values, dtype=np.float64), axis=0)

    @staticmethod
    def _mean_periodic_angle_180(degrees):
        if not degrees:
            return 0.0
        a2 = np.deg2rad(np.array(degrees, dtype=np.float64) * 2.0)
        s, c = np.mean(np.sin(a2)), np.mean(np.cos(a2))
        return float((0.5 * math.degrees(math.atan2(s, c))) % 180.0)

    def stable_px_geom(self):
        return self._median_pair([d.geom_px for d in self.history])

    def stable_px_pick(self):
        return self._median_pair([d.pick_px for d in self.history])

    def stable_table_pick(self):
        vals = [d.pick_table for d in self.history if d.pick_table is not None]
        if not vals:
            return None
        return self._median_vec3(vals)

    def stable_angle(self):
        return self._mean_periodic_angle_180([d.angle_deg for d in self.history])

    def stable_score(self):
        return float(np.median([d.score for d in self.history]))

    def position_std_px(self):
        if len(self.history) < 2:
            return float("inf")
        arr = np.array([d.pick_px for d in self.history], dtype=np.float64)
        return float(np.linalg.norm(np.std(arr, axis=0)))

    def angle_std_deg(self):
        if len(self.history) < 2:
            return float("inf")
        a2 = np.deg2rad(np.array([d.angle_deg for d in self.history], dtype=np.float64) * 2.0)
        mean = math.atan2(np.mean(np.sin(a2)), np.mean(np.cos(a2)))
        diff = np.angle(np.exp(1j * (a2 - mean)))
        return float(np.rad2deg(np.std(diff)) / 2.0)

    def is_stable(self, min_frames, max_px_std, max_angle_std_deg):
        return self.count() >= min_frames and self.missed == 0 and self.position_std_px() <= max_px_std and self.angle_std_deg() <= max_angle_std_deg


class TemplateBank(object):
    def __init__(self, canvas_size=96, angle_step_deg=5, refine_step_deg=1, cell_size=20, margin=14):
        self.canvas_size = int(canvas_size)
        self.angle_step_deg = int(angle_step_deg)
        self.refine_step_deg = int(refine_step_deg)
        self.cell_size = int(cell_size)
        self.margin = int(margin)
        self.coarse = []
        for sid in sorted(BASE_SHAPES.keys()):
            for angle in range(0, 180, self.angle_step_deg):
                self.coarse.append((sid, angle, self._render_shape(sid, angle)))

    def _render_shape(self, shape_id: int, angle_deg: float):
        raw = BASE_SHAPES[shape_id]
        min_x = min(p[0] for p in raw)
        min_y = min(p[1] for p in raw)
        pts = [(x - min_x, y - min_y) for x, y in raw]
        max_x = max(x for x, _ in pts) + 1
        max_y = max(y for _, y in pts) + 1
        w = max_x * self.cell_size + 2 * self.margin
        h = max_y * self.cell_size + 2 * self.margin
        base = np.zeros((h, w), dtype=np.uint8)
        for x, y in pts:
            x0 = self.margin + x * self.cell_size
            y0 = self.margin + y * self.cell_size
            cv2.rectangle(base, (x0, y0), (x0 + self.cell_size - 1, y0 + self.cell_size - 1), 255, -1)
        # Smooth tiny raster seams.
        base = cv2.morphologyEx(base, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        return self._rotate_crop_square(base, angle_deg)

    def _rotate_crop_square(self, img, angle_deg: float):
        h, w = img.shape[:2]
        diag = int(math.ceil(math.sqrt(h * h + w * w))) + 8
        canvas = np.zeros((diag, diag), dtype=np.uint8)
        y0 = (diag - h) // 2
        x0 = (diag - w) // 2
        canvas[y0:y0 + h, x0:x0 + w] = img
        m = cv2.getRotationMatrix2D((diag / 2.0, diag / 2.0), float(angle_deg), 1.0)
        rot = cv2.warpAffine(canvas, m, (diag, diag), flags=cv2.INTER_NEAREST, borderValue=0)
        ys, xs = np.where(rot > 0)
        if len(xs) == 0:
            return np.zeros((self.canvas_size, self.canvas_size), dtype=np.uint8)
        x1, x2 = np.min(xs), np.max(xs)
        y1, y2 = np.min(ys), np.max(ys)
        crop = rot[y1:y2 + 1, x1:x2 + 1]
        return normalize_binary_mask(crop, self.canvas_size)

    def match(self, mask_norm):
        best = None
        for sid, angle, tmpl in self.coarse:
            iou = binary_iou(mask_norm, tmpl)
            if best is None or iou > best[2]:
                best = (sid, angle, iou)
        sid, coarse_angle, _ = best
        # Refine angle around the best coarse match for that shape only.
        start = coarse_angle - self.angle_step_deg
        end = coarse_angle + self.angle_step_deg
        best_ref = best
        for angle in range(start, end + 1, self.refine_step_deg):
            a = angle % 180
            tmpl = self._render_shape(sid, a)
            iou = binary_iou(mask_norm, tmpl)
            if iou > best_ref[2]:
                best_ref = (sid, a, iou)
        sid, angle, iou = best_ref
        return sid, float(angle % 180), float(iou)


def binary_iou(a, b):
    aa = a > 0
    bb = b > 0
    inter = np.logical_and(aa, bb).sum()
    union = np.logical_or(aa, bb).sum()
    if union <= 0:
        return 0.0
    return float(inter) / float(union)


def normalize_binary_mask(mask, size=96):
    mask = (mask > 0).astype(np.uint8) * 255
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return np.zeros((size, size), dtype=np.uint8)
    x1, x2 = np.min(xs), np.max(xs)
    y1, y2 = np.min(ys), np.max(ys)
    crop = mask[y1:y2 + 1, x1:x2 + 1]
    h, w = crop.shape[:2]
    scale = min((size - 10) / float(max(w, 1)), (size - 10) / float(max(h, 1)))
    nw, nh = max(1, int(round(w * scale))), max(1, int(round(h * scale)))
    resized = cv2.resize(crop, (nw, nh), interpolation=cv2.INTER_NEAREST)
    out = np.zeros((size, size), dtype=np.uint8)
    y0 = (size - nh) // 2
    x0 = (size - nw) // 2
    out[y0:y0 + nh, x0:x0 + nw] = resized
    return out


class ShapeTemplateVisionNode(object):
    def __init__(self):
        rospy.init_node("vision_processor_node", anonymous=True)
        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_rect_color")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.assume_image_rectified = bool(rospy.get_param("~assume_image_rectified", True))
        self.table_frame = rospy.get_param("~table_frame", "table_frame")
        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.target_plane_z = rospy.get_param("~target_plane_z", rospy.get_param("/tetris/PICK_Z", 0.0))
        self.hover_z = rospy.get_param("~hover_z", rospy.get_param("/tetris/HOVER_Z", 0.10))

        # Foreground segmentation. Tune these after making blocks dark on a bright background.
        self.scale_percent = int(rospy.get_param("~scale_percent", 100))
        self.roi_x_min = int(rospy.get_param("~roi_x_min", 0))
        self.roi_y_min = int(rospy.get_param("~roi_y_min", 0))
        self.roi_x_max = int(rospy.get_param("~roi_x_max", 0))  # 0 means image width
        self.roi_y_max = int(rospy.get_param("~roi_y_max", 0))  # 0 means image height
        self.threshold_mode = rospy.get_param("~threshold_mode", "otsu")  # otsu/manual/adaptive
        self.manual_dark_threshold = int(rospy.get_param("~manual_dark_threshold", 105))
        self.adaptive_block_size = int(rospy.get_param("~adaptive_block_size", 51))
        self.adaptive_c = int(rospy.get_param("~adaptive_c", 7))
        self.blur_kernel = int(rospy.get_param("~blur_kernel", 5))
        self.close_kernel_size = int(rospy.get_param("~close_kernel", 3))
        self.open_kernel_size = int(rospy.get_param("~open_kernel", 3))
        self.min_area = float(rospy.get_param("~min_area", 900.0))
        self.max_area = float(rospy.get_param("~max_area", 50000.0))
        self.min_template_iou = float(rospy.get_param("~min_template_iou", 0.42))
        self.max_contour_fill_ratio = float(rospy.get_param("~max_contour_fill_ratio", 0.98))
        self.min_contour_fill_ratio = float(rospy.get_param("~min_contour_fill_ratio", 0.18))
        self.pick_point_mode = rospy.get_param("~pick_point_mode", "centroid")  # centroid/distance/hybrid
        self.distance_pick_shapes = set([
    int(x) for x in rospy.get_param("~distance_pick_shapes", [3, 4])
])
        self.template_size = int(rospy.get_param("~template_size", 96))
        self.template_angle_step = int(rospy.get_param("~template_angle_step", 5))
        self.template_refine_step = int(rospy.get_param("~template_refine_step", 1))
        self.templates = TemplateBank(self.template_size, self.template_angle_step, self.template_refine_step)

        # Tracking/stability.
        self.track_history_len = int(rospy.get_param("~track_history_len", 9))
        self.stable_min_frames = int(rospy.get_param("~stable_min_frames", 5))
        self.max_missed_frames = int(rospy.get_param("~max_missed_frames", 8))
        self.track_match_gate_px = float(rospy.get_param("~track_match_gate_px", 55.0))
        self.track_match_gate_m = float(rospy.get_param("~track_match_gate_m", 0.035))
        self.stable_max_px_std = float(rospy.get_param("~stable_max_px_std", 3.0))
        self.stable_max_angle_std_deg = float(rospy.get_param("~stable_max_angle_std_deg", 5.0))
        self.publish_only_stable = bool(rospy.get_param("~publish_only_stable", True))

        self.publish_debug_foreground = bool(rospy.get_param("~publish_debug_foreground", True))
        self.publish_debug_edges = bool(rospy.get_param("~publish_debug_edges", True))
        self.publish_debug_pose_array = bool(rospy.get_param("~publish_debug_pose_array", True))

        self.camera_model = CameraModel()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.latest_frame = None
        self.latest_header = None
        self.tracks = []
        self.next_track_id = 1

        self.state_pub = rospy.Publisher("/vision/board_state", Int32MultiArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/vision/debug_image", Image, queue_size=1)
        self.foreground_pub = rospy.Publisher("/vision/debug_foreground", Image, queue_size=1) if self.publish_debug_foreground else None
        self.edges_pub = rospy.Publisher("/vision/debug_edges", Image, queue_size=1) if self.publish_debug_edges else None
        self.pose_array_pub = rospy.Publisher("/vision/tracked_blocks_table", PoseArray, queue_size=1) if self.publish_debug_pose_array else None

        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

        rospy.loginfo(
            "Shape-template vision ready. image=%s camera_info=%s table_frame=%s plane_z=%.4f rectified=%s",
            self.image_topic, self.camera_info_topic, self.table_frame, self.target_plane_z, self.assume_image_rectified,
        )

    def camera_info_callback(self, msg):
        self.camera_model.update(msg, prefer_rectified=self.assume_image_rectified)

    @staticmethod
    def _find_contours(mask):
        res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return res[0] if len(res) == 2 else res[1]

    def _resize(self, img):
        if self.scale_percent == 100:
            return img.copy(), 1.0
        scale = self.scale_percent / 100.0
        w = max(1, int(round(img.shape[1] * scale)))
        h = max(1, int(round(img.shape[0] * scale)))
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA), scale

    def _roi_bounds(self, shape):
        h, w = shape[:2]
        x1 = max(0, self.roi_x_min)
        y1 = max(0, self.roi_y_min)
        x2 = w if self.roi_x_max <= 0 else min(w, self.roi_x_max)
        y2 = h if self.roi_y_max <= 0 else min(h, self.roi_y_max)
        return x1, y1, x2, y2

    def _foreground_mask(self, frame):
        small, scale = self._resize(frame)
        x1, y1, x2, y2 = self._roi_bounds(small.shape)
        roi = small[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if self.blur_kernel > 1:
            k = self.blur_kernel if self.blur_kernel % 2 == 1 else self.blur_kernel + 1
            gray = cv2.GaussianBlur(gray, (k, k), 0)
        if self.threshold_mode == "manual":
            _, mask_roi = cv2.threshold(gray, self.manual_dark_threshold, 255, cv2.THRESH_BINARY_INV)
        elif self.threshold_mode == "adaptive":
            bs = self.adaptive_block_size if self.adaptive_block_size % 2 == 1 else self.adaptive_block_size + 1
            bs = max(3, bs)
            mask_roi = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, bs, self.adaptive_c)
        else:
            # Otsu inverse: dark objects become white foreground.
            _, mask_roi = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        if self.close_kernel_size > 1:
            k = np.ones((self.close_kernel_size, self.close_kernel_size), np.uint8)
            mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, k)
        if self.open_kernel_size > 1:
            k = np.ones((self.open_kernel_size, self.open_kernel_size), np.uint8)
            mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, k)

        mask = np.zeros(small.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = mask_roi
        return small, mask, scale

    @staticmethod
    def _contour_centroid(cnt):
        """Area centroid from image moments. For uniform plastic blocks, this is the
        closest 2D image equivalent of the physical center of mass.
        """
        m = cv2.moments(cnt)
        if abs(m["m00"]) > 1e-6:
            return float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"])
        rect = cv2.minAreaRect(cnt)
        return float(rect[0][0]), float(rect[0][1])

    @staticmethod
    def _distance_transform_center(mask_shape, cnt):
        """Maximum inscribed-circle center. Useful for contact safety, but not
        equivalent to physical center of mass on L/Z/T shapes.
        """
        single = np.zeros(mask_shape, dtype=np.uint8)
        cv2.drawContours(single, [cnt], -1, 255, -1)
        dist = cv2.distanceTransform(single, cv2.DIST_L2, 5)
        _, _, _, loc = cv2.minMaxLoc(dist)
        return float(loc[0]), float(loc[1])

    def _pick_point_for_contour(self, cnt, shape_id, mask_shape):
        centroid = self._contour_centroid(cnt)
        if self.pick_point_mode == "distance":
            return centroid, self._distance_transform_center(mask_shape, cnt)
        if self.pick_point_mode == "hybrid" and int(shape_id) in self.distance_pick_shapes:
            return centroid, self._distance_transform_center(mask_shape, cnt)
        return centroid, centroid

    def _match_contour(self, mask, cnt):
        contour_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
        x, y, w, h = cv2.boundingRect(cnt)
        crop = contour_mask[y:y+h, x:x+w]
        norm = normalize_binary_mask(crop, self.template_size)
        sid, angle, iou = self.templates.match(norm)
        return sid, angle, iou

    def _detect_blocks(self, frame, stamp):
        small, fg, scale = self._foreground_mask(frame)
        detections = []
        inv = 1.0 / scale
        edges = cv2.Canny(fg, 50, 150)

        for cnt in self._find_contours(fg):
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            bbox_area = max(1.0, float(w * h))
            fill = area / bbox_area
            if fill < self.min_contour_fill_ratio or fill > self.max_contour_fill_ratio:
                continue
            if w < 10 or h < 10:
                continue

            sid, angle, iou = self._match_contour(fg, cnt)
            if iou < self.min_template_iou:
                continue

            geom_small, pick_small = self._pick_point_for_contour(cnt, sid, fg.shape)
            cnt_full = np.round(cnt.astype(np.float32) * inv).astype(np.int32)
            geom = (geom_small[0] * inv, geom_small[1] * inv)
            pick = (pick_small[0] * inv, pick_small[1] * inv)
            detections.append(Detection(
                shape_id=sid,
                name=SHAPE_NAMES[sid],
                contour=cnt_full,
                area=float(area * inv * inv),
                geom_px=geom,
                pick_px=pick,
                angle_deg=angle,
                score=iou,
                iou=iou,
                stamp=stamp,
            ))
        return detections, fg, edges

    def _lookup_camera_to_table(self, header):
        camera_frame = self.camera_frame_override or header.frame_id or self.camera_model.frame_id
        if not camera_frame:
            return None
        try:
            return self.tf_buffer.lookup_transform(self.table_frame, camera_frame, rospy.Time(0), rospy.Duration(0.05))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "TF lookup %s <- %s failed: %s", self.table_frame, camera_frame, str(exc))
            return None

    @staticmethod
    def _transform_rotation_matrix(tx):
        q = tx.transform.rotation
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

    @staticmethod
    def _transform_translation(tx):
        t = tx.transform.translation
        return np.array([t.x, t.y, t.z], dtype=np.float64)

    def _pixel_to_table_point(self, u, v, tx, plane_z=None):
        if plane_z is None:
            plane_z = self.target_plane_z
        ray_cam = self.camera_model.pixel_to_normalized_ray(u, v, self.assume_image_rectified)
        if ray_cam is None:
            return None
        R = self._transform_rotation_matrix(tx)
        cam_pos = self._transform_translation(tx)
        ray_table = R.dot(ray_cam)
        denom = float(ray_table[2])
        if abs(denom) < 1e-9:
            return None
        t = (float(plane_z) - float(cam_pos[2])) / denom
        if t < 0:
            return None
        return cam_pos + t * ray_table

    def _attach_table_points(self, detections, tx):
        for d in detections:
            d.geom_table = self._pixel_to_table_point(d.geom_px[0], d.geom_px[1], tx)
            d.pick_table = self._pixel_to_table_point(d.pick_px[0], d.pick_px[1], tx)

    @staticmethod
    def _px_distance(a, b):
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))

    @staticmethod
    def _table_distance(a, b):
        if a is None or b is None:
            return float("inf")
        return float(np.linalg.norm(np.array(a[:2]) - np.array(b[:2])))

    def _match_score(self, tr: BlockTrack, det: Detection):
        # Allow shape id to change while tracks warm up; penalize mismatches heavily.
        shape_penalty = 0.0 if tr.shape_id == det.shape_id else 1.5
        p = tr.stable_table_pick()
        if p is not None and det.pick_table is not None:
            d = self._table_distance(p, det.pick_table)
            if d <= self.track_match_gate_m:
                return d / max(self.track_match_gate_m, 1e-6) + shape_penalty
            return float("inf")
        dpx = self._px_distance(tr.stable_px_pick(), det.pick_px)
        if dpx <= self.track_match_gate_px:
            return dpx / max(self.track_match_gate_px, 1e-6) + shape_penalty
        return float("inf")

    def _update_tracks(self, detections):
        unmatched_t = set(range(len(self.tracks)))
        unmatched_d = set(range(len(detections)))
        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, det in enumerate(detections):
                s = self._match_score(tr, det)
                if math.isfinite(s):
                    pairs.append((s, ti, di))
        pairs.sort(key=lambda x: x[0])
        for _, ti, di in pairs:
            if ti not in unmatched_t or di not in unmatched_d:
                continue
            self.tracks[ti].update(detections[di])
            unmatched_t.remove(ti)
            unmatched_d.remove(di)
        for ti in unmatched_t:
            self.tracks[ti].mark_missed()
        for di in unmatched_d:
            self.tracks.append(BlockTrack(self.next_track_id, detections[di], self.track_history_len))
            self.next_track_id += 1
        self.tracks = [t for t in self.tracks if t.missed <= self.max_missed_frames]

    def _stable_tracks(self):
        return [t for t in self.tracks if t.is_stable(self.stable_min_frames, self.stable_max_px_std, self.stable_max_angle_std_deg)]

    def _tracks_for_publish(self):
        return self._stable_tracks() if self.publish_only_stable else [t for t in self.tracks if t.missed == 0]

    def _publish_board_state(self, tracks):
        inventory = [0] * 7
        board_state = [0] * 140
        payload = []
        for tr in tracks:
            sid = int(tr.shape_id)
            if sid < 0 or sid >= 7:
                continue
            inventory[sid] += 1
            u, v = tr.stable_px_geom()
            payload.extend([sid, int(round(u)), int(round(v)), int(round(tr.stable_angle()))])
        msg = Int32MultiArray()
        msg.data = inventory + board_state + [len(payload) // 4] + payload
        self.state_pub.publish(msg)

    def _publish_pose_array(self, tracks, header):
        if self.pose_array_pub is None:
            return
        arr = PoseArray()
        arr.header.stamp = header.stamp if header is not None else rospy.Time.now()
        arr.header.frame_id = self.table_frame
        for tr in tracks:
            p = tr.stable_table_pick()
            if p is None:
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            q = quaternion_from_euler(math.pi, 0.0, math.radians(tr.stable_angle()))
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            arr.poses.append(pose)
        self.pose_array_pub.publish(arr)

    def _draw_debug(self, frame, detections, tracks):
        out = frame.copy()
        for d in detections:
            cv2.drawContours(out, [d.contour], -1, DRAW_COLORS[d.shape_id], 1)
            cv2.circle(out, (int(round(d.geom_px[0])), int(round(d.geom_px[1]))), 3, (180, 180, 180), -1)
        for tr in tracks:
            u, v = tr.stable_px_geom()
            pu, pv = tr.stable_px_pick()
            stable = tr.is_stable(self.stable_min_frames, self.stable_max_px_std, self.stable_max_angle_std_deg)
            color = (0, 255, 0) if stable else (0, 200, 255)
            cv2.circle(out, (int(round(u)), int(round(v))), 5, color, -1)
            cv2.drawMarker(out, (int(round(pu)), int(round(pv))), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
            label = "#{} s{} {} a{} iou{:.2f}".format(
                tr.track_id, tr.shape_id, SHAPE_NAMES.get(tr.shape_id, "?"), int(round(tr.stable_angle())), tr.stable_score()
            )
            cv2.putText(out, label, (int(round(u)) + 8, int(round(v)) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        return out

    def image_callback(self, msg):
        if not self.camera_model.ready:
            rospy.logwarn_throttle(2.0, "Waiting for CameraInfo on %s", self.camera_info_topic)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr("CvBridge Error: %s", str(exc))
            return
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()
        detections, fg, edges = self._detect_blocks(frame, stamp)
        tx = self._lookup_camera_to_table(msg.header)
        if tx is not None:
            self._attach_table_points(detections, tx)

        with self.lock:
            self.latest_frame = frame.copy()
            self.latest_header = msg.header
            self._update_tracks(detections)
            publish_tracks = self._tracks_for_publish()
            all_tracks = list(self.tracks)

        self._publish_board_state(publish_tracks)
        self._publish_pose_array(publish_tracks, msg.header)
        debug = self._draw_debug(frame, detections, all_tracks)
        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            if self.foreground_pub is not None:
                self.foreground_pub.publish(self.bridge.cv2_to_imgmsg(fg, "mono8"))
            if self.edges_pub is not None:
                self.edges_pub.publish(self.bridge.cv2_to_imgmsg(edges, "mono8"))
        except CvBridgeError as exc:
            rospy.logerr("debug publish failed: %s", str(exc))


if __name__ == "__main__":
    try:
        ShapeTemplateVisionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass