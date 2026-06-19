#!/usr/bin/env python3
"""ChArUco 手眼标定追踪节点。

替代 aruco_ros 的 single 标记检测：检测一整块 ChArUco 标定板（来自
calib.io，11x8 方格，方格 15mm，标记 11mm，DICT_4X4），用棋盘格亚像素
角点估计板位姿，然后把 camera_color_optical_frame -> charuco_board_frame
这条 TF 广播出去，供 easy_handeye 的 calibrate.launch 当作 marker frame 消费。

整块板的角点远多于单个 ArUco 标记，位姿更稳、抗遮挡，手眼标定精度更高。

订阅:
  ~image (default /camera/color/image_raw)          —— 原始彩色图(含畸变)
  ~camera_info (default /camera/color/camera_info)  —— 相机内参+畸变
发布:
  TF: camera_optical_frame -> charuco_board_frame
  ~debug_image (sensor_msgs/Image)                  —— 检测叠加可视化

注意: OpenCV 4.2 使用 legacy ChArUco 标记排布。若打印的 calib.io PDF 采用
OpenCV 4.6+ 新排布会检测不到 —— 用 ~save_board_image 导出本节点生成的板图，
和 calib.io 的 PDF 目视比对后再打印。
"""
import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped


def _resolve_dictionary(name):
    flag = getattr(cv2.aruco, name, None)
    if flag is None:
        rospy.logwarn("未知字典 %s, 回退到 DICT_4X4_50", name)
        flag = cv2.aruco.DICT_4X4_50
    return cv2.aruco.getPredefinedDictionary(flag)


class CharucoTracker(object):
    def __init__(self):
        # 板参数：calib.io 列(squaresX)=11, 行(squaresY)=8
        self.squares_x = rospy.get_param("~squares_x", 11)
        self.squares_y = rospy.get_param("~squares_y", 8)
        self.square_length = rospy.get_param("~square_length", 0.015)  # m
        self.marker_length = rospy.get_param("~marker_length", 0.011)  # m
        dict_name = rospy.get_param("~dictionary", "DICT_4X4_50")
        self.optical_frame = rospy.get_param(
            "~camera_optical_frame", "camera_color_optical_frame")
        self.board_frame = rospy.get_param("~board_frame", "charuco_board_frame")
        self.min_corners = rospy.get_param("~min_charuco_corners", 6)

        self.dictionary = _resolve_dictionary(dict_name)
        self.board = cv2.aruco.CharucoBoard_create(
            self.squares_x, self.squares_y,
            self.square_length, self.marker_length, self.dictionary)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # 可选：导出本节点生成的板图，便于和 calib.io PDF 目视核对排布是否一致
        save_path = rospy.get_param("~save_board_image", "")
        if save_path:
            # 用方格像素数渲染，保证清晰
            px = int(self.squares_x * 100)
            py = int(self.squares_y * 100)
            img = self.board.draw((px, py))
            cv2.imwrite(save_path, img)
            rospy.loginfo("已导出 ChArUco 板图到 %s (与 calib.io PDF 比对后再打印)",
                          save_path)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        rospy.Subscriber(rospy.get_param("~camera_info",
                                         "/camera/color/camera_info"),
                         CameraInfo, self._info_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image",
                                         "/camera/color/image_raw"),
                         Image, self._image_cb, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("ChArUco 追踪器就绪: %dx%d, square=%.3fm marker=%.3fm dict=%s",
                      self.squares_x, self.squares_y,
                      self.square_length, self.marker_length, dict_name)

    def _info_cb(self, msg):
        self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D, dtype=np.float64)

    def _image_cb(self, msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5.0, "等待 camera_info ...")
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)

        debug = img
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            n_ch, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board)

            if n_ch is not None and n_ch >= self.min_corners:
                cv2.aruco.drawDetectedCornersCharuco(
                    debug, ch_corners, ch_ids, (0, 0, 255))
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    ch_corners, ch_ids, self.board,
                    self.camera_matrix, self.dist_coeffs, None, None)
                if ok:
                    self._publish_tf(rvec, tvec, msg.header.stamp)
                    cv2.aruco.drawAxis(debug, self.camera_matrix,
                                       self.dist_coeffs, rvec, tvec,
                                       self.square_length * 3)
                else:
                    rospy.logwarn_throttle(2.0, "位姿估计失败")
            else:
                rospy.loginfo_throttle(
                    2.0, "ChArUco 角点不足: %s (<%d)", n_ch, self.min_corners)
        else:
            rospy.loginfo_throttle(2.0, "未检测到任何标记")

        if self.debug_pub.get_num_connections() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

    def _publish_tf(self, rvec, tvec, stamp):
        rmat, _ = cv2.Rodrigues(rvec)
        homo = np.eye(4)
        homo[:3, :3] = rmat
        quat = tft.quaternion_from_matrix(homo)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.optical_frame
        t.child_frame_id = self.board_frame
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("charuco_tracker")
    CharucoTracker()
    rospy.spin()
