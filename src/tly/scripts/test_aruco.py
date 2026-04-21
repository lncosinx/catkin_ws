#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_callback(msg):
    bridge = CvBridge()
    # 将 ROS 图像转换为 OpenCV 格式
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 准备常见的 ArUco 字典大全
    dicts = {
        "Original ArUco": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_4X4": cv2.aruco.DICT_4X4_50,
        "DICT_5X5": cv2.aruco.DICT_5X5_50,
        "DICT_7X7": cv2.aruco.DICT_7X7_50
    }

    found = False
    for name, dict_flag in dicts.items():
        try:
            # OpenCV 4.x 的标准调用
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_flag)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
            
            if ids is not None:
                rospy.loginfo(f"✅ 破案了！当前画面中的码 -> 字典: [{name}], 真实 ID: [{ids[0][0]}]")
                found = True
                break
        except Exception as e:
            continue
            
    if not found:
        rospy.loginfo_throttle(2, "❌ 画面中没检测到码，可能是反光或者距离太远...")

if __name__ == '__main__':
    rospy.init_node('test_aruco_scanner', anonymous=True)
    rospy.loginfo("正在启动 ArUco 扫描器，请确保相机正对着平板...")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()
    