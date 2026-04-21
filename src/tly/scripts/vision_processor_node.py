#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from tly.srv import GetPrecisePose, GetPrecisePoseResponse

class VisionProcessorNode:
    def __init__(self):
        rospy.init_node('vision_processor_node', anonymous=True)
        
        self.bridge = CvBridge()
       
        
        self.color_ranges = {
            0: ("Red", np.array([156, 63, 100]), np.array([0, 255, 201]), (0, 0, 255)),
            1: ("Orange", np.array([5, 165, 176]), np.array([19, 254, 208]), (0, 165, 255)),
            2: ("Brown", np.array([4, 67, 74]), np.array([13, 215, 135]), (42, 42, 165)),
            3: ("Purple", np.array([124, 38, 90]), np.array([140, 107, 193]), (255, 0, 255)),
            4: ("Yellow", np.array([22, 138, 140]), np.array([29, 255, 225]), (0, 255, 255)),
            5: ("Blue", np.array([92, 91, 57]), np.array([111, 255, 146]), (255, 0, 0)),
            6: ("Green", np.array([68, 81, 51]), np.array([84, 220, 99]), (0, 255, 0))
        }

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)
        self.state_pub = rospy.Publisher("/vision/board_state", Int32MultiArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/vision/debug_image", Image, queue_size=1)
        # 1. 缓存最新一帧图像，供 Service 随时调用
        self.latest_cv_image = None 
        
        # 2. 注册精调服务
        self.precise_srv = rospy.Service('/vision/get_precise_pose', GetPrecisePose, self.handle_precise_pose)

        rospy.loginfo("🚀 Vision Node (Gravity Center & Edge Angle Edition) Ready.")

    def image_callback(self, data):
        # 每次收到图像，先存下来
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        cv_image = self.latest_cv_image.copy()
        scale_percent = 50 
        width = int(cv_image.shape[1] * scale_percent / 100)
        height = int(cv_image.shape[0] * scale_percent / 100)
        img_resized = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_AREA)
        
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
        result_img = img_resized.copy()

        inventory = [0] * 7
        board_state = [0] * 140
        color_masks = {}
        blocks_info = [] 

        for shape_id, (name, lower, upper, draw_color) in self.color_ranges.items():
            if lower[0] <= upper[0]:
                mask = cv2.inRange(hsv, lower, upper)
            else:
                lower1, upper1 = np.copy(lower), np.copy(upper)
                upper1[0] = 179
                mask1 = cv2.inRange(hsv, lower1, upper1)
                lower2, upper2 = np.copy(lower), np.copy(upper)
                lower2[0] = 0
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
                
            # === 🌟 形态学动态调整：防粘连专属方案 ===
            # 1. 轻度闭运算 (5x5)：填平积木内部少量的台灯反光白点
            kernel_close = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
            
            # 2. 强力开运算 (7x7)：像刀一样切断边缘靠在一起的黄块和蓝块！
            # 如果跑完发现偶尔还有粘连，可以勇敢地把这里的 (7,7) 改成 (9,9)
            kernel_open = np.ones((7,7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
            # =================================================
            
            color_masks[shape_id] = mask

            # 直接使用完整的 mask 寻找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 250:
                    inventory[shape_id] += 1
                    
                    epsilon = 0.02 * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)
                    cv2.drawContours(result_img, [approx], -1, draw_color, 2)
                    
                    # === 🌟 角度修复：提取“最长物理边缘”作为绝对角度，彻底干掉 L 型和 Z 型的对角线歪斜角 ===
                    max_len = 0
                    best_angle = 0
                    for i in range(len(approx)):
                        p1 = approx[i][0]
                        p2 = approx[(i+1)%len(approx)][0]
                        dx_edge = p2[0] - p1[0]
                        dy_edge = p2[1] - p1[1]
                        length = math.hypot(dx_edge, dy_edge)
                        if length > max_len:
                            max_len = length
                            # cv2图像坐标系中，计算出的角度完美贴合真实物理直边
                            best_angle = np.degrees(math.atan2(dy_edge, dx_edge))
                    
                    true_angle = best_angle % 180
                    if true_angle >= 175 or true_angle <= 5: 
                        true_angle = 0  # 强迫症福音：对齐 0 度

                    # 作为参考系的几何中心 (不再作为抓取点)
                    rect = cv2.minAreaRect(cnt)
                    cX_geom, cY_geom = rect[0]

                    # === 🌟 抓取点修复：回滚为物理重心 (Moments)，稳固抓取 L 型 ===
                    M = cv2.moments(approx)
                    if M["m00"] != 0:
                        cX_grav = int(M["m10"] / M["m00"])
                        cY_grav = int(M["m01"] / M["m00"])
                        
                        dx = cX_grav - cX_geom
                        dy = cY_grav - cY_geom
                        
                        dist = math.hypot(dx, dy)
                        if dist > 3.0: 
                            cv2.arrowedLine(result_img, (int(cX_geom), int(cY_geom)), (cX_grav, cY_grav), (255, 255, 255), 2, tipLength=0.3)
                            grav_angle = np.degrees(math.atan2(dy, dx)) % 360
                            diff = abs(grav_angle - true_angle)
                            if 90 < diff < 270:
                                true_angle = (true_angle + 180) % 360
                                
                        # 【核心修正】：用物理重心覆盖掉几何中心，保证吸盘落在实体上
                        cX_pick, cY_pick = cX_grav, cY_grav
                    else:
                        cX_pick, cY_pick = int(cX_geom), int(cY_geom)

                    # 标出抓取点并打印对齐后的角度
                    cv2.circle(result_img, (cX_pick, cY_pick), 4, (255, 255, 255), -1)
                    debug_text = f"{name} {int(true_angle)}deg"
                    cv2.putText(result_img, debug_text, (cX_pick - 30, cY_pick - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    cX_orig = int(cX_pick * 100 / scale_percent)
                    cY_orig = int(cY_pick * 100 / scale_percent)
                    
                    blocks_info.extend([shape_id, cX_orig, cY_orig, int(true_angle)])

        vision_data_array = inventory + board_state + [len(blocks_info) // 4] + blocks_info
        msg = Int32MultiArray()
        msg.data = vision_data_array
        self.state_pub.publish(msg)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error during debug image publish: {e}")

    def handle_precise_pose(self, req):
        if self.latest_cv_image is None:
            rospy.logwarn("Precise Vision: No image received yet.")
            return GetPrecisePoseResponse(success=False, dx=0, dy=0, angle=0)
            
        img = self.latest_cv_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 画面中心点（即相机光心正下方）
        center_x, center_y = img.shape[1] // 2, img.shape[0] // 2
        
        # 提取请求的特定颜色的掩码
        shape_id = req.target_shape_type
        if shape_id not in self.color_ranges:
            return GetPrecisePoseResponse(success=False, dx=0, dy=0, angle=0)
            
        name, lower, upper, _ = self.color_ranges[shape_id]
        
        # 🌟 加入完整的 HSV 跨界拼接逻辑
        if lower[0] <= upper[0]:
            mask = cv2.inRange(hsv, lower, upper)
        else:
            lower1, upper1 = np.copy(lower), np.copy(upper)
            upper1[0] = 179
            mask1 = cv2.inRange(hsv, lower1, upper1)
            lower2, upper2 = np.copy(lower), np.copy(upper)
            lower2[0] = 0
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        
        # 同样进行形态学防粘连处理
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7,7), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        min_dist = float('inf')
        best_cx, best_cy = 0, 0
        
        # 寻找距离画面正中心最近的那个方块（即当前吸盘正下方的目标）
        for cnt in contours:
            if cv2.contourArea(cnt) > 250:
                # === 🌟 精调服务也必须使用距离变换，寻找最肉点！ ===
                single_mask = np.zeros(mask.shape, dtype=np.uint8)
                cv2.drawContours(single_mask, [cnt], -1, 255, -1)
                dist_transform = cv2.distanceTransform(single_mask, cv2.DIST_L2, 5)
                _, _, _, max_loc = cv2.minMaxLoc(dist_transform)
                
                cx, cy = max_loc # 这个就是我们吸盘该吸的精确物理中心
                
                dist = math.hypot(cx - center_x, cy - center_y)
                if dist < min_dist:
                    min_dist = dist
                    best_cnt = cnt
                    best_cx = cx
                    best_cy = cy
                        
        if best_cnt is None:
            rospy.logwarn(f"Precise Vision: Target {name} block not found near center!")
            return GetPrecisePoseResponse(success=False, dx=0, dy=0, angle=0)

        # 重新提取最长边绝对角度
        epsilon = 0.02 * cv2.arcLength(best_cnt, True)
        approx = cv2.approxPolyDP(best_cnt, epsilon, True)
        max_len = 0
        best_angle = 0
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i+1)%len(approx)][0]
            dx_edge = p2[0] - p1[0]
            dy_edge = p2[1] - p1[1]
            length = math.hypot(dx_edge, dy_edge)
            if length > max_len:
                max_len = length
                best_angle = np.degrees(math.atan2(dy_edge, dx_edge))
        true_angle = int(best_angle % 180)
        
        # 计算偏移量：方块重心位置 - 画面正中心位置
        dx = best_cx - center_x
        dy = best_cy - center_y
        
        rospy.loginfo(f" Precise Vision Triggered! Target: {name}, dx={dx}, dy={dy}, angle={true_angle}")
        return GetPrecisePoseResponse(success=True, dx=dx, dy=dy, angle=true_angle)

if __name__ == '__main__':
    try:
        VisionProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Vision Processor Node shut down.")