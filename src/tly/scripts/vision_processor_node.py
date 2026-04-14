#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

class VisionProcessorNode:
    def __init__(self):
        rospy.init_node('vision_processor_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # --- 配置参数 ---
        self.board_grid_points = [
            [(411, 60), (428, 60), (446, 60), (464, 60), (482, 60), (499, 60), (517, 60), (535, 60), (553, 60), (571, 60)],
            [(411, 77), (429, 77), (446, 77), (464, 77), (482, 77), (500, 77), (518, 77), (536, 77), (554, 77), (571, 77)],
            [(411, 94), (429, 94), (447, 94), (465, 94), (483, 94), (501, 94), (519, 94), (537, 94), (554, 94), (572, 94)],
            [(411, 111), (429, 111), (447, 111), (465, 111), (483, 111), (501, 111), (519, 111), (537, 111), (555, 111), (573, 111)],
            [(412, 129), (430, 129), (448, 129), (466, 129), (484, 129), (502, 129), (520, 129), (538, 129), (556, 129), (574, 129)],
            [(412, 146), (430, 146), (448, 146), (466, 146), (485, 146), (503, 146), (521, 146), (539, 146), (557, 146), (575, 146)],
            [(412, 164), (431, 164), (449, 164), (467, 164), (485, 164), (503, 164), (522, 164), (540, 164), (558, 164), (576, 164)],
            [(413, 182), (431, 182), (449, 182), (468, 182), (486, 182), (504, 182), (522, 182), (541, 182), (559, 182), (577, 182)],
            [(413, 200), (431, 200), (450, 200), (468, 200), (486, 200), (505, 200), (523, 200), (542, 200), (560, 200), (578, 200)],
            [(413, 218), (432, 218), (450, 218), (469, 218), (487, 218), (506, 218), (524, 218), (542, 218), (561, 218), (579, 218)],
            [(414, 236), (432, 236), (451, 236), (469, 236), (488, 236), (506, 236), (525, 236), (543, 236), (562, 236), (580, 236)],
            [(414, 255), (432, 255), (451, 255), (470, 255), (488, 255), (507, 255), (526, 255), (544, 255), (563, 255), (581, 255)],
            [(414, 274), (433, 274), (452, 274), (470, 274), (489, 274), (508, 274), (526, 274), (545, 274), (564, 274), (582, 274)],
            [(415, 293), (433, 293), (452, 293), (471, 293), (490, 293), (508, 293), (527, 293), (546, 293), (565, 293), (584, 293)],
        ]
        
        self.color_ranges = {
            0: ("Red", np.array([175, 94, 92]), np.array([2, 255, 175]), (0, 0, 255)),
            1: ("Orange", np.array([3, 227, 109]), np.array([9, 255, 181]), (0, 165, 255)),
            2: ("Brown", np.array([3, 136, 48]), np.array([19, 244, 87]), (42, 42, 165)),
            3: ("Purple", np.array([129, 41, 79]), np.array([163, 107, 159]), (255, 0, 255)),
            4: ("Yellow", np.array([16, 202, 88]), np.array([23, 255, 179]), (0, 255, 255)),
            5: ("Blue", np.array([91, 171, 52]), np.array([111, 255, 98]), (255, 0, 0)),
            6: ("Green", np.array([64, 93, 33]), np.array([83, 255, 64]), (0, 255, 0))
        }

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=1)
        self.state_pub = rospy.Publisher("/vision/board_state", Int32MultiArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/vision/debug_image", Image, queue_size=1)

        rospy.loginfo("🚀 Vision Node (Gravity Center & Edge Angle Edition) Ready.")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

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

            split_x = 390 
            mask_left_only = mask.copy()
            mask_left_only[:, split_x:] = 0

            contours, _ = cv2.findContours(mask_left_only, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

        # 读取右侧白板状态
        flat_points = [pt for row in self.board_grid_points for pt in row] if isinstance(self.board_grid_points[0], list) else self.board_grid_points
        
        for i, (cx, cy) in enumerate(flat_points):
            best_color_id = -1
            max_pixels = 0
            y1, y2 = max(0, cy - 2), min(height, cy + 3)
            x1, x2 = max(0, cx - 2), min(width, cx + 3)

            for shape_id in range(7):
                roi_mask = color_masks[shape_id][y1:y2, x1:x2]
                white_pixels = cv2.countNonZero(roi_mask)
                if white_pixels > max_pixels and white_pixels >= 5:
                    max_pixels = white_pixels
                    best_color_id = shape_id

            if best_color_id != -1:
                board_state[i] = best_color_id + 1
                draw_color = self.color_ranges[best_color_id][3]
                cv2.circle(result_img, (cx, cy), 6, draw_color, -1)
            else:
                cv2.circle(result_img, (cx, cy), 2, (0, 255, 0), -1)

        vision_data_array = inventory + board_state + [len(blocks_info) // 4] + blocks_info
        msg = Int32MultiArray()
        msg.data = vision_data_array
        self.state_pub.publish(msg)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error during debug image publish: {e}")

if __name__ == '__main__':
    try:
        VisionProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Vision Processor Node shut down.")