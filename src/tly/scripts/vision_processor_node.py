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
            [(373, 54), (391, 53), (410, 52), (428, 52), (447, 51), (466, 51), (485, 50), (504, 50), (523, 49), (543, 49)],
            [(373, 72), (392, 71), (410, 71), (429, 70), (448, 70), (467, 69), (486, 69), (505, 68), (524, 68), (543, 67)],
            [(374, 90), (392, 90), (411, 89), (430, 89), (448, 88), (467, 88), (486, 87), (505, 87), (524, 87), (544, 86)],
            [(374, 109), (393, 108), (411, 108), (430, 107), (449, 107), (468, 107), (487, 106), (506, 106), (525, 105), (544, 105)],
            [(375, 127), (393, 127), (412, 126), (431, 126), (449, 126), (468, 125), (487, 125), (506, 124), (526, 124), (545, 124)],
            [(375, 146), (394, 145), (412, 145), (431, 145), (450, 144), (469, 144), (488, 143), (507, 143), (526, 143), (546, 142)],
            [(376, 164), (394, 164), (413, 163), (432, 163), (451, 163), (470, 162), (489, 162), (508, 162), (527, 162), (546, 161)],
            [(376, 182), (395, 182), (414, 182), (432, 182), (451, 181), (470, 181), (489, 181), (508, 181), (527, 180), (547, 180)],
            [(377, 201), (395, 201), (414, 200), (433, 200), (452, 200), (471, 200), (490, 200), (509, 199), (528, 199), (547, 199)],
            [(377, 219), (396, 219), (415, 219), (433, 219), (452, 219), (471, 219), (490, 218), (509, 218), (529, 218), (548, 218)],
            [(378, 238), (397, 238), (415, 238), (434, 238), (453, 237), (472, 237), (491, 237), (510, 237), (529, 237), (549, 237)],
            [(378, 256), (397, 256), (416, 256), (435, 256), (453, 256), (472, 256), (492, 256), (511, 256), (530, 256), (549, 256)],
            [(379, 275), (398, 275), (416, 275), (435, 275), (454, 275), (473, 275), (492, 275), (511, 275), (531, 275), (550, 275)],
            [(380, 294), (398, 294), (417, 294), (436, 294), (455, 294), (474, 294), (493, 294), (512, 294), (531, 294), (551, 294)],
        ]
        
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