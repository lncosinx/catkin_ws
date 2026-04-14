import cv2
import numpy as np

# =================================================================
# 1. 粘贴你的标定数据
# 将你刚才用 calibrate_board.py 生成的 BOARD_GRID_POINTS 粘贴在这里
# =================================================================
BOARD_GRID_POINTS = [
    [(405, 72), (422, 72), (439, 72), (457, 72), (475, 72), (492, 72), (510, 72), (528, 72), (546, 72), (564, 72)],
    [(405, 89), (422, 89), (440, 89), (457, 89), (475, 89), (493, 89), (510, 89), (528, 89), (546, 89), (564, 89)],
    [(405, 106), (422, 106), (440, 106), (458, 106), (475, 106), (493, 106), (511, 106), (529, 107), (546, 107), (564, 107)],
    [(405, 124), (423, 124), (440, 124), (458, 124), (476, 124), (493, 124), (511, 124), (529, 124), (547, 124), (565, 124)],
    [(405, 141), (423, 141), (440, 141), (458, 141), (476, 141), (494, 142), (511, 142), (529, 142), (547, 142), (565, 142)],
    [(405, 159), (423, 159), (441, 159), (458, 159), (476, 159), (494, 159), (512, 159), (530, 160), (548, 160), (566, 160)],
    [(405, 176), (423, 176), (441, 177), (459, 177), (476, 177), (494, 177), (512, 177), (530, 177), (548, 177), (566, 178)],
    [(406, 194), (423, 194), (441, 194), (459, 194), (477, 195), (495, 195), (513, 195), (531, 195), (549, 195), (567, 196)],
    [(406, 212), (423, 212), (441, 212), (459, 212), (477, 213), (495, 213), (513, 213), (531, 213), (549, 213), (567, 214)],
    [(406, 230), (424, 230), (441, 230), (459, 230), (477, 230), (495, 231), (513, 231), (531, 231), (549, 231), (568, 232)],
    [(406, 247), (424, 248), (442, 248), (460, 248), (478, 248), (496, 249), (514, 249), (532, 249), (550, 249), (568, 250)],
    [(406, 265), (424, 266), (442, 266), (460, 266), (478, 266), (496, 267), (514, 267), (532, 267), (550, 268), (569, 268)],
    [(406, 283), (424, 284), (442, 284), (460, 284), (478, 285), (496, 285), (514, 285), (533, 286), (551, 286), (569, 286)],
    [(407, 302), (424, 302), (442, 302), (460, 302), (478, 303), (497, 303), (515, 303), (533, 304), (551, 304), (570, 305)],
]

def process_vision(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: 找不到图像 {image_path}")
        return None
        
    scale_percent = 50 
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    img_resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    
    hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    result_img = img_resized.copy()
    
    color_ranges = {
        0: ("Red", np.array([175, 94, 92]), np.array([2, 255, 175]), (0, 0, 255)),
        1: ("Orange", np.array([3, 227, 109]), np.array([9, 255, 181]), (0, 165, 255)),
        2: ("Brown", np.array([3, 136, 48]), np.array([19, 244, 87]), (42, 42, 165)),
        3: ("Purple", np.array([129, 41, 79]), np.array([163, 107, 159]), (255, 0, 255)),
        4: ("Yellow", np.array([16, 202, 88]), np.array([23, 255, 179]), (0, 255, 255)),
        5: ("Blue", np.array([91, 171, 52]), np.array([111, 255, 98]), (255, 0, 0)),
        6: ("Green", np.array([64, 93, 33]), np.array([83, 255, 64]), (0, 255, 0))
    }

    inventory = [0] * 7
    board_state = [0] * 140
    # 将分界线定在右侧网格的最左边缘稍微往左一点 (比如 390)
    # 你也可以根据实际情况微调这个数值
    split_x = 390
    
    # 预先存储所有颜色的完整形态学掩码，供后续使用
    color_masks = {}

    # =================================================================
    # 2. 盘点左侧库存 (Inventory)
    # =================================================================
    for shape_id, (name, lower, upper, draw_color) in color_ranges.items():
        
        # 🌟 同步调参器的“跨界”解析逻辑
        if lower[0] <= upper[0]:
            # 常规情况：H_min <= H_max
            mask = cv2.inRange(hsv, lower, upper)
        else:
            # 跨界情况：H_min > H_max (例如红色 131 -> 0)
            # 拆分第一段：H_min 到 179
            lower1 = np.copy(lower)
            upper1 = np.copy(upper)
            upper1[0] = 179
            mask1 = cv2.inRange(hsv, lower1, upper1)
            
            # 拆分第二段：0 到 H_max
            lower2 = np.copy(lower)
            upper2 = np.copy(upper)
            lower2[0] = 0
            mask2 = cv2.inRange(hsv, lower2, upper2)
            
            # 合并掩码
            mask = cv2.bitwise_or(mask1, mask2)
            
        # 后续的形态学处理和原来的保持一致
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        color_masks[shape_id] = mask

        # 【核心拦截】：使用根据网格动态计算的 split_x
        split_x = 390 
        mask_left_only = mask.copy()
        mask_left_only[:, split_x:] = 0

        # 在左半边找轮廓并计数
        contours, _ = cv2.findContours(mask_left_only, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 500: # 面积阈值
                
                # 🌟 核心优化：多边形轮廓逼近
                # 0.02 是一个经验系数，越小拟合越精细，越大越趋近于简单的多边形
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                inventory[shape_id] += 1
                
                # 画出平滑后的几何轮廓（方便看效果）
                cv2.drawContours(result_img, [approx], -1, draw_color, 2)
                
                # 基于平滑后的轮廓计算重心，极其稳定
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # 强迫症福音：把重心点明确画出来，方便你校对机械臂的抓取点
                    cv2.circle(result_img, (cX, cY), 4, (255, 255, 255), -1)
                    # 文字稍微偏移一点，别挡住抓取点
                    cv2.putText(result_img, name, (cX - 20, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # =================================================================
    # 3. 读取右侧白板状态 (Board State)
    # =================================================================
    if BOARD_GRID_POINTS:
        # 将二维列表展平为 140 长度的一维列表
        flat_points = [pt for row in BOARD_GRID_POINTS for pt in row] if isinstance(BOARD_GRID_POINTS[0], list) else BOARD_GRID_POINTS
        
        for i, (cx, cy) in enumerate(flat_points):
            best_color_id = -1
            max_pixels = 0
            
            # 定义 5x5 的取色区域 (ROI)，注意防止越界
            y1, y2 = max(0, cy - 2), min(height, cy + 3)
            x1, x2 = max(0, cx - 2), min(width, cx + 3)

            # 在这个 5x5 区域内，检查哪种颜色的像素最多
            for shape_id in range(7):
                roi_mask = color_masks[shape_id][y1:y2, x1:x2]
                white_pixels = cv2.countNonZero(roi_mask)
                
                # 设定一个小门槛：至少要有 5 个同色像素，才认为该颜色命中了这个孔洞
                if white_pixels > max_pixels and white_pixels >= 5:
                    max_pixels = white_pixels
                    best_color_id = shape_id

            if best_color_id != -1:
                # 你的 C++ 策略节点代码中：有方块是 1-7 (所以需要 ID + 1)
                board_state[i] = best_color_id + 1
                draw_color = color_ranges[best_color_id][3]
                # 用实心大圆点标记被占用的格子
                cv2.circle(result_img, (cx, cy), 6, draw_color, -1)
            else:
                # 空格子保持默认 0，用绿色小点标记空底板
                cv2.circle(result_img, (cx, cy), 2, (0, 255, 0), -1)
    else:
        print("提示：请先填入 BOARD_GRID_POINTS 标定数据！")

    # =================================================================
    # 4. 组装数据并输出
    # =================================================================
    vision_data_array = inventory + board_state
    
    print("\n====== 最终输出状态 (发送给 ROS) ======")
    print(f"库存总数: {sum(inventory)}")
    print(f"网格总数: {len(board_state)} (应为 140)")
    print(f"最终数组长度: {len(vision_data_array)} (应为 147)")
    
    cv2.imshow("Final System Prototype", result_img)
    print("\n按任意键关闭窗口...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return vision_data_array

if __name__ == "__main__":
    final_array = process_vision("/root/catkin_ws/src/tly/images/14_Color.png")