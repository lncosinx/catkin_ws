import cv2
import numpy as np

points = []

def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow("Calibration", img)
            if len(points) == 4:
                print("四个角点已记录，正在生成网格...")
                draw_grid()

def draw_grid():
    # 假设点击顺序：左上, 右上, 右下, 左下
    pts1 = np.float32(points)
    
    # 计算标定矩形的近似宽高
    width = max(np.linalg.norm(pts1[0] - pts1[1]), np.linalg.norm(pts1[2] - pts1[3]))
    height = max(np.linalg.norm(pts1[0] - pts1[3]), np.linalg.norm(pts1[1] - pts1[2]))
    
    pts2 = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    matrix = cv2.getPerspectiveTransform(pts2, pts1)
    
    grid_points = []
    # 生成 14 行 10 列的网格
    for row in range(14):
        for col in range(10):
            # 因为点击的是最外侧的孔，首尾孔的间距刚好是 9 等分和 13 等分
            x_norm = col * (width / 9.0)
            y_norm = row * (height / 13.0)
            
            # 透视映射回原图坐标
            p = np.array([[[x_norm, y_norm]]], dtype=np.float32)
            p_trans = cv2.perspectiveTransform(p, matrix)
            cx, cy = int(p_trans[0][0][0]), int(p_trans[0][0][1])
            
            grid_points.append((cx, cy))
            cv2.circle(img, (cx, cy), 3, (0, 255, 0), -1)
            
    cv2.imshow("Calibration", img)
    print("\n=== 请将以下坐标列表复制到你的主程序中 ===")
    print("BOARD_GRID_POINTS = [")
    for i in range(14):
        row_pts = grid_points[i*10 : (i+1)*10]
        print(f"    {row_pts},")
    print("]")
    print("\n按任意键退出...")

if __name__ == "__main__":
    img = cv2.imread("/root/catkin_ws/src/tly/images/14_Color.png") # 换成你的原图名字
    if img is None:
        print("找不到图片")
        exit()
        
    scale_percent = 50 
    w = int(img.shape[1] * scale_percent / 100)
    h = int(img.shape[0] * scale_percent / 100)
    img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    print("请按顺序点击最外侧的四个孔：1.左上 -> 2.右上 -> 3.右下 -> 4.左下")
    cv2.imshow("Calibration", img)
    cv2.setMouseCallback("Calibration", click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()