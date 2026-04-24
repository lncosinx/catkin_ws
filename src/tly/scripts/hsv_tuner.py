import cv2
import numpy as np

def nothing(x):
    pass

def main():
    image_path = "/root/catkin_ws/src/tly/images/5_Color.png"
    img = cv2.imread(image_path)
    if img is None:
        print("找不到图片！请检查路径。")
        return
        
    # 缩小尺寸方便在笔记本屏幕上和控制台并排显示
    img = cv2.resize(img, (800, 450))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 创建调参窗口
    cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('HSV Tuner', 500, 300)

    # 创建 6 个滑块 (H, S, V 的上下限)
    # H的范围是 0-179，S和V的范围是 0-255
    cv2.createTrackbar('H Min', 'HSV Tuner', 0, 179, nothing)
    cv2.createTrackbar('H Max', 'HSV Tuner', 179, 179, nothing)
    cv2.createTrackbar('S Min', 'HSV Tuner', 0, 255, nothing)
    cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'HSV Tuner', 0, 255, nothing)
    cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

    # 初始化滑块位置（这里以绿色为例，你可以随便调）
    cv2.setTrackbarPos('H Min', 'HSV Tuner', 35)
    cv2.setTrackbarPos('H Max', 'HSV Tuner', 85)
    cv2.setTrackbarPos('S Min', 'HSV Tuner', 40)
    cv2.setTrackbarPos('S Max', 'HSV Tuner', 255)
    cv2.setTrackbarPos('V Min', 'HSV Tuner', 40)
    cv2.setTrackbarPos('V Max', 'HSV Tuner', 255)

    print("🚀 调参器已启动！")
    print("操作指南：")
    print("1. 拖动滑块，直到右侧的 Mask 窗口里，你想要的方块变成纯白色，背景变成纯黑色。")
    print("2. 留意方块之间是否有黑色的缝隙，如果有，说明能被完美切开。")
    print("3. 按下 'P' 键可以在终端打印当前的 HSV 数组，方便你直接复制到主代码里。")
    print("4. 按下 'Q' 键退出。")

    while True:
        # 获取当前滑块的值
        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

        # 🌟 升级：处理 Hue 跨越 0 度的特殊情况（针对红色）🌟
        if h_min <= h_max:
            # 正常情况：提取单段区间
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            # 跨界情况：提取两段区间并合并 (例如 h_min=160, h_max=10)
            lower1 = np.array([h_min, s_min, v_min])
            upper1 = np.array([179, s_max, v_max])
            mask1 = cv2.inRange(hsv, lower1, upper1)

            lower2 = np.array([0, s_min, v_min])
            upper2 = np.array([h_max, s_max, v_max])
            mask2 = cv2.inRange(hsv, lower2, upper2)

            mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学处理
        kernel = np.ones((3, 3), np.uint8)
        mask_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_CLOSE, kernel)

        # 提取彩色结果
        result = cv2.bitwise_and(img, img, mask=mask_morph)

        # 🌟 核心升级：强行在结果图上写字 (HUD 面板) 🌟
        # 无论系统 GUI 怎么 Bug，画在图上的字绝对丢不了
        cv2.putText(result, f"H: {h_min} - {h_max}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f"S: {s_min} - {s_max}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(result, f"V: {v_min} - {v_max}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 显示
        cv2.imshow('Original Image', img)
        cv2.imshow('Mask (White is Target)', mask_morph)
        cv2.imshow('Color Result (with HUD)', result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            print("\n✅ 当前选中的 HSV 阈值：")
            print(f"np.array([{h_min}, {s_min}, {v_min}]), np.array([{h_max}, {s_max}, {v_max}])")
            print("-" * 30)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()