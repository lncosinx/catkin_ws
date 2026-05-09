import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 添加这一行：强制使用无头后端，彻底摆脱对 tkinter 的依赖
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def setup_chinese_font():
    """配置Matplotlib以支持中文显示，兼容Windows和Mac"""
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei']
    plt.rcParams['axes.unicode_minus'] = False

def create_base_l_shape(size=96):
    """生成一个完美的 L字形 模板 (类似于文档中的 紫色L字形左)"""
    img = np.zeros((size, size), dtype=np.uint8)
    # L字形参数: 3格高, 底部2格宽, 每格20像素
    # 竖条: x:[38, 58], y:[18, 78]
    cv2.rectangle(img, (38, 18), (58, 78), 255, -1)
    # 横条: x:[58, 78], y:[58, 78]
    cv2.rectangle(img, (58, 58), (78, 78), 255, -1)
    return img

def create_candidate_mask(template, size=96):
    """通过仿射变换和形态学操作，模拟从真实图像提取出来的带噪声候选掩膜"""
    cand = template.copy()
    
    # 1. 模拟现实中极微小的旋转和位移 (中心点48, 48，旋转3度，平移2像素)
    M = cv2.getRotationMatrix2D((48, 48), 4, 1.0)
    M[0, 2] += -2  # X轴微小偏移
    M[1, 2] += 3   # Y轴微小偏移
    cand = cv2.warpAffine(cand, M, (size, size))
    
    # 2. 模拟二值化后的边缘锯齿与噪声
    kernel = np.ones((3, 3), np.uint8)
    cand = cv2.erode(cand, kernel, iterations=1)
    
    # 增加一点随机边缘毛刺
    noise = np.random.randint(0, 2, (size, size), dtype=np.uint8) * 255
    noise_mask = cv2.Canny(cand, 100, 200)
    noise_mask = cv2.dilate(noise_mask, kernel, iterations=1)
    cand[noise_mask > 0] = cv2.bitwise_and(cand, noise)[noise_mask > 0]
    
    # 闭运算平滑一下，符合你们文档中的预处理步骤
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, kernel)
    return cand

def calculate_iou(mask1, mask2):
    """计算交并比"""
    intersection = cv2.bitwise_and(mask1, mask2)
    union = cv2.bitwise_or(mask1, mask2)
    i_area = np.sum(intersection > 0)
    u_area = np.sum(union > 0)
    return i_area / u_area if u_area > 0 else 0

def create_iou_diagram():
    setup_chinese_font()
    size = 96
    
    # 1. 生成图像数据
    tmpl_img = create_base_l_shape(size)
    cand_img = create_candidate_mask(tmpl_img, size)
    iou_score = calculate_iou(cand_img, tmpl_img)
    
    # 2. 生成中间的叠加对比图 (伪彩色)
    # 模板用红色表示，候选掩膜用绿色表示，重叠部分为黄色
    overlay = np.zeros((size, size, 3), dtype=np.uint8)
    overlay[:, :, 0] = tmpl_img       # Red channel -> Template
    overlay[:, :, 1] = cand_img       # Green channel -> Candidate
    # 背景填点浅灰色好看些
    bg_mask = cv2.bitwise_not(cv2.bitwise_or(tmpl_img, cand_img))
    overlay[bg_mask > 0] = [30, 30, 30]

    # 3. 开始使用 Matplotlib 绘图
    fig = plt.figure(figsize=(12, 4.5), facecolor='white')
    
    # 定义子图：左(候选)、中(交并比可视化)、右(模板)
    ax1 = plt.subplot(1, 3, 1)
    ax2 = plt.subplot(1, 3, 2)
    ax3 = plt.subplot(1, 3, 3)
    
    # 绘制左侧：候选掩膜
    ax1.imshow(cand_img, cmap='gray')
    ax1.set_title("候选归一化掩膜", fontsize=14, pad=15)
    ax1.axis('off')
    
    # 绘制右侧：标准模板
    ax3.imshow(tmpl_img, cmap='gray')
    ax3.set_title("得分最高的模板", fontsize=14, pad=15)
    ax3.axis('off')
    
    # 绘制中间：叠加与IoU
    ax2.imshow(overlay)
    ax2.set_title("交并比匹配过程", fontsize=14, pad=15)
    ax2.axis('off')
    
    # 在中间子图上添加图例和文字描述
    # 1. 将图例设为水平排列 (ncol=3)，放在图片正下方
    legend_elements = [
        patches.Patch(color=(0, 1, 0), label='候选区域'),
        patches.Patch(color=(1, 0, 0), label='模板区域'),
        patches.Patch(color=(1, 1, 0), label='交集重叠')
    ]
    ax2.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.02), 
               fontsize=11, frameon=False, ncol=3)
    
    # 2. 添加交并比公式与得分，使用相对坐标 (transAxes) 调整到图例的更下方
    iou_text = f"IoU = {iou_score:.3f}\n" \
               f"(交集面积 / 并集面积)"
    ax2.text(0.5, -0.18, iou_text, transform=ax2.transAxes, ha='center', va='top', 
             fontsize=13, color='black', bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray', boxstyle='round,pad=0.5'))

    # 画指示箭头 (从左右两边指向中间)
    fig.canvas.draw()
    
    # 调整布局
    plt.tight_layout()
    plt.subplots_adjust(wspace=0.3, bottom=0.3)  # 增加 bottom 留白以容纳图例和文本
    
    # 画大箭头（跨子图）
    arrow_props = dict(facecolor='gray', shrink=0.05, width=2, headwidth=8)
    fig.add_artist(patches.ConnectionPatch(xyA=(1.0, 0.5), xyB=(0.0, 0.5), 
                                           coordsA="axes fraction", coordsB="axes fraction",
                                           axesA=ax1, axesB=ax2,
                                           arrowstyle="->", lw=2, color='gray'))
    
    fig.add_artist(patches.ConnectionPatch(xyA=(0.0, 0.5), xyB=(1.0, 0.5), 
                                           coordsA="axes fraction", coordsB="axes fraction",
                                           axesA=ax3, axesB=ax2,
                                           arrowstyle="->", lw=2, color='gray'))

    # 4. 保存为高分辨率图片，供论文插入使用
    output_filename = "figure_3_3_iou_matching.png"
    plt.savefig(output_filename, dpi=300, bbox_inches='tight', transparent=False)
    print(f"✅ 生成成功！图片已保存为: {output_filename}")
    
    # 在本地运行时取消注释可以弹窗预览
    # plt.show()

if __name__ == "__main__":
    create_iou_diagram()