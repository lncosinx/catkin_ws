# 视觉感知子系统技术文档（vision_processor_node）

本文档从**数据处理**角度说明 `lucky` 视觉节点：输入一帧彩色图，经过哪些步骤、每
步用什么方法、方法的**数学原理与公式**是什么，最终输出库存计数与逐块抓取信息给策
略节点。

参考代码：
- DexiNed 实现（生产默认）：[vision_processor_node_dexined.cpp](../src/lucky/src/vision_processor_node_dexined.cpp)
- 经典实现：[vision_processor_node.cpp](../src/lucky/src/vision_processor_node.cpp)
- 共享深度采样器：[include/lucky/depth_sampler.hpp](../src/lucky/include/lucky/depth_sampler.hpp)
- 启动：[test_vision.launch](../src/lucky/launch/test_vision.launch)

> 检测对象是**彩色**多联骨牌（红/橙/棕/紫/黄/蓝/绿），不是黑色。

> 输入图像：节点参数 `image_topic` 代码默认 `/camera/color/image_rect_color`（`image_proc` 输出），但
> [lucky.launch](../src/lucky/launch/lucky.launch) 与 [test_vision.launch](../src/lucky/launch/test_vision.launch)
> 都显式喂 `/camera/color/image_raw` 并设 `assume_image_rectified=true`（即把原图当作已去畸变处理；
> [test_vision.launch](../src/lucky/launch/test_vision.launch) 里 `image_proc` 已注释掉）。

---

## 0. 数据流总览

```
彩色图 I(x,y)∈ℝ^{H×W×3}
   │
   ▼ [1] 预处理          缩放 / 灰度 / 高斯模糊 / EMA / 发光板掩膜
   ▼ [2] 前景分割        DexiNed 边缘 + Lab 颜色边界切割 → 二值前景 M_fg
   ▼ [3] 轮廓提取        findContours + 面积/填充率过滤 → {C_i}
   ▼ [4] 模板匹配分类    归一化 + IoU/Chamfer/签名/网格 加权 → (shape, angle, score)
   ▼ [5] 抓取点+朝向     质心/距离变换 + 主轴/有向轴
   ▼ [5.5] 同帧 NMS      质心去重
   ▼ [6] 跨帧跟踪/稳定   循环均值 + 抖动标准差门控
   ▼ [7] 深度采样        对齐深度中值 + 反投影（debug 通路）
   ▼ 发布 /vision/board_state
```

两套实现（DexiNed / 经典）**只有步骤 2 不同**，其余共用。生产用 DexiNed 版，下文以它
为主，经典分割在 §2.4 单列。

形状 ID 契约（跨节点共享）：`0` 一字、`1` 田、`2` T、`3` L 左、`4` L 右、`5` Z 左、
`6` Z 右。

---

## 1. 步骤 1 — 预处理

### 1.1 缩放
按 `scale_percent` 用 `INTER_AREA`（区域平均）下采样，缩放因子 $s=\text{scale\_percent}/100$。
区域插值输出像素是源区域的面积加权均值，等价于抗混叠的降采样：

$$I'(x,y)=\frac{1}{|R|}\sum_{(p,q)\in R}I(p,q)$$

其中 $R$ 是目标像素反投影回源图覆盖的矩形区域。之后按 `roi_*` 裁出感兴趣区，避开画
面边缘干扰。所有像素坐标最后乘 $s^{-1}$ 还原到原图尺度。

### 1.2 灰度化
`COLOR_BGR2GRAY` 按 ITU-R BT.601 亮度加权：

$$Y = 0.299\,R + 0.587\,G + 0.114\,B$$

### 1.3 高斯模糊
用 `blur_kernel`（强制奇数 $k$）做高斯卷积 $I_b = I * G_\sigma$，二维高斯核

$$G_\sigma(x,y)=\frac{1}{2\pi\sigma^2}\exp\!\left(-\frac{x^2+y^2}{2\sigma^2}\right)$$

当传入 $\sigma=0$，OpenCV 由核宽推算 $\sigma = 0.3\big((k-1)\cdot 0.5 - 1\big)+0.8$。作用是压
高频传感器噪声，稳定后续阈值/边缘。

### 1.4 时间平均（EMA，仅 DexiNed 版）
静止料堆下对连续帧做指数滑动平均，抑制背光闪烁与传感器噪声：

$$\bar I_t = (1-\alpha)\,\bar I_{t-1} + \alpha\,I_t,\qquad \alpha=\texttt{temporal\_alpha}\in(0,1)$$

$\alpha$ 越小越稳但越滞后（等效时间常数 $\tau\approx 1/\alpha$ 帧）；$\alpha=0$ 关闭。

### 1.5 发光板掩膜
只在背光板区域内检测，挡掉板外反光/手/边框。方法：对灰度图做 **Otsu 二值化** →
形态学闭+开 → 取**最大连通域** → 腐蚀收边。

**Otsu 阈值**（类间方差最大化）：设灰度直方图归一化为概率 $p(i)$，阈值 $t$ 把像素分
成两类，类概率与类均值

$$\omega_0(t)=\sum_{i\le t}p(i),\quad \omega_1(t)=1-\omega_0(t),\quad
\mu_k(t)=\frac{1}{\omega_k}\sum_{i\in \text{class }k} i\,p(i)$$

Otsu 取使类间方差最大的 $t^*$：

$$t^*=\arg\max_t\ \sigma_B^2(t),\qquad \sigma_B^2(t)=\omega_0(t)\,\omega_1(t)\,\big(\mu_0(t)-\mu_1(t)\big)^2$$

**形态学**闭运算 $I\bullet B=(I\oplus B)\ominus B$ 填小洞、开运算 $I\circ B=(I\ominus
B)\oplus B$ 去小刺，其中膨胀 $\oplus$、腐蚀 $\ominus$ 为

$$(I\oplus B)(x)=\max_{b\in B}I(x-b),\qquad (I\ominus B)(x)=\min_{b\in B}I(x-b)$$

**最大连通域**：8-邻接连通标注后保留面积占比 $\ge$ `lightboard_min_area_ratio` 的最
大分量，得到干净的发光板 ROI 掩膜 $M_\text{board}$。

---

## 2. 步骤 2 — 前景分割（DexiNed 版）

目标：从 ROI 里分出**彼此分开**的方块前景 $M_\text{fg}$。难点是贴碰在一起的异色块要切
开、块内眩光/掉漆不能误切。方法是「暗/彩前景」**减去**「Lab 颜色边界切割线」。

### 2.1 DexiNed 神经网络边缘
DexiNed 是一个全卷积边缘检测 CNN（ONNX 模型 [dexined.onnx](../src/lucky/module/dexined.onnx)，`use_dexined` 开启时
加载并强制设 CUDA 后端；加载/配置抛异常则关闭 DexiNed、退回经典分割）。预处理：ROI 尺寸 pad 到
16 的倍数，扣 ImageNet BGR 均值 $\mu=(103.939,116.779,123.68)$ 构造网络输入张量

$$X = I_\text{roi} - \mu$$

前向 $Y=f_\text{DexiNed}(X)$ 得边缘响应图，裁回 ROI 尺寸后**min–max 归一化**到 $[0,255]$：

$$E(x,y)=255\cdot\frac{Y(x,y)-\min Y}{\max Y-\min Y}$$

DexiNed 边逐帧会抖、且块内眩光处会产生假边，故**不直接**作主切割线（见 2.3）。

### 2.2 前景掩膜（暗块 ∪ 彩色块）
兼顾黑块与彩色块：

- 暗块：灰度 Otsu 反二值 $M_\text{dark}=[\,\text{gray} < t^*_\text{otsu}\,]$。
- 彩色块：转 HSV，取高饱和高明度像素 $M_\text{color}=[\,S>S_\text{min}\,]\wedge[\,V>V_\text{min}\,]$，
  开运算去噪。

$$M_\text{obj}=M_\text{dark}\ \cup\ M_\text{color}$$

BGR→HSV：$V=\max(R,G,B)$，$S=(V-\min(R,G,B))/V$（$V>0$），色相 $H$ 由主导通道定义。

### 2.3 Lab 颜色边界切割（核心）
把 BGR 转 **CIE Lab**（$L$ 亮度、$a$ 绿-红、$b$ 蓝-黄色度）。对**色度通道** $a,b$ 各做
**形态学梯度**取边缘强度，再取两者较大者作颜色边界：

$$g_a=(a\oplus B_3)-(a\ominus B_3),\quad g_b=(b\oplus B_3)-(b\ominus B_3)$$
$$E_\text{color}=\big[\ \max(g_a,g_b) > \texttt{color\_edge\_thresh}\ \big]$$

**为什么用 $a,b$ 而非亮度**：块内均匀色 → $a,b$ 无梯度 → 不内部过切；眩光只改亮度
$L$、不动 $a,b$ → 对眩光免疫；异色块交界色度突变 → 强梯度 → 切开。这正是它比逐帧抖
动的 DexiNed 边稳的原因。

- **同色贴碰补切**（`use_dexined_for_split`）：对同色相邻块（$a,b$ 无边界），或上高阈值
  DexiNed 强边 $[E>\texttt{dexined\_thresh}]$。
- **掉漆滤除**：对 $E_\text{color}$ 做连通域分析，只保留面积 $\ge$ `color_edge_min_area`
  的分量——真边界是成片长线，掉漆是孤立小斑点，按面积一筛即分开。
- 膨胀 $E_\text{color}$ 一圈（保证 8-连通切透），最后**相减**得前景：

$$M_\text{fg}=\big(M_\text{obj}\setminus (E_\text{color}\oplus B_3)\big)\circ B_\text{open}\ \cap\ M_\text{board}$$

### 2.4 经典分割（[vision_processor_node.cpp](../src/lucky/src/vision_processor_node.cpp)）
不用 CNN，直接对灰度做阈值前景：`threshold_mode` = `otsu`（§1.5 公式）/ `adaptive`
（高斯自适应阈值 $T(x,y)=\text{gauss-mean}_{31\times31}(x,y)-7$）/ 手动常数；并上饱和度彩
色前景，与发光板掩膜相交，形态学闭/开，Canny 出边缘。适合纯黑块、无 GPU 场景。

---

## 3. 步骤 3 — 轮廓提取与过滤

对 $M_\text{fg}$ 用 `findContours`（Suzuki–Abe 边界跟踪，`RETR_EXTERNAL` 只取外轮廓）。
每个轮廓 $C$ 按几何量过滤：

- **面积**（Green 公式，多边形有向面积）：
  $$A(C)=\tfrac12\Big|\sum_i \big(x_i\,y_{i+1}-x_{i+1}\,y_i\big)\Big|,\qquad \texttt{min\_area}\le A\le\texttt{max\_area}$$
- **bbox 填充率**：$\text{fill}=A/(w_\text{bb}\,h_\text{bb})$，要求 $\texttt{min\_fill}\le\text{fill}\le\texttt{max\_fill}$
  （滤掉细长噪声与粘连大 blob）。
- 最小宽高 $\ge 10$ px。

---

## 4. 步骤 4 — 模板匹配分类

为 7 形状 × 360° 预渲染二值模板；每个候选轮廓归一化后与模板库打分，取最高分。

### 4.1 归一化
`normalize_binary_mask`：裁到 bbox，等比缩放使长边填满 $(\text{size}-10)$ 像素，居中贴到
$\text{size}\times\text{size}$ 画布。缩放因子

$$\rho=\min\!\Big(\frac{\text{size}-10}{w_\text{bb}},\ \frac{\text{size}-10}{h_\text{bb}}\Big)$$

保证平移/尺度不变，只剩形状与旋转差异。匹配前可用 `approxPolyDP`（Douglas–Peucker，
容差 $\varepsilon=\texttt{eps\_ratio}\cdot\text{arcLength}$）拉直分割波纹边。

### 4.2 打分项（`topology_score`）
候选掩膜 $A$ 与模板掩膜 $T$ 的综合相似度由多项加权：

**① 硬 IoU / 软 IoU**（交并比，软 = 各膨胀 3×3 后）：
$$\text{IoU}(A,T)=\frac{|A\cap T|}{|A\cup T|}$$

**② Chamfer 双向距离**：用距离变换 $D_T(p)=\min_{q\in T}\lVert p-q\rVert$（`distanceTransform`,
DIST_L2 欧氏），对称平均候选→模板、模板→候选：
$$d=\tfrac12\Big(\underbrace{\tfrac{1}{|A|}\!\sum_{p\in A}\!D_T(p)}_{d_{ct}}+\underbrace{\tfrac{1}{|T|}\!\sum_{p\in T}\!D_A(p)}_{d_{tc}}\Big),\qquad
s_\text{chamfer}=e^{-d/3.5}$$

**③ 网格占用签名相似度**：把掩膜分成 $g\times g$ 格，签名 $\text{sig}[k]=(\text{格 }k\text{ 内前景像素数})/(\text{总前景})$，
用 $L_1$ 距离转相似度（$4\times4$ 与 $8\times8$ 两套）：
$$s_\text{sig}=\max\!\Big(0,\ 1-\tfrac12\lVert \text{sig}_A-\text{sig}_T\rVert_1\Big)$$

**④ L 形逐格占用分**（仅 sid∈{3,4}）：把候选反旋 $-\theta$ 摆正，按形状的 $g_w\times g_h$
理论占用格逐格核对，占用格看 $\text{ratio}/0.22$、空格看 $1-\text{ratio}/0.12$，平均。

**加权综合**（权重按形状定制，L 重逐格分、T 与其他各一套）：

$$\text{score}=\begin{cases}
0.12\,\text{IoU}_h+0.08\,\text{IoU}_s+0.08\,s_\text{ch}+0.17\,s_\text{sig4}+0.25\,s_\text{sig8}+0.30\,s_\text{cell} & \text{L 左/右}\\[2pt]
0.24\,\text{IoU}_h+0.18\,\text{IoU}_s+0.18\,s_\text{ch}+0.20\,s_\text{sig4}+0.20\,s_\text{sig8} & \text{T}\\[2pt]
0.30\,\text{IoU}_h+0.25\,\text{IoU}_s+0.20\,s_\text{ch}+0.15\,s_\text{sig4}+0.10\,s_\text{sig8} & \text{其他}
\end{cases}$$

### 4.3 粗扫 + 细化
先对所有 `coarse_templates`（步进 `angle_step`）取最高分角 $\hat\theta$，再在
$[\hat\theta-\texttt{angle\_step},\ \hat\theta+\texttt{angle\_step}]$ 内以 `refine_step`
逐度细化。返回 $(\text{shape},\ \theta\bmod 360,\ \text{score})$；$\text{score}<\texttt{min\_template\_iou}$ 丢弃。

---

## 5. 步骤 5 — 抓取点与朝向

### 5.1 抓取点
`pick_point_mode` 三选一：

- **质心** `centroid`：图像矩 $m_{pq}=\sum_{x,y}x^p y^q\,\mathbf 1_C(x,y)$，
  $$\bar x=m_{10}/m_{00},\quad \bar y=m_{01}/m_{00}$$
- **距离变换内点** `distance`：填充掩膜的距离变换极大点
  $$p^*=\arg\max_{p\in C} D_{\partial C}(p),\qquad D_{\partial C}(p)=\min_{q\notin C}\lVert p-q\rVert$$
  即**最大内切圆圆心**，远离边缘，吸盘最稳。
- **hybrid**：仅对 `distance_pick_shapes`（默认 L 左/右）用距离变换，其余用质心。
- Z/S 形（sid∈{5,6}）几何中心用 `minAreaRect` 的中心（质心可能落在缺口外）。

### 5.2 朝向
模板角给基础朝向；可选 `use_contour_axis_yaw` 用轮廓主轴细化：

- **minAreaRect 主轴**：最小外接矩形的长边方向。
- **Hough 直线**：边缘图霍夫变换，直线参数化 $\rho=x\cos\theta+y\sin\theta$，累加器峰值定
  主方向。
- **有向轴**（T/L/Z 各一个 `directed_*` 函数）：主轴只给 0–180° 的无向角，T/L/Z 需区分
  正反 180°——用质心相对轮廓质量分布的偏置（如 T 的凸出臂、L 的拐角、Z 的手性）把角
  唯一定到 0–360°。

`publish_angle_in_table_frame` 时把图像角经相机→board 的 TF 投影成板面偏航再发布（默
认关，board_state 不依赖它）。

---

## 5.5 步骤 5.5 — 同帧 NMS（仅 DexiNed 版）

分割过切会让一个块出多个框。按 score 降序贪心：新框若质心与任一已保留框的平方距离
小于门限，判为同块抑制：

$$(\Delta x)^2+(\Delta y)^2 < \texttt{nms\_center\_dist\_px}^2\ \Rightarrow\ \text{抑制}$$

---

## 6. 步骤 6 — 跨帧跟踪与稳定

把检测按（像素/桌面距离门 + 同形状）关联到历史轨迹（最近匹配），保留
`track_history_len` 帧历史，只有**稳定**轨迹才发布。

### 6.1 角度的循环均值/标准差
角度是周期量（田周期 90°、一字/Z 180°、其余 360°），不能算术平均。设周期 $P$、
$k=360/P$，把角度映到单位圆求**循环均值**：

$$\bar\theta=\frac1k\cdot\operatorname{atan2}\!\Big(\sum_i\sin(k\theta_i),\ \sum_i\cos(k\theta_i)\Big)$$

**循环标准差**（对每个样本取到均值的圆周最短角差 $d_i=\operatorname{atan2}(\sin,\cos)$）：

$$\sigma_\theta=\frac1k\sqrt{\frac1N\sum_i d_i^2}$$

### 6.2 位置标准差
$$\sigma_\text{px}=\sqrt{\frac1N\Big(\sum_i(x_i-\bar x)^2+\sum_i(y_i-\bar y)^2\Big)}$$

### 6.3 稳定判据
$$\text{stable}\iff \text{count}\ge\texttt{stable\_min\_frames}\ \wedge\ \text{missed}\le\texttt{allowed}\ \wedge\ \sigma_\text{px}\le\texttt{max\_px\_std}\ \wedge\ \sigma_\theta\le\texttt{max\_angle\_std}$$

`stable_publish_max_missed` 容忍已稳定的边缘块偶尔漏检仍保持发布（稳住库存计数），
超 `max_missed_frames` 才彻底删轨迹。

---

## 7. 步骤 7 — 深度抓取 Z（debug 通路）

`use_depth_pick_z` 开启时，订阅对齐深度（16UC1，毫米），在抓取点邻域取深度：

- **rect→raw 映射**：检测像素是去畸变（rect）坐标，对齐深度在原始彩色坐标。先反投影
  归一化 $(x,y)=((u-c_x)/f_x,(v-c_y)/f_y)$，再经 `projectPoints`（用畸变系数 $D$）投回
  raw 像素。
- **邻域中值**：半径 `depth_sample_radius_px` 方形窗内所有非零深度取中值（抗空洞/噪声），
  $Z=\operatorname{median}\{d>0\}/1000$（米）。
- **反投影到 board 系**：相机系点 $p_\text{cam}=Z\cdot(x,y,1)$，再 $p_\text{table}=R\,p_\text{cam}+T$。

目前仅打 log + 发 `/vision/pick_depth_debug`，**尚未写入 board_state**（Z 的生产解算在
path_planner，见 [path_plan.md](path_plan.md)）。

---

## 8. 输出契约

| 话题 | 类型 | 内容 |
|------|------|------|
| `/vision/board_state` | `Int32MultiArray` | **主输出**，见下 |
| `/vision/tracked_blocks_table` | `PoseArray` | 稳定块位姿（debug） |
| `/vision/debug_image` / `debug_foreground` / `debug_edges` | `Image` | 可视化 / 前景 / 颜色边界 |
| `/vision/preprocess/*` | `Image` | rgb/gray/gaussian/otsu/morphology |
| `/vision/pick_depth_debug` | `Float32MultiArray` | 每块 `[shape,u,v,z_m,valid]` |

`/vision/board_state` 布局（`strategy_node` 要求 `size()>=147` 才读其余）：
```
[0..6]    7 个形状的库存计数
[7..146]  140 格棋盘占用栅格（14×10，视觉端置 0 占位，占用判定在下游）
[147]     num_blocks
[148..]   每块 6 元组: [shape_id, pick_u, pick_v, angle_deg, geom_u, geom_v]
```
`geom_u/v`（几何中心）供下游补偿 hybrid 吸点偏离几何中心的放置平移。

---

## 9. 运行与调参

```bash
roslaunch lucky test_vision.launch                                  # 默认 DexiNed
roslaunch lucky test_vision.launch vision_node:=vision_processor_node_cpp   # 经典
roslaunch lucky test_vision.launch color_edge_thresh:=16 min_template_iou:=0.6
```

对照 debug 话题调参：块没切开→调小 `color_edge_thresh`（12~16）；块被切碎→调大
（28~35）；掉漆零碎边→调大 `color_edge_min_area`；粘连误分类→调高 `min_template_iou`；
彩色块漏检→确认 `use_saturation_foreground`；静止抖动→调小 `temporal_alpha`。

> 相机曝光/白平衡固定档见 [camera_config.yaml](../src/lucky/config/camera_config.yaml)（DexiNed）与
> [camera_config_1.yaml](../src/lucky/config/camera_config_1.yaml)（经典，高对比去饱和）；
> [test_vision.launch](../src/lucky/launch/test_vision.launch) 与 [lucky.launch](../src/lucky/launch/lucky.launch)
> 启动 6s 后自动 `dynparam load` 前者（硬编码）。**切换视觉实
> 现时相机档要一并换**，否则检测质量明显变差。命名空间 `/camera/rgb_camera`，务必先关自
> 动曝光/自动白平衡再调。
