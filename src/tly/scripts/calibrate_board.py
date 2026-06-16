#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白板标定工具 (Tetris/xArm/RealSense) —— 三类标定之一。

只做“白板/放置区”标定，产出放置所需的几何与高度数据，并用白板平面的法向确定
table_frame 的竖直方向（D1：实测 base-Z 不垂直于桌面，改用板面法向作竖直参考）。
不再标定独立的“散落/抓取面平面”（旧的 TABLE/PICK_SURFACE_PLANE_BASE 已弃用）——
抓取点 Z 由 RealSense 对齐深度在线获得，见 vision / path_planner 节点。

流程：
  步骤2  在白板平坦(无凸起)表面均匀采点 → PCA 拟合板面 → 法向定义 table_frame 的 Z 轴。
  步骤3  选 table_frame 原点 P 与 +X 方向（投影到板面）。
  步骤4  采方块顶面高度 → block_thickness（放置 Z = 凸起/板面高度 + block_thickness + 余量）。
  步骤5  采白板凸起网格 (A=9 / B=25 / C=140) → 仿射网格 + 高度图。
  步骤6  采白板无凸起平面 → BOARD_SURFACE_PLANE（table 系，应近似水平）。

主要输出（写入 tetris_config.yaml）：
  table_tf, BOARD_CENTERS_14x10_TABLE, PLACE_Z_MAP_14x10, BOARD_BUMP_HEIGHT_MAP_14x10,
  BOARD_SURFACE_PLANE, BOARD_SURFACE_NORMAL_BASE,
  PICK_Z(=block_thickness, 仅作 path 的平面回退), HOVER_Z 等。

另两类标定：手眼(easy_handeye)、单应性(pick_affine_calibration_tool.py)。
采点时输入 'u' / 'undo' 可撤销并重记上一个点。
"""

import os
import sys
import yaml
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_matrix


DEFAULT_CONFIG_PATH = "/root/catkin_ws/src/tly/config/tetris_config.yaml"
ROWS = 14
COLS = 10

BOARD_MODE_A_COL_ROW = [
    (0, 0), (5, 0), (9, 0),
    (0, 7), (5, 7), (9, 7),
    (0, 13), (5, 13), (9, 13),
]

BOARD_MODE_B_COLS = [0, 2, 5, 7, 9]
BOARD_MODE_B_ROWS = [0, 3, 7, 10, 13]


def get_eef_pose(tf_buffer, parent="link_base", child="link_eef"):
    try:
        trans = tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(3.0))
        return np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        ], dtype=float)
    except Exception as e:
        rospy.logerr("TF 变换获取失败，请确认机械臂状态: %s", e)
        return None


def require_pose(tf_buffer, message, parent="link_base", child="link_eef", allow_undo=False):
    """
    提示用户按回车记录当前位姿。
    如果 allow_undo 为 True，用户可以输入 'u' 返回 'UNDO' 以撤销上一步操作。
    """
    while not rospy.is_shutdown():
        prompt_str = message
        if allow_undo:
            prompt_str += " [按回车记录 | 输入 'u' 撤销上一步]: "
        else:
            prompt_str += " [按回车记录]: "
            
        ans = input(prompt_str).strip().lower()
        if allow_undo and ans in ('u', 'undo', '撤销'):
            return "UNDO"
            
        pose = get_eef_pose(tf_buffer, parent, child)
        if pose is not None and np.all(np.isfinite(pose)):
            print("  记录 {}<-{}: x={:.6f}, y={:.6f}, z={:.16f}".format(parent, child, *pose))
            return pose
        print("  ❌ 读取失败，请重新移动并再试。")
    sys.exit(1)


def normalize(v, name="vector"):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise RuntimeError("{} 长度太小，无法标定".format(name))
    return v / n


def project_point_to_plane(point, plane_point, normal):
    return point - np.dot(point - plane_point, normal) * normal


def fit_plane_pca(points_base):
    pts = np.asarray(points_base, dtype=float)
    centroid = np.mean(pts, axis=0)
    q = pts - centroid
    _, _, vh = np.linalg.svd(q, full_matrices=False)
    normal = normalize(vh[-1], "surface normal")

    # link_base usually has +Z upward. Force normal roughly upward.
    if normal[2] < 0:
        normal = -normal

    signed = q.dot(normal)
    rms = float(np.sqrt(np.mean(signed ** 2)))
    max_abs = float(np.max(np.abs(signed)))
    return centroid, normal, signed, rms, max_abs


def to_table(R_mat, P, point_base):
    return np.dot(R_mat.T, point_base - P)


def fit_plane_xyz(points_xyz):
    pts = np.asarray(points_xyz, dtype=float)
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    coef, _, _, _ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    pred = A.dot(coef)
    resid = pts[:, 2] - pred
    return float(coef[0]), float(coef[1]), float(coef[2]), float(np.std(resid))


def fit_affine_grid(sample_rows, sample_cols, sample_points):
    """
    Fit point(row,col) = origin + row_vec*row + col_vec*col.
    This gives a stable XY grid model. Z is handled separately by height models/maps.
    """
    A = np.c_[np.ones(len(sample_rows)), np.asarray(sample_rows, dtype=float), np.asarray(sample_cols, dtype=float)]
    P = np.asarray(sample_points, dtype=float)
    coeff, _, _, _ = np.linalg.lstsq(A, P, rcond=None)
    origin = coeff[0]
    row_vec = coeff[1]
    col_vec = coeff[2]
    pred = A.dot(coeff)
    resid = P - pred
    rms_xy = float(np.sqrt(np.mean(np.sum(resid[:, :2] ** 2, axis=1))))
    rms_z = float(np.sqrt(np.mean(resid[:, 2] ** 2)))
    return origin, row_vec, col_vec, rms_xy, rms_z


def idw_interpolate(sample_rows, sample_cols, sample_z, rows=ROWS, cols=COLS, power=2.0, eps=1e-6):
    """
    Inverse-distance weighting interpolation over board row/col.
    Used for 9/25/140 modes. In 140-point mode, exact sample values are preserved.
    """
    sr = np.asarray(sample_rows, dtype=float)
    sc = np.asarray(sample_cols, dtype=float)
    sz = np.asarray(sample_z, dtype=float)

    height_map = []
    for r in range(rows):
        line = []
        for c in range(cols):
            d = np.sqrt((sr - float(r)) ** 2 + (sc - float(c)) ** 2)
            exact_idx = np.where(d < eps)[0]
            if len(exact_idx) > 0:
                z = float(sz[exact_idx[0]])
            else:
                w = 1.0 / np.power(d + eps, power)
                z = float(np.sum(w * sz) / np.sum(w))
            line.append(z)
        height_map.append(line)
    return height_map


def fit_quadratic_height(sample_rows, sample_cols, sample_z):
    """
    Fit a smooth 2D quadratic height field:
      z = a0 + ar*r + ac*c + arr*r^2 + acc*c^2 + arc*r*c
    r/c normalized to 0..1.
    Works well for 9/25 points if the board warp is smooth.
    """
    rr = np.asarray(sample_rows, dtype=float) / float(ROWS - 1)
    cc = np.asarray(sample_cols, dtype=float) / float(COLS - 1)
    z = np.asarray(sample_z, dtype=float)
    A = np.c_[np.ones_like(rr), rr, cc, rr * rr, cc * cc, rr * cc]
    coef, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
    pred = A.dot(coef)
    resid = z - pred

    height_map = []
    for r in range(ROWS):
        r_n = r / float(ROWS - 1)
        row = []
        for c in range(COLS):
            c_n = c / float(COLS - 1)
            z_hat = (coef[0] + coef[1] * r_n + coef[2] * c_n +
                     coef[3] * r_n * r_n + coef[4] * c_n * c_n +
                     coef[5] * r_n * c_n)
            row.append(float(z_hat))
        height_map.append(row)
    return coef, height_map, float(np.std(resid)), float(np.max(np.abs(resid)))


def point_on_grid(origin, row_vec, col_vec, r, c):
    return origin + row_vec * float(r) + col_vec * float(c)


def ask_yes_no(question, default=True):
    default_txt = "Y/n" if default else "y/N"
    ans = input(f"{question} [{default_txt}]: ").strip().lower()
    if ans == "":
        return default
    return ans in ("y", "yes", "是", "1", "true")


def get_camera_info():
    print("正在监听相机内参 /camera/color/camera_info ...")
    try:
        cam_msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=3.0)
        cam_fx, cam_fy = cam_msg.K[0], cam_msg.K[4]
        cam_cx, cam_cy = cam_msg.K[2], cam_msg.K[5]
        print(f"✅ 成功获取相机内参: fx={cam_fx:.2f}, fy={cam_fy:.2f}, cx={cam_cx:.2f}, cy={cam_cy:.2f}")
        return cam_fx, cam_fy, cam_cx, cam_cy
    except Exception:
        print("❌ 未收到相机消息，将使用代码中的默认内参。")
        return 911.8016, 911.2428, 634.7139, 357.0596


def choose_pick_surface_sample_count():
    val = rospy.get_param("~pick_surface_samples", 0)
    try:
        val = int(val)
    except Exception:
        val = 0

    if val in (9, 16):
        return val

    while True:
        ans = input("白板平面拟合采样点数选择 9 或 16 [默认 16]: ").strip()
        if ans == "":
            return 16
        try:
            n = int(ans)
            if n in (9, 16):
                return n
        except Exception:
            pass
        print("请输入 9 或 16。")


def choose_board_mode():
    val = str(rospy.get_param("~board_mode", "")).strip().upper()
    if val in ("A", "B", "C"):
        return val

    print("\n白板标定模式：")
    print("  A = 9 点快速标定")
    print("  B = 25 点中等精度标定")
    print("  C = 140 点完整标定")
    while True:
        ans = input("请选择白板标定模式 A/B/C [默认 B]: ").strip().upper()
        if ans == "":
            return "B"
        if ans in ("A", "B", "C"):
            return ans
        print("请输入 A、B 或 C。")


def surface_prompt_positions(n):
    if n == 9:
        return [
            "左上", "上中", "右上",
            "左中", "中心", "右中",
            "左下", "下中", "右下",
        ]
    return [
        "第1行第1列", "第1行第2列", "第1行第3列", "第1行第4列",
        "第2行第1列", "第2行第2列", "第2行第3列", "第2行第4列",
        "第3行第1列", "第3行第2列", "第3行第3列", "第3行第4列",
        "第4行第1列", "第4行第2列", "第4行第3列", "第4行第4列",
    ]


def board_samples_for_mode(mode):
    if mode == "A":
        return BOARD_MODE_A_COL_ROW

    if mode == "B":
        return [(c, r) for r in BOARD_MODE_B_ROWS for c in BOARD_MODE_B_COLS]

    # Mode C: all 140 points, row-major. Prompt as (col,row).
    return [(c, r) for r in range(ROWS) for c in range(COLS)]


def choose_height_interpolation(board_mode):
    """
    For Mode A/B, default quadratic for smooth board warp.
    For Mode C, IDW is used, but exact values are preserved at every integer point.
    """
    default = "idw" if board_mode == "C" else "quadratic"
    val = str(rospy.get_param("~height_interpolation", "")).strip().lower()
    if val in ("quadratic", "idw"):
        return val
    ans = input(f"白板高度插值方式 quadratic/idw [默认 {default}]: ").strip().lower()
    if ans in ("quadratic", "idw"):
        return ans
    return default


def main():
    rospy.init_node("calibration_tool_node", anonymous=True)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    base_frame = rospy.get_param("~base_frame", "link_base")
    eef_frame = rospy.get_param("~eef_frame", "link_eef")
    rospy.sleep(2.0)

    rospy.loginfo("Waiting TF: %s <- %s", base_frame, eef_frame)
    if not tf_buffer.can_transform(base_frame, eef_frame, rospy.Time(0), rospy.Duration(10.0)):
        rospy.logerr("TF not ready: %s <- %s", base_frame, eef_frame)
        print(tf_buffer.all_frames_as_string())
        sys.exit(1)

    config_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
    block_samples = int(rospy.get_param("~block_samples", 3))
    board_surface_samples = int(rospy.get_param("~board_surface_samples", 5))

    pick_surface_samples = choose_pick_surface_sample_count()
    board_mode = choose_board_mode()
    board_sample_col_row = board_samples_for_mode(board_mode)
    height_interpolation = choose_height_interpolation(board_mode)

    print("\n" + "=" * 94)
    print("=== 俄罗斯方块增强标定工具：真实抓取平面 + 白板 A/B/C 多模式标定 ===")
    print(f"= 抓取/散落平面采样: {pick_surface_samples} 点")
    print(f"= 白板模式: {board_mode}，采样 {len(board_sample_col_row)} 个凸起中心")
    print(f"= 白板高度插值: {height_interpolation}")
    print("= 所有点都用吸盘中心轻触目标点，不要压弯发光板/白板。")
    print("= (新增) 如果标定失误，可以输入 'u' 撤销并重新记录上一个点。")
    print("=" * 94 + "\n")

    # Step 1.
    start_pose = require_pose(tf_buffer, "[步骤 1] 请将机械臂移动到【视觉拍照起点高度】", base_frame, eef_frame)
    cam_fx, cam_fy, cam_cx, cam_cy = get_camera_info()

    # Step 2: fit the white board plane (defines table_frame vertical / Z axis).
    print("\n--- 步骤 2：多点拟合【白板平坦表面】(定义 table_frame 竖直方向) ---")
    print("请在白板【平坦、无凸起】区域均匀采点。不要采凸起顶面，也不要采方块顶面。")
    scatter_surface_points = []
    labels = surface_prompt_positions(pick_surface_samples)
    i = 0
    while i < len(labels):
        label = labels[i]
        p = require_pose(
            tf_buffer,
            f"[步骤 2.{i+1}/{len(labels)}] 吸盘轻触【白板平坦(无凸起)表面】【{label}】点",
            base_frame,
            eef_frame,
            allow_undo=(i > 0)
        )
        if isinstance(p, str) and p == "UNDO":
            i -= 1
            scatter_surface_points.pop()
            print(f"  ↩️ 已撤销，重新回到上一个点:【{labels[i]}】。")
            continue
        scatter_surface_points.append(p)
        i += 1
    scatter_surface_points = np.asarray(scatter_surface_points, dtype=float)

    surface_centroid_base, surface_normal_base, signed_resid, surf_rms, surf_max = fit_plane_pca(scatter_surface_points)
    print("\n✅ 白板平面拟合完成（法向作为 table_frame 竖直方向）：")
    print("  normal(base): [{:.6f}, {:.6f}, {:.6f}]".format(*surface_normal_base))
    print("  centroid(base): [{:.6f}, {:.6f}, {:.6f}]".format(*surface_centroid_base))
    print("  residual RMS/max: {:.3f} / {:.3f} mm".format(surf_rms * 1000.0, surf_max * 1000.0))

    # Step 3: table_frame.
    print("\n--- 步骤 3：定义 table_frame 原点和 X 方向 ---")
    while True:
        P_user = require_pose(tf_buffer, "[步骤 3.1] 选择散落区桌面上的 table_frame 原点 P", base_frame, eef_frame, allow_undo=False)
        X_user = require_pose(tf_buffer, "[步骤 3.2] 沿你希望的 table_frame +X 方向选择一点 X", base_frame, eef_frame, allow_undo=True)
        if isinstance(X_user, str) and X_user == "UNDO":
            print("  ↩️ 已撤销，重新标定原点 P。")
            continue
        break

    P = project_point_to_plane(P_user, surface_centroid_base, surface_normal_base)
    X_proj = project_point_to_plane(X_user, surface_centroid_base, surface_normal_base)

    vZ = surface_normal_base
    vX = normalize(X_proj - P, "projected table X axis")
    vY = normalize(np.cross(vZ, vX), "table Y axis")
    vX = normalize(np.cross(vY, vZ), "table X axis")

    R_mat = np.eye(3)
    R_mat[:, 0] = vX
    R_mat[:, 1] = vY
    R_mat[:, 2] = vZ

    R_4x4 = np.eye(4)
    R_4x4[:3, :3] = R_mat
    rpy = euler_from_matrix(R_4x4, axes="sxyz")

    Cam_table = to_table(R_mat, P, start_pose)
    table_z_in_cam = float(Cam_table[2])

    # Step 4: block top height.
    print("\n--- 步骤 4：标定散落区方块顶面高度 ---")
    print("建议换不同颜色/不同位置采样，最终取中位数。")
    block_top_points_base = []
    block_top_points_table = []
    num_blocks = max(1, block_samples)
    i = 0
    while i < num_blocks:
        pt = require_pose(tf_buffer, f"[步骤 4.{i+1}/{num_blocks}] 方块平放在散落区，吸盘轻触【方块顶面】", base_frame, eef_frame, allow_undo=(i > 0))
        if isinstance(pt, str) and pt == "UNDO":
            i -= 1
            block_top_points_base.pop()
            block_top_points_table.pop()
            print(f"  ↩️ 已撤销，重新标定第 {i+1} 个方块高度。")
            continue
        block_top_points_base.append(pt)
        block_top_points_table.append(to_table(R_mat, P, pt))
        i += 1
        
    block_top_points_base = np.asarray(block_top_points_base)
    block_top_points_table = np.asarray(block_top_points_table)
    block_thickness = float(np.median(block_top_points_table[:, 2]))

    # Step 5: board samples.
    print("\n--- 步骤 5：白色底盘凸起网格标定 ---")
    print("坐标显示为 (col,row)，因为底盘是 10 列 x 14 行。")
    print(f"当前模式 {board_mode}: 需要采 {len(board_sample_col_row)} 个凸起中心。")
    if board_mode == "C":
        print("模式 C 会采完整 140 点，耗时较长，但能最好反映白板局部翘曲。如果误按，输入 'u' 撤销！")

    sample_rows = []
    sample_cols = []
    sample_points = []
    sample_records = []

    i = 0
    while i < len(board_sample_col_row):
        col, row = board_sample_col_row[i]
        p_base = require_pose(
            tf_buffer,
            f"[步骤 5.{i+1}/{len(board_sample_col_row)}] 吸盘对准【白色底盘凸起中心】 (col={col}, row={row})",
            base_frame,
            eef_frame,
            allow_undo=(i > 0)
        )
        if isinstance(p_base, str) and p_base == "UNDO":
            i -= 1
            sample_cols.pop()
            sample_rows.pop()
            sample_points.pop()
            sample_records.pop()
            prev_col, prev_row = board_sample_col_row[i]
            print(f"  ↩️ 已撤销！退回上一个点 (col={prev_col}, row={prev_row}) 重新记录。")
            continue
            
        p_tab = to_table(R_mat, P, p_base)
        sample_cols.append(col)
        sample_rows.append(row)
        sample_points.append(p_tab)
        sample_records.append({
            "col": int(col), "row": int(row),
            "table": [float(p_tab[0]), float(p_tab[1]), float(p_tab[2])],
            "link_base": [float(p_base[0]), float(p_base[1]), float(p_base[2])],
        })
        i += 1

    sample_points = np.asarray(sample_points, dtype=float)
    sample_rows = np.asarray(sample_rows, dtype=float)
    sample_cols = np.asarray(sample_cols, dtype=float)

    origin, row_vec, col_vec, xy_rms, grid_z_rms = fit_affine_grid(sample_rows, sample_cols, sample_points)

    q_coef = None
    q_height_map = None
    q_std = None
    q_max = None
    if len(sample_rows) >= 9:
        q_coef, q_height_map, q_std, q_max = fit_quadratic_height(sample_rows, sample_cols, sample_points[:, 2])

    idw_height_map = idw_interpolate(sample_rows, sample_cols, sample_points[:, 2], ROWS, COLS)

    if height_interpolation == "quadratic" and q_height_map is not None:
        bump_height_map = q_height_map
        height_model_type = "quadratic"
        height_std = q_std
        height_max_abs = q_max
    else:
        bump_height_map = idw_height_map
        height_model_type = "idw"
        # For IDW, compute residual at measured samples. In Mode C exact values should be zero.
        pred_at_samples = []
        for r, c in zip(sample_rows.astype(int), sample_cols.astype(int)):
            pred_at_samples.append(bump_height_map[int(r)][int(c)])
        pred_at_samples = np.asarray(pred_at_samples)
        residual = sample_points[:, 2] - pred_at_samples
        height_std = float(np.std(residual))
        height_max_abs = float(np.max(np.abs(residual)))

    # Step 6: board flat surface.
    print("\n--- 步骤 6：标定白色底盘无凸起平面高度 ---")
    surface_points = []
    num_surf = max(1, board_surface_samples)
    i = 0
    while i < num_surf:
        pt = require_pose(tf_buffer, f"[步骤 6.{i+1}/{num_surf}] 吸盘轻触【底盘无凸起的平坦表面】", base_frame, eef_frame, allow_undo=(i > 0))
        if isinstance(pt, str) and pt == "UNDO":
            i -= 1
            surface_points.pop()
            print(f"  ↩️ 已撤销，重新记录平坦表面的第 {i+1} 个点。")
            continue
        surface_points.append(to_table(R_mat, P, pt))
        i += 1
        
    surface_points = np.asarray(surface_points, dtype=float)
    board_surface_z = float(np.median(surface_points[:, 2]))
    surf_a, surf_b, surf_c, board_surf_std = fit_plane_xyz(surface_points)

    use_bump_height_for_place = ask_yes_no(
        "放置高度是否按局部凸起/鼓包高度补偿？如果方块主要压在凸起上，选是；如果落在底板平面上，选否",
        default=True,
    )

    place_z_margin = float(rospy.get_param("~place_z_margin", 0.0))
    try:
        txt = input(f"放置高度额外余量 place_z_margin，单位 m [默认 {place_z_margin:.4f}]: ").strip()
        if txt:
            place_z_margin = float(txt)
    except Exception:
        pass

    place_z_map = []
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            if use_bump_height_for_place:
                base_z = bump_height_map[r][c]
            else:
                p_grid = point_on_grid(origin, row_vec, col_vec, r, c)
                base_z = surf_a * p_grid[0] + surf_b * p_grid[1] + surf_c
            row.append(float(base_z + block_thickness + place_z_margin))
        place_z_map.append(row)

    board_centers = []
    for r in range(ROWS):
        line = []
        for c in range(COLS):
            p = point_on_grid(origin, row_vec, col_vec, r, c)
            # x/y from affine grid, z from selected height map
            line.append([float(p[0]), float(p[1]), float(bump_height_map[r][c])])
        board_centers.append(line)

    legacy_grid = float(0.5 * (np.linalg.norm(row_vec[:2]) + np.linalg.norm(col_vec[:2])))
    legacy_origin_x = float(origin[0])
    legacy_origin_y = float(origin[1])
    place_z_legacy = float(np.median(np.asarray(place_z_map)))
    hover_z = place_z_legacy + 0.10
    pick_z = float(block_thickness)

    height_model_info = {
        "selected_type": height_model_type,
        "idw": {
            "power": 2.0,
            "residual_std_m_at_samples": float(height_std) if height_model_type == "idw" else None,
            "residual_max_abs_m_at_samples": float(height_max_abs) if height_model_type == "idw" else None,
        },
        "quadratic": {
            "available": q_coef is not None,
            "z =": "a0 + ar*r + ac*c + arr*r^2 + acc*c^2 + arc*r*c, r/c normalized to 0..1",
            "coefficients": [float(v) for v in q_coef] if q_coef is not None else [],
            "residual_std_m": float(q_std) if q_std is not None else None,
            "residual_max_abs_m": float(q_max) if q_max is not None else None,
        },
        "selected_residual_std_m": float(height_std),
        "selected_residual_max_abs_m": float(height_max_abs),
    }

    config = {
        "tetris": {
            "calibration_frames": {"base_frame": str(base_frame), "eef_frame": str(eef_frame)},

            "table_tf": {
                "x": float(P[0]), "y": float(P[1]), "z": float(P[2]),
                "roll": float(rpy[0]), "pitch": float(rpy[1]), "yaw": float(rpy[2]),
            },

            # 白板平面法向 = table_frame 竖直参考 (D1)。不再输出独立的散落/抓取面平面。
            "BOARD_SURFACE_NORMAL_BASE": [float(x) for x in surface_normal_base],
            "BOARD_SURFACE_CENTROID_BASE": [float(x) for x in surface_centroid_base],
            "BOARD_PLANE_FIT_RMS_M": float(surf_rms),
            "BOARD_PLANE_FIT_MAX_ABS_M": float(surf_max),
            "BOARD_PLANE_SAMPLES_BASE": [[float(v) for v in p] for p in scatter_surface_points],
            "TABLE_FRAME_ORIGIN_BASE": [float(v) for v in P],
            "TABLE_FRAME_XDIR_BASE": [float(v) for v in X_proj],

            # Legacy-compatible parameters.
            "GRID_SIZE": legacy_grid,
            "BOARD_ORIGIN_X": legacy_origin_x,
            "BOARD_ORIGIN_Y": legacy_origin_y,
            "PICK_Z": pick_z,
            "PLACE_Z": place_z_legacy,
            "HOVER_Z": float(hover_z),
            "CAM_FX": float(cam_fx), "CAM_FY": float(cam_fy),
            "CAM_CX": float(cam_cx), "CAM_CY": float(cam_cy),
            "TABLE_Z_IN_CAMERA": float(table_z_in_cam),

            # Enhanced board outputs.
            "BOARD_ROWS": ROWS,
            "BOARD_COLS": COLS,
            "BOARD_CALIBRATION_MODE": {
                "mode": str(board_mode),
                "sample_count": int(len(board_sample_col_row)),
                "description": {
                    "A": "9 points",
                    "B": "25 points",
                    "C": "140 points",
                }[board_mode],
            },
            "BOARD_SAMPLE_FORMAT": "samples are prompted as (col,row), stored internally as row/col",
            "BOARD_SAMPLES_TABLE": sample_records,
            "BOARD_ORIGIN_TABLE": [float(origin[0]), float(origin[1]), float(origin[2])],
            "BOARD_ROW_STEP_TABLE": [float(row_vec[0]), float(row_vec[1]), float(row_vec[2])],
            "BOARD_COL_STEP_TABLE": [float(col_vec[0]), float(col_vec[1]), float(col_vec[2])],
            "BOARD_CENTERS_14x10_TABLE": board_centers,
            "BOARD_GRID_FIT_RMS_XY_M": float(xy_rms),
            "BOARD_GRID_FIT_RMS_Z_M": float(grid_z_rms),

            "BOARD_SURFACE_Z": float(board_surface_z),
            "BOARD_SURFACE_PLANE": {
                "model": "z = a*x + b*y + c in table_frame",
                "a": float(surf_a), "b": float(surf_b), "c": float(surf_c),
                "residual_std_m": float(board_surf_std),
            },

            "BOARD_BUMP_HEIGHT_MODEL": height_model_info,
            "BOARD_BUMP_HEIGHT_MAP_14x10": bump_height_map,
            "PLACE_Z_MAP_14x10": place_z_map,
            "PLACE_Z_MARGIN": float(place_z_margin),
            "BLOCK_THICKNESS": float(block_thickness),
            "USE_BUMP_HEIGHT_FOR_PLACE": bool(use_bump_height_for_place),
        }
    }

    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    print("\n🎉 增强标定完成！")
    print("配置已保存:", config_path)
    print("\n关键结果：")
    print(f"  白板平面拟合采样点数: {pick_surface_samples}")
    print(f"  白板平面 residual RMS/max: {surf_rms*1000:.3f} / {surf_max*1000:.3f} mm")
    print(f"  board 法向(base) = table_frame 竖直方向: {[round(float(v), 6) for v in surface_normal_base]}")
    print(f"  方块厚度 block_thickness(=PICK_Z, 平面回退用): {pick_z:.6f} m")
    print(f"  白板模式: {board_mode}, 采样点数: {len(board_sample_col_row)}")
    print(f"  白板高度模型: {height_model_type}, residual std/max: {height_std*1000:.3f} / {height_max_abs*1000:.3f} mm")
    print(f"  9/25/140 点网格 XY 拟合 RMS: {xy_rms*1000:.3f} mm")
    print(f"  底板无凸起平面拟合残差 std: {board_surf_std*1000:.3f} mm")
    print(f"  PLACE_Z extra margin: {place_z_margin*1000:.3f} mm")
    print("\n建议：")
    print("  - 如果白板局部翘曲明显，优先用模式 C=140 点。")
    print("  - 抓取 Z 由视觉/路径节点用 RealSense 深度在线获得，无需再标定抓取面。")
    print("  - 放置控制读取 BOARD_CENTERS_14x10_TABLE 和 PLACE_Z_MAP_14x10。")


if __name__ == "__main__":
    main()