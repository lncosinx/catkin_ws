#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白板标定工具 (Tetris/xArm/RealSense) —— 三类标定之一。

只做“白板/放置区”标定，产出放置所需的几何与高度数据。坐标系 board_frame 完全由
白板网格派生（原点=网格原点, Z=板面法向, X≈行轴），不再有手动选的 table_frame，
也不再发布/查询 table_frame TF —— board_frame 的位姿以常量 BOARD_POSE_BASE 写入
config，控制/路径/单应性节点直接在 base 系用它换算（全量 base 化）。
抓取点 Z 由 RealSense 对齐深度在线获得，见 vision / path_planner 节点。

流程：
  步骤1  记录拍照起点 + 相机内参。
  步骤2  单机位对齐深度多帧中值 + ROI + RANSAC 拟板面 → 法向定义 board_frame 的 Z 轴。
  步骤3  采白板凸起网格 (A=9 / B=25 / C=140, base 系采点) → 仿射网格 + 高度图；并由网格
         派生 board_frame（origin=网格原点, X≈行轴, 方向对齐到现有系以保 way 约定）。
  步骤4  方块厚度（触顶面+触旁边裸面，沿法向取差，同波纹管；恒正，免疫绝对零点）→ block_thickness。
  步骤5  分批在白板上摆方块，按格心采深度【方块顶面】→ 直接得每格放置 Z（不受方块缝隙影响，
         自动含板面翘曲+凸起+多格架桥）；裸板面(步骤2平面)仅作覆盖判据基准。
  步骤6  波纹管长度补偿（可选）：拆波纹管触碰发光板 + 装波纹管触碰同一点，两次 link_tcp
         高度差即自由长度；配期望压缩量 → tcp_*_offset_z（只补偿下扎 Z，不进入 XY/单应性/
         射线求交）。控制/路径节点从 /tetris/TCP_*_OFFSET_Z 自动读取。

主要输出（写入 tetris_config.yaml）：
  BOARD_POSE_BASE, BOARD_CENTERS_14x10_BOARD, PLACE_Z_MAP_14x10, BOARD_PLACE_TOP_MAP_14x10,
  BOARD_BUMP_HEIGHT_MAP_14x10, BOARD_SURFACE_PLANE(board 系), BOARD_SURFACE_NORMAL_BASE,
  PICK_Z(=block_thickness, 仅作 path 的平面回退), HOVER_Z 等。

另两类标定：手眼(easy_handeye)、单应性(pick_affine_calibration_tool.py)。
⚠️ 坐标系由网格派生后，必须重跑 pick_affine 重标单应性（pixel→board-XY），再上机核对。
采点时输入 'u' / 'undo' 可撤销并重记上一个点。

部分标定（只跑某些步骤）：
  用 ROS 参数 ~steps 选择要执行的步骤，未选中的步骤会从现有 tetris_config.yaml
  读取其结果。格式示例：'4'（只标定方块高度）、'3,4,5'、'2-6'、'all'（默认全部）。
  不给 ~steps 时会交互式提示。写文件时只更新本次产出的键，其余键（含手眼/单应性
  等其它工具写入的内容）原样保留。
  注意依赖：重跑步骤 2 会让 3/4/5/6 的旧数据失配；重跑步骤 3(网格+坐标系)会让 4/5/6
  以及单应性失配。这种情况脚本会提示并要求确认。

单独修改某些放置格心（不重标整块网格）：
  用 ROS 参数 ~edit_cells:=true 进入交互模式：选 (col,row) → 吸嘴对准该格凸起中心回车读 TF
  → 覆盖该格的放置 XY；覆盖量存入 BOARD_CELL_OVERRIDES_BOARD 并即时重写 BOARD_CENTERS_14x10_BOARD。
  只改 XY（z=凸起高度、放置深度 PLACE_Z_MAP 不变）。需已有步骤 2/3 的坐标系与网格；重跑步骤 3
  会重派生网格并清空这些逐格覆盖。输入 'list' 查看已有覆盖，'undo' 撤销上一次修改，'q' 结束。

步骤8 白板格心巡检（验证标定，不写配置）：
  用 ROS 参数 ~verify_cells:=true 进入：驱动机械臂依次悬停到白板各格中心【上方设定高度】，
  肉眼核对标定 XY 是否对准（与生产控制器同一条路径：原生 /xarm/move_line + BOARD_POSE_BASE）。
  按回车走下一个格；支持选定格子（~verify_cells_spec: all / r<行> / c<列> / 'col,row;...'）与设置
  每格上方高度（~verify_height，米；巡检中输入 'h' 可改）。需已有步骤 2/3 结果。⚠️ 会自动驱动真机。
"""

import os
import sys
import json
import yaml
import rospy
import tf2_ros
import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import euler_from_matrix, euler_matrix, quaternion_matrix


DEPTH_TOPIC_DEFAULT = "/camera/aligned_depth_to_color/image_raw"
COLOR_TOPIC_DEFAULT = "/camera/color/image_raw"
CAMERA_FRAME_DEFAULT = "camera_color_optical_frame"


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


def _step3_progress_path(config_path):
    return os.path.join(os.path.dirname(os.path.abspath(config_path)), ".step3_bump_progress.json")


def _save_step3_progress(path, board_mode, cols, rows, pts):
    """边采边存白板凸起进度，便于相机掉线/重启 launch 后续采，避免重标整堆凸起。"""
    try:
        data = {"board_mode": str(board_mode),
                "points": [[int(c), int(r), float(p[0]), float(p[1]), float(p[2])]
                           for c, r, p in zip(cols, rows, pts)]}
        with open(path, "w") as f:
            json.dump(data, f)
    except Exception as e:
        rospy.logwarn("步骤3进度保存失败(不影响标定): %s", e)


def _load_step3_progress(path, board_mode):
    try:
        if not os.path.exists(path):
            return None
        with open(path) as f:
            data = json.load(f)
        if str(data.get("board_mode")) != str(board_mode):
            return None
        pts = data.get("points") or []
        return pts if pts else None
    except Exception:
        return None


def _clear_step3_progress(path):
    try:
        if os.path.exists(path):
            os.remove(path)
    except Exception:
        pass


# ---- 固件上报位姿（xArm 原生 /xarm/xarm_states.pose，单位 mm / rad）----
# 为什么默认用它替代 ROS TF：URDF 名义运动学与固件【出厂逐台标定】运动学有 ~mm 级差异，
# 而执行(move_line)走的是固件系。标定若用 ROS TF 记录、控制器用 move_line 执行，就会带一个
# 系统性偏差（实测本机约 4~5mm）。改成用固件位姿记录 → 记录源=执行源，偏差归零。
# 前提：固件 TCP 偏置=吸嘴尖(link_tcp)，故 pose[0:3] 即吸嘴尖在固件 base 系坐标。默认【只读+核对】，
# 不主动改固件 TCP（避免上报位姿跳变+save_conf 覆盖你 UF Studio 的 TCP 配置）。任一时刻固件位姿
# 不可用则自动回退 ROS TF。
_FW = {
    "enabled": False,      # ~use_firmware_pose 主开关
    "base": "link_base",   # 仅当请求 (parent,child)==(base,eef) 时才用固件位姿
    "eef": "link_tcp",
    "pose": None,          # 最新 [x,y,z(mm), roll,pitch,yaw(rad)]
    "offset": None,        # 固件当前 TCP 偏置 [x,y,z(mm), r,p,y(rad)]（RobotMsg.offset）
    "stamp": None,
    "max_age": 0.5,        # s；位姿过旧视为不可用，回退 TF
}


def _fw_state_cb(msg):
    if len(msg.pose) >= 6:
        _FW["pose"] = [float(v) for v in msg.pose[:6]]
        _FW["stamp"] = rospy.Time.now()
    if len(getattr(msg, "offset", [])) >= 3:
        _FW["offset"] = [float(v) for v in msg.offset[:6]]


def _fw_pose_fresh():
    """返回新鲜的固件位姿 [x,y,z(mm),r,p,y(rad)]；未启用/过旧/缺失返回 None。"""
    if not _FW["enabled"] or _FW["pose"] is None or _FW["stamp"] is None:
        return None
    if (rospy.Time.now() - _FW["stamp"]).to_sec() > _FW["max_age"]:
        return None
    return _FW["pose"]


def _fw_use(parent, child):
    return _FW["enabled"] and parent == _FW["base"] and child == _FW["eef"]


def get_eef_pose(tf_buffer, parent="link_base", child="link_eef"):
    if _fw_use(parent, child):
        fp = _fw_pose_fresh()
        if fp is not None:
            return np.array([fp[0] / 1000.0, fp[1] / 1000.0, fp[2] / 1000.0], dtype=float)
        rospy.logwarn_throttle(5.0, "固件位姿(/xarm/xarm_states)暂不可用，本点回退 ROS TF 记录。")
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


def get_eef_rotmat(tf_buffer, parent="link_base", child="link_eef"):
    """读 parent<-child 的旋转矩阵(3x3)。失败返回 None。"""
    if _fw_use(parent, child):
        fp = _fw_pose_fresh()
        if fp is not None:
            return euler_matrix(fp[3], fp[4], fp[5], axes="sxyz")[:3, :3]
        rospy.logwarn_throttle(5.0, "固件姿态(/xarm/xarm_states)暂不可用，回退 ROS TF。")
    try:
        trans = tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(3.0))
        q = trans.transform.rotation
        return quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    except Exception as e:
        rospy.logerr("TF 朝向获取失败，请确认机械臂状态: %s", e)
        return None


def setup_firmware_pose(tf_buffer, base_frame, eef_frame):
    """启用"用固件 /xarm/xarm_states.pose 记录触点"（消除 URDF↔固件运动学系统偏差）。
    【纯被动，不命令机械臂运动】：订阅状态话题 → 等首帧 → 读固件当前 TCP 偏置(RobotMsg.offset)与
    吸嘴尖(TF link_eef->eef_frame)核对，一致即 pose=吸嘴尖可用、不一致仅提示 → 打印固件位姿 vs ROS TF
    偏差供核对。仅当 ~set_tcp_offset_for_record:=true(默认 false)才主动设固件 TCP（仍不动臂，但会让上报
    位姿跳变且 save_conf 持久化）。~use_firmware_pose:=false 可整体关闭退回 TF。任一环节失败不致命：
    置 enabled=False，全流程自动回退 ROS TF。"""
    if not bool(rospy.get_param("~use_firmware_pose", True)):
        print("ℹ️ ~use_firmware_pose=false：仍用 ROS TF 记录触点（URDF 系）。")
        return
    try:
        from xarm_msgs.msg import RobotMsg
        from xarm_msgs.srv import TCPOffset
    except Exception as e:
        print(f"⚠️ 无法导入 xarm_msgs（{e}）；退回 ROS TF 记录。")
        return

    _FW["base"] = base_frame
    _FW["eef"] = eef_frame
    rospy.Subscriber("/xarm/xarm_states", RobotMsg, _fw_state_cb)

    # 先等首帧（拿到 pose + 当前固件 TCP 偏置 offset）。
    try:
        rospy.wait_for_message("/xarm/xarm_states", RobotMsg, timeout=6.0)
    except Exception:
        print("⚠️ 未收到 /xarm/xarm_states；退回 ROS TF 记录。确认机器人在线、robot_ip 正确。")
        return

    # 期望的吸嘴尖偏置 = TF(link_eef->eef_frame)。默认【只核对不改】：读固件当前 TCP 偏置与它比对，
    # 一致则 xarm_states.pose 即吸嘴尖、可直接用；不一致才提示。绝不默认调 set_tcp_offset
    # （那会让上报位姿跳变，且驱动会 save_conf 覆盖你 UF Studio 的 TCP 配置）。
    exp = None
    try:
        tr = tf_buffer.lookup_transform("link_eef", eef_frame, rospy.Time(0), rospy.Duration(3.0))
        exp = np.array([tr.transform.translation.x, tr.transform.translation.y,
                        tr.transform.translation.z]) * 1000.0
    except Exception as e:
        print(f"  ⚠️ 读 TF link_eef->{eef_frame} 失败（{e}）；无法核对固件 TCP，请自行确认=吸嘴尖。")

    cur = _FW.get("offset")
    tcp_ok = False
    if exp is not None and cur is not None:
        d = np.array(cur[:3]) - exp
        if float(np.linalg.norm(d)) <= 1.0:   # 1mm 容差
            tcp_ok = True
            print("  ✅ 固件当前 TCP 偏置=({:.2f},{:.2f},{:.2f})mm ≈ 吸嘴尖，xarm_states.pose 即吸嘴尖。"
                  .format(*cur[:3]))
        else:
            print("  ⚠️ 固件当前 TCP 偏置=({:.2f},{:.2f},{:.2f})mm 与吸嘴尖({:.2f},{:.2f},{:.2f})mm 差 {:.2f}mm！"
                  .format(cur[0], cur[1], cur[2], exp[0], exp[1], exp[2], float(np.linalg.norm(d))))
            print("     → 请在 UF Studio 选中吸嘴尖那个 TCP，或用 ~set_tcp_offset_for_record:=true 让本工具设置")
            print("       （注意：set 会让上报位姿跳变、且驱动 save_conf 会持久化覆盖 UF Studio 的 TCP）。")

    # 仅当【显式】要求时才主动设固件 TCP（默认 False）。不会命令机械臂运动，但会改上报位姿+save_conf。
    if not tcp_ok and exp is not None and bool(rospy.get_param("~set_tcp_offset_for_record", False)):
        try:
            rospy.wait_for_service("/xarm/set_tcp_offset", timeout=5.0)
            set_tcp = rospy.ServiceProxy("/xarm/set_tcp_offset", TCPOffset)
            resp = set_tcp(float(exp[0]), float(exp[1]), float(exp[2]), 0.0, 0.0, 0.0)
            if getattr(resp, "ret", 0) != 0:
                print(f"  ⚠️ set_tcp_offset 返回 ret={resp.ret}；沿用固件内已存工具偏置。")
            else:
                print("  已设固件工具偏置(TCP)=({:.2f},{:.2f},{:.2f})mm（不动臂；上报位姿会跳变；已 save_conf 持久化）。"
                      .format(*exp))
        except Exception as e:
            print(f"  ⚠️ 设固件 TCP 失败（{e}）；沿用固件内已存工具偏置。")

    _FW["enabled"] = True
    fp = _fw_pose_fresh()
    print("✅ 记录源=固件位姿 /xarm/xarm_states.pose（消除 URDF↔固件运动学偏差；执行 move_line 可精确复现）。")
    # 启动自检：同一时刻固件位姿 vs ROS TF，打印偏差（即之前 verify 偏 4~5mm 的根源量）。
    if fp is not None:
        try:
            tr = tf_buffer.lookup_transform(base_frame, eef_frame, rospy.Time(0), rospy.Duration(2.0))
            tfp = np.array([tr.transform.translation.x, tr.transform.translation.y,
                            tr.transform.translation.z]) * 1000.0
            d = np.array(fp[:3]) - tfp
            print("  🔎 固件 vs TF 偏差(mm): dx={:.2f} dy={:.2f} dz={:.2f} | XY={:.2f} mm"
                  .format(d[0], d[1], d[2], float(np.hypot(d[0], d[1]))))
            print("     （即 URDF↔固件运动学差；用固件记录后此偏差不再进入标定→执行链路）")
        except Exception:
            pass


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


def derive_board_frame(origin_base, row_vec_base, col_vec_base, normal_base, plane_centroid, tcfg):
    """由网格(base)派生 board_frame：Z=板面法向，X≈行/列轴(投影到板面)，原点=网格原点(投影到板面)。
    为保持与 strategy 的 way 约定一致，X 取与【现有坐标系 X 轴最接近】的网格轴方向；首次标定
    (无现有系)默认 +行轴并提示需上机核对。返回 (P, R_mat, rpy)。"""
    vZ = normalize(np.asarray(normal_base, dtype=float), "board normal")
    P = project_point_to_plane(np.asarray(origin_base, dtype=float), plane_centroid, vZ)

    def in_plane(v):
        v = np.asarray(v, dtype=float)
        v = v - np.dot(v, vZ) * vZ
        return normalize(v, "grid axis")

    candidates = [in_plane(row_vec_base), -in_plane(row_vec_base),
                  in_plane(col_vec_base), -in_plane(col_vec_base)]

    ref_x = None
    bp = tcfg.get("BOARD_POSE_BASE") if isinstance(tcfg, dict) else None
    if isinstance(bp, dict) and bp.get("rpy"):
        r = [float(v) for v in bp["rpy"]]
        ref_x = euler_matrix(r[0], r[1], r[2], axes="sxyz")[:3, 0]
    elif isinstance(tcfg, dict) and isinstance(tcfg.get("table_tf"), dict):
        tt = tcfg["table_tf"]
        ref_x = euler_matrix(float(tt["roll"]), float(tt["pitch"]), float(tt["yaw"]), axes="sxyz")[:3, 0]

    if ref_x is not None:
        vX = max(candidates, key=lambda c: float(np.dot(c, ref_x)))
        print("  X 轴方向已对齐到与现有坐标系最接近的网格轴（保持 way 约定）。")
    else:
        vX = candidates[0]
        print("  ⚠️ 无现有坐标系参考：默认 X=+行轴。请在 test_controller(限1任务) 上核对放置朝向后再批量运行！")

    vY = normalize(np.cross(vZ, vX), "board Y axis")
    vX = normalize(np.cross(vY, vZ), "board X axis")
    R_mat = np.eye(3)
    R_mat[:, 0] = vX
    R_mat[:, 1] = vY
    R_mat[:, 2] = vZ
    R_4x4 = np.eye(4)
    R_4x4[:3, :3] = R_mat
    rpy = euler_from_matrix(R_4x4, axes="sxyz")
    return P, R_mat, rpy


def ask_yes_no(question, default=True):
    default_txt = "Y/n" if default else "y/N"
    ans = input(f"{question} [{default_txt}]: ").strip().lower()
    if ans == "":
        return default
    return ans in ("y", "yes", "是", "1", "true")


def ask_float(question, default):
    """提示输入一个浮点数；回车用默认值；非法输入重试。"""
    while True:
        ans = input(f"{question} [默认 {default:.4f}]: ").strip()
        if ans == "":
            return float(default)
        try:
            return float(ans)
        except ValueError:
            print("  请输入数字（米），或直接回车用默认值。")


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


# ---------------------------------------------------------------------------
# 深度采板面（替代旧的触点 Step 2 / Step 6）
# ---------------------------------------------------------------------------

def load_existing_config(path):
    """读取已有 YAML 配置（用于复用持久化的 ROI 等）。缺失时返回 {}。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def write_config_merge(config_path, existing_cfg, tetris_updates):
    """把 tetris_updates 合并进配置的 tetris 段并【原子】落盘；返回更新后的 existing_cfg。
    - 只更新给定键，其余键（含手眼/单应性等其它工具写入的内容）原样保留。
    - 先写临时文件再 os.replace 原子替换，避免写一半被杀导致配置损坏。
    供各步检查点增量保存：完成一步就落盘该步参数，后续步骤崩溃也不丢前面的标定。"""
    out_cfg = existing_cfg if isinstance(existing_cfg, dict) else {}
    merged = dict(out_cfg.get("tetris", {})) if isinstance(out_cfg.get("tetris"), dict) else {}
    merged.update(tetris_updates)
    out_cfg["tetris"] = merged
    d = os.path.dirname(config_path) or "."
    os.makedirs(d, exist_ok=True)
    tmp = config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(out_cfg, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    os.replace(tmp, config_path)
    return out_cfg


def parse_roi(val):
    """把 ~board_roi 解析成 [u0,v0,w,h]；接受 list 或 'u0,v0,w,h' 字符串；非法返回 None。"""
    if val is None:
        return None
    if isinstance(val, str):
        parts = [p for p in val.replace(",", " ").split() if p]
        if len(parts) != 4:
            return None
        try:
            return [int(float(p)) for p in parts]
        except Exception:
            return None
    try:
        seq = list(val)
        if len(seq) < 4:
            return None
        return [int(seq[0]), int(seq[1]), int(seq[2]), int(seq[3])]
    except Exception:
        return None


def depth_msg_to_meters(msg):
    """sensor_msgs/Image 深度帧 → float32 米。支持 16UC1(mm) / 32FC1(m)。"""
    if msg.encoding in ("16UC1", "mono16"):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth = arr.astype(np.float32) / 1000.0
    elif msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = arr.astype(np.float32).copy()
    else:
        raise RuntimeError("不支持的深度编码: %s" % msg.encoding)
    depth[~np.isfinite(depth)] = 0.0
    return depth


def grab_median_depth(depth_topic, n_frames, timeout=5.0):
    """静止连抓 n_frames 帧，逐像素对有效(>0)样本取中值，返回米制 HxW。"""
    print(f"  连抓 {n_frames} 帧深度做逐像素中值（保持静止）...")
    stack = []
    h = w = None
    for _ in range(max(1, n_frames)):
        try:
            msg = rospy.wait_for_message(depth_topic, Image, timeout=timeout)
        except Exception as e:
            print(f"  ⚠️ 等待深度帧失败: {e}")
            continue
        d = depth_msg_to_meters(msg)
        if h is None:
            h, w = d.shape
        elif d.shape != (h, w):
            continue
        stack.append(d)
    if not stack:
        raise RuntimeError("未取到任何深度帧，检查 %s 是否在线" % depth_topic)
    arr = np.stack(stack, axis=0)
    arr[arr <= 0.0] = np.nan
    with np.errstate(all="ignore"):
        median = np.nanmedian(arr, axis=0)
    median[~np.isfinite(median)] = 0.0
    print(f"  ✅ 有效帧 {len(stack)}/{n_frames}")
    return median.astype(np.float32)


def depth_preview(depth_m):
    """把米制深度图转成 8-bit 伪彩预览，供拖框用。"""
    valid = depth_m[depth_m > 0]
    if valid.size == 0:
        lo, hi = 0.0, 1.0
    else:
        lo, hi = float(np.percentile(valid, 2)), float(np.percentile(valid, 98))
    if hi - lo < 1e-6:
        hi = lo + 1e-6
    norm = np.clip((depth_m - lo) / (hi - lo), 0.0, 1.0)
    img8 = (norm * 255).astype(np.uint8)
    img8[depth_m <= 0] = 0
    return cv2.applyColorMap(img8, cv2.COLORMAP_JET)


def grab_color_bgr(color_topic, timeout=5.0):
    """抓一帧彩色图 → BGR uint8 (HxWx3)。深度已对齐到彩色，故彩色像素坐标=深度像素坐标。失败返回 None。"""
    try:
        msg = rospy.wait_for_message(color_topic, Image, timeout=timeout)
    except Exception as e:
        print(f"  ⚠️ 等待彩色帧失败（退回深度预览框选）: {e}")
        return None
    h, w, enc = msg.height, msg.width, msg.encoding
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        if enc in ("rgb8", "bgr8"):
            img = arr.reshape(h, w, 3)
            if enc == "rgb8":
                img = img[:, :, ::-1]
        elif enc == "mono8" and cv2 is not None:
            img = cv2.cvtColor(arr.reshape(h, w), cv2.COLOR_GRAY2BGR)
        else:
            print(f"  ⚠️ 不支持的彩色编码 {enc}（退回深度预览框选）。")
            return None
    except Exception as e:
        print(f"  ⚠️ 彩色帧解析失败（退回深度预览框选）: {e}")
        return None
    return np.ascontiguousarray(img)


def make_roi_display(depth_m, color_bgr):
    """框选用底图：彩色图叠加“无深度区压暗”，提示把 ROI 框在有深度(明亮)处。
    color_bgr 与 depth_m 必须同分辨率（aligned_depth_to_color 保证）；不一致时缩放。"""
    disp = color_bgr
    if disp.shape[:2] != depth_m.shape[:2]:
        if cv2 is None:
            return depth_preview(depth_m)
        disp = cv2.resize(disp, (depth_m.shape[1], depth_m.shape[0]))
    disp = disp.copy()
    invalid = depth_m <= 0
    disp[invalid] = (disp[invalid].astype(np.float32) * 0.4).astype(np.uint8)
    return disp


def show_coverage_overlay(depth_m, color_bgr, P, R_mat, R_cam, t_cam, intr,
                          origin, row_vec, col_vec, proj_z_map, block_thickness,
                          covered, newly_cells, batch, save_dir, H_inv=None):
    """步骤5每批覆盖调试图：彩色底图(无深度区压暗)上画 140 个格心投影点。
    绿=本批新增, 青=既有覆盖, 红=未覆盖。存盘并弹窗(在图窗按任意键继续)。
    H_inv 给定则用单应性(board-XY->pixel)投影格心(更准、绕开手眼)，否则用 proj_z_map+手眼 3D 投影。"""
    if cv2 is None:
        return
    base = make_roi_display(depth_m, color_bgr) if color_bgr is not None else depth_preview(depth_m)
    overlay = base.copy()
    H, W = depth_m.shape
    rows = len(proj_z_map)
    cols = len(proj_z_map[0]) if rows else 0
    for r in range(rows):
        for c in range(cols):
            cell_z = float(proj_z_map[r][c])
            p_board = point_on_grid(origin, row_vec, col_vec, r, c)
            p_guess = np.array([p_board[0], p_board[1], cell_z], dtype=float)
            uv = cell_center_pixel(p_guess, H_inv, P, R_mat, R_cam, t_cam, intr)
            if uv is None:
                continue
            ui, vi = int(round(uv[0])), int(round(uv[1]))
            if ui < 0 or vi < 0 or ui >= W or vi >= H:
                continue
            if (r, c) in newly_cells:
                color = (0, 255, 0)        # 绿=本批新增
            elif covered[r][c]:
                color = (255, 255, 0)      # 青=既有覆盖
            else:
                color = (0, 0, 255)        # 红=未覆盖
            cv2.circle(overlay, (ui, vi), 4, color, -1)
            cv2.circle(overlay, (ui, vi), 4, (0, 0, 0), 1)
    save_path = None
    try:
        save_path = os.path.join(save_dir, f"tly_place_coverage_batch{batch}.png")
        cv2.imwrite(save_path, overlay)
    except Exception:
        save_path = None
    print("  📷 覆盖调试图：绿=本批新增, 青=既有, 红=未覆盖；在【图窗】按任意键继续。"
          + (f" 已存 {save_path}" if save_path else ""))
    win = "place coverage  green=new cyan=covered red=uncovered  (press any key)"
    cv2.imshow(win, overlay)
    cv2.waitKey(0)
    cv2.destroyWindow(win)


def select_roi_interactive(depth_m, display_img=None):
    """在底图上拖一个矩形 ROI。display_img 为彩色底图(默认用深度伪彩)。返回 [u0,v0,w,h] 或 None。"""
    if cv2 is None:
        raise RuntimeError("cv2 不可用，无法交互框选；请用 ~board_roi 给定矩形")
    preview = display_img if display_img is not None else depth_preview(depth_m)
    win = "select board ROI (drag a box, ENTER=ok, c=cancel)"
    r = cv2.selectROI(win, preview, showCrosshair=False, fromCenter=False)
    cv2.destroyWindow(win)
    u0, v0, w, h = [int(v) for v in r]
    if w <= 0 or h <= 0:
        return None
    return [u0, v0, w, h]


def resolve_roi(depth_m, prior_roi, param_roi, force_interactive, display_img=None):
    """优先级: force_interactive > ~board_roi 参数 > 持久化 ROI > 交互框选。"""
    if not force_interactive:
        if param_roi is not None:
            return list(param_roi), "param"
        if prior_roi is not None:
            return list(prior_roi), "persisted"
    roi = select_roi_interactive(depth_m, display_img=display_img)
    if roi is None:
        raise RuntimeError("未选择有效 ROI")
    return roi, "interactive"


def deproject_roi(depth_m, roi, z_range, intr):
    """ROI + 深度范围门 → Nx3 相机系点（光学系：x右 y下 z前）。"""
    fx, fy, cx, cy = intr
    u0, v0, w, h = roi
    H, W = depth_m.shape
    u0 = max(0, min(int(u0), W - 1))
    v0 = max(0, min(int(v0), H - 1))
    u1 = max(u0 + 1, min(u0 + int(w), W))
    v1 = max(v0 + 1, min(v0 + int(h), H))
    sub = depth_m[v0:v1, u0:u1]
    us, vs = np.meshgrid(np.arange(u0, u1), np.arange(v0, v1))
    z = sub.reshape(-1).astype(np.float64)
    us = us.reshape(-1).astype(np.float64)
    vs = vs.reshape(-1).astype(np.float64)
    zmin, zmax = float(z_range[0]), float(z_range[1])
    m = (z > max(1e-3, zmin)) & (z < zmax)
    z, us, vs = z[m], us[m], vs[m]
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def ransac_plane(points, thresh=0.004, iters=300, min_inliers_frac=0.3, seed=0):
    """RANSAC 主平面 → (normal, centroid, inlier_mask)，内点上 PCA 复拟合。"""
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    if n < 50:
        raise RuntimeError("ROI 内有效深度点太少(%d)，调整 ROI/深度范围" % n)
    rng = np.random.default_rng(seed)
    best_inliers, best_count = None, -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        inliers = np.abs((pts - p0).dot(nrm)) < thresh
        c = int(np.count_nonzero(inliers))
        if c > best_count:
            best_count, best_inliers = c, inliers
    if best_inliers is None or best_count < max(50, int(min_inliers_frac * n)):
        raise RuntimeError("RANSAC 未找到稳定平面(inliers=%d/%d)" % (best_count, n))
    inl = pts[best_inliers]
    centroid = inl.mean(axis=0)
    _, _, vh = np.linalg.svd(inl - centroid, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    return normal, centroid, best_inliers


def lookup_R_t(tf_buffer, target_frame, source_frame, timeout=3.0):
    """返回 source→target 的 (R 3x3, t 3)，即 point_target = R·point_source + t。"""
    tr = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(timeout))
    q = tr.transform.rotation
    R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    t = np.array([tr.transform.translation.x,
                  tr.transform.translation.y,
                  tr.transform.translation.z], dtype=float)
    return R, t


def project_board_to_pixel(p_board, P, R_mat, R_cam, t_cam, intr):
    """board 系点 → 像素 (u,v)。R_cam,t_cam 为 camera→base。光心后方/非法返回 None。"""
    p_base = R_mat.dot(np.asarray(p_board, dtype=float)) + P
    p_cam = R_cam.T.dot(p_base - t_cam)
    if p_cam[2] <= 1e-6:
        return None
    fx, fy, cx, cy = intr
    return (fx * p_cam[0] / p_cam[2] + cx, fy * p_cam[1] / p_cam[2] + cy)


def pixel_depth_to_board(u, v, z_meas, P, R_mat, R_cam, t_cam, intr):
    """像素 (u,v) 处实测深度 z_meas(米) → board 系 3D 点（取其 z 即该处表面高度）。"""
    fx, fy, cx, cy = intr
    x = (u - cx) * z_meas / fx
    y = (v - cy) * z_meas / fy
    p_cam = np.array([x, y, z_meas], dtype=float)
    p_base = R_cam.dot(p_cam) + t_cam
    return R_mat.T.dot(p_base - P)


def sample_window_median_depth(depth_m, u, v, half=2, min_valid=5):
    """像素 (u,v) 周围 (2*half+1)^2 窗口取有效(>0)深度中值；有效点不足或越界返回 None。"""
    H, W = depth_m.shape
    ui, vi = int(round(u)), int(round(v))
    if ui < 0 or vi < 0 or ui >= W or vi >= H:
        return None
    u0, u1 = max(0, ui - half), min(W, ui + half + 1)
    v0, v1 = max(0, vi - half), min(H, vi + half + 1)
    win = depth_m[v0:v1, u0:u1].reshape(-1)
    win = win[win > 0]
    if win.size < min_valid:
        return None
    return float(np.median(win))


def load_homography_inv(tcfg):
    """从 PICK_HOMOGRAPHY(pixel->board-XY, 9 数行主序 3x3) 取逆 → board-XY->pixel。
    返回 3x3 ndarray 或 None。单应性是【拍照位姿专属】的平面映射，绕开手眼 3D 投影。"""
    h = tcfg.get("PICK_HOMOGRAPHY") if isinstance(tcfg, dict) else None
    if not isinstance(h, dict) or not h.get("enabled"):
        return None
    m = h.get("matrix")
    if not m or len(m) != 9:
        return None
    try:
        return np.linalg.inv(np.asarray(m, dtype=float).reshape(3, 3))
    except Exception:
        return None


def board_xy_to_pixel(bx, by, H_inv):
    """board-XY → 像素 (u,v)，经 PICK_HOMOGRAPHY 的逆。要求相机处于 affine/拍照位姿。"""
    p = H_inv.dot(np.array([float(bx), float(by), 1.0], dtype=float))
    if abs(p[2]) < 1e-9:
        return None
    return (p[0] / p[2], p[1] / p[2])


def cell_center_pixel(p_board, H_inv, P, R_mat, R_cam, t_cam, intr):
    """格心 board 点 → 像素。有单应性 H_inv(board-XY->pixel)就用它(平面映射, 不经手眼)，否则回退手眼 3D 投影。"""
    if H_inv is not None:
        return board_xy_to_pixel(p_board[0], p_board[1], H_inv)
    return project_board_to_pixel(p_board, P, R_mat, R_cam, t_cam, intr)


def place_homography_from_clicks(color_img, origin, row_vec, col_vec, rows, cols):
    """在当前彩色图上点选白板网格【四个角格】的凸起中心，配合已知 board-XY 拟合 board-XY->pixel 单应性。
    pose+region 都正确(当前机位、白板本身)，绕开手眼和抓取单应性的外推误差。带放大镜便于精确点选。
    返回 3x3 (board-XY->pixel) 或 None。"""
    if cv2 is None:
        return None
    corners = [(0, 0), (0, cols - 1), (rows - 1, cols - 1), (rows - 1, 0)]
    names = ["左上(r0,c0)", f"右上(r0,c{cols-1})", f"右下(r{rows-1},c{cols-1})", f"左下(r{rows-1},c0)"]
    clicks = []
    st = {"x": 0, "y": 0}

    def on_mouse(event, x, y, flags, param):
        st["x"], st["y"] = x, y
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < len(corners):
            clicks.append((x, y))
            print(f"  ✅ 角{len(clicks)} {names[len(clicks)-1]} pixel=({x},{y})")
        elif event == cv2.EVENT_RBUTTONDOWN and clicks:
            clicks.pop()
            print("  ↩️ 撤销上一个角点")

    win = "click 4 grid CORNER bumps: TL -> TR -> BR -> BL (right=undo, Enter=ok, q=skip)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    print("\n  按【左上→右上→右下→左下】依次点白板网格四角的凸起中心(右上角有放大镜)；右键撤销，点满4个回车确认，q跳过。")
    H_full, W_full = color_img.shape[:2]
    while not rospy.is_shutdown():
        disp = color_img.copy()
        for i, (u, v) in enumerate(clicks):
            cv2.drawMarker(disp, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(disp, str(i + 1), (u + 7, v - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # 放大镜
        mh, ms = 26, 7
        mx, my = st["x"], st["y"]
        x0, y0 = max(0, mx - mh), max(0, my - mh)
        x1, y1 = min(W_full, mx + mh), min(H_full, my + mh)
        patch = color_img[y0:y1, x0:x1]
        if patch.size > 0:
            zoom = cv2.resize(patch, (0, 0), fx=ms, fy=ms, interpolation=cv2.INTER_NEAREST)
            zx, zy = int((mx - x0) * ms), int((my - y0) * ms)
            cv2.drawMarker(zoom, (zx, zy), (0, 255, 0), cv2.MARKER_CROSS, 22, 1)
            zh, zw = zoom.shape[:2]
            ox = disp.shape[1] - zw - 10
            if ox > 0 and zh + 10 < disp.shape[0]:
                disp[10:10 + zh, ox:ox + zw] = zoom
                cv2.rectangle(disp, (ox, 10), (ox + zw, 10 + zh), (0, 255, 0), 2)
        nxt = names[len(clicks)] if len(clicks) < len(corners) else "完成→回车"
        cv2.putText(disp, f"next: {nxt}  ({len(clicks)}/4)", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(win, disp)
        k = cv2.waitKey(30) & 0xFF
        if k in (13, 10) and len(clicks) == len(corners):
            break
        if k == ord("q"):
            cv2.destroyWindow(win)
            return None
    cv2.destroyWindow(win)
    board_xy = np.array([point_on_grid(origin, row_vec, col_vec, r, c)[:2] for (r, c) in corners], dtype=np.float64)
    pix = np.array(clicks, dtype=np.float64)
    H, _ = cv2.findHomography(board_xy, pix)   # board-XY -> pixel
    if H is None:
        print("  ⚠️ 单应性拟合失败，回退。")
        return None
    # 残差自检
    res = []
    for (bx, by), (pu, pv) in zip(board_xy, pix):
        q = H.dot([bx, by, 1.0]); q = q / q[2]
        res.append(((q[0] - pu) ** 2 + (q[1] - pv) ** 2) ** 0.5)
    print("  ✅ 点角单应性拟合完成(当前机位)，四角重投影残差 max=%.1f px。" % max(res))
    return H


def click_pixels(color_img, num_points, title="click points (Left=add Right=undo Enter=ok q=cancel)", min_points=None):
    """带放大镜在彩色图上点选像素点。左键加、右键撤销、回车确认、q 取消。返回 [(u,v),...] 或 None。
    min_points 为 None：须恰好点 num_points 个才可回车。否则为【可选点数】模式——num_points 作为上限，
    点够 min_points 个后即可回车结束（用于步骤7单应性：点数可变、仅要求 >=4）。"""
    if cv2 is None:
        return None
    clicks = []
    H_full, W_full = color_img.shape[:2]
    st = {"x": W_full // 2, "y": H_full // 2}

    def disp_to_image(x, y):
        # WINDOW_NORMAL 窗口被 WM 缩放时，鼠标回调给的是“显示坐标”而非图像坐标。
        # 按当前实际显示尺寸换算回全分辨率图像坐标，否则记录的像素被缩放带偏，
        # 污染单应性标定（运行时检测像素未缩放 → 系统性、随位置变化的吸偏）。
        try:
            _, _, rw, rh = cv2.getWindowImageRect(title)
            if rw > 0 and rh > 0:
                x = x * W_full / float(rw)
                y = y * H_full / float(rh)
        except Exception:
            pass
        ix = int(max(0, min(W_full - 1, round(x))))
        iy = int(max(0, min(H_full - 1, round(y))))
        return ix, iy

    def on_mouse(event, x, y, flags, param):
        ix, iy = disp_to_image(x, y)
        st["x"], st["y"] = ix, iy
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < num_points:
            clicks.append((ix, iy))
            print(f"  ✅ P{len(clicks)} pixel=({ix},{iy})")
        elif event == cv2.EVENT_RBUTTONDOWN and clicks:
            clicks.pop()
            print("  ↩️ 撤销上一个点")

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(title, on_mouse)
    while not rospy.is_shutdown():
        disp = color_img.copy()
        for i, (u, v) in enumerate(clicks):
            cv2.drawMarker(disp, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 15, 2)
            cv2.circle(disp, (u, v), 9, (0, 0, 255), 2)
            cv2.putText(disp, f"P{i+1}", (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        mh, ms = 30, 6
        mx, my = st["x"], st["y"]
        x0, y0 = max(0, mx - mh), max(0, my - mh)
        x1, y1 = min(W_full, mx + mh), min(H_full, my + mh)
        patch = color_img[y0:y1, x0:x1]
        if patch.size > 0:
            z = cv2.resize(patch, (0, 0), fx=ms, fy=ms, interpolation=cv2.INTER_NEAREST)
            cv2.drawMarker(z, (int((mx - x0) * ms), int((my - y0) * ms)), (0, 255, 0), cv2.MARKER_CROSS, 20, 1)
            zh, zw = z.shape[:2]
            ox = disp.shape[1] - zw - 10
            if ox > 0 and zh + 10 < disp.shape[0]:
                disp[10:10 + zh, ox:ox + zw] = z
                cv2.rectangle(disp, (ox, 10), (ox + zw, 10 + zh), (0, 255, 0), 2)
        hud = (f"{len(clicks)} pts (min {min_points} max {num_points})" if min_points is not None
               else f"{len(clicks)}/{num_points}")
        cv2.putText(disp, hud + "  Left=add Right=undo i=type(u,v) Enter=ok q=cancel",
                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(title, disp)
        k = cv2.waitKey(30) & 0xFF
        enough = (len(clicks) >= min_points) if min_points is not None else (len(clicks) == num_points)
        if k in (13, 10) and enough:
            break
        if k == ord("q"):
            cv2.destroyWindow(title)
            return None
        if k == ord("i") and len(clicks) < num_points:
            # 可选：手动键入像素坐标（如从 realsense-viewer 读数直接输入），彻底绕开点选/窗口缩放。
            # 注意：键入时焦点要切到终端。
            try:
                s = input(f"  ⌨️  输入像素 u,v（逗号或空格分隔，范围 0..{W_full-1}, 0..{H_full-1}；空回车取消）: ").strip()
            except EOFError:
                s = ""
            if s:
                try:
                    parts = s.replace(",", " ").split()
                    u, v = int(round(float(parts[0]))), int(round(float(parts[1])))
                    if 0 <= u < W_full and 0 <= v < H_full:
                        clicks.append((u, v))
                        st["x"], st["y"] = u, v
                        print(f"  ✅ P{len(clicks)} pixel=({u},{v})  [手动输入]")
                    else:
                        print("  ⚠️ 越界，已忽略。")
                except (ValueError, IndexError):
                    print("  ⚠️ 格式无效，示例：640 360 或 640,360")
    cv2.destroyWindow(title)
    return clicks


def fit_plane_capture(tf_buffer, depth_m, roi, z_range, intr, camera_frame, base_frame,
                      thresh, iters, min_inliers_frac=0.3):
    """从一张(已多帧中值的)深度图拟板面，并转入 base 系。返回结果字典。"""
    pts_cam = deproject_roi(depth_m, roi, z_range, intr)
    normal_cam, centroid_cam, inliers = ransac_plane(pts_cam, thresh=thresh, iters=iters,
                                                     min_inliers_frac=min_inliers_frac)
    R, t = lookup_R_t(tf_buffer, base_frame, camera_frame)   # camera → base
    inl_cam = pts_cam[inliers]
    inl_base = inl_cam.dot(R.T) + t
    centroid_base = centroid_cam.dot(R.T) + t
    normal_base = R.dot(normal_cam)
    normal_base = normal_base / np.linalg.norm(normal_base)
    if normal_base[2] < 0:                     # link_base +Z 大致向上
        normal_base = -normal_base
        normal_cam = -normal_cam
    signed = (inl_base - centroid_base).dot(normal_base)
    return {
        "normal_cam": normal_cam, "centroid_cam": centroid_cam,
        "normal_base": normal_base, "centroid_base": centroid_base,
        "inliers_base": inl_base,
        "inlier_count": int(len(inl_base)), "total": int(len(pts_cam)),
        "rms": float(np.sqrt(np.mean(signed ** 2))),
        "max_abs": float(np.max(np.abs(signed))),
        "cam_dist": float(normal_cam.dot(centroid_cam)),   # 相机原点→平面的有符号距离
        "cam_R": R, "cam_t": t,
    }


def verify_motion_report(first, second):
    """走位差分：用两个机位的深度+TF 互校，估深度尺度、法向一致性、base-Z↔法向夹角。"""
    c1, c2 = first["cam_t"], second["cam_t"]
    dmove = c2 - c1
    n = first["normal_base"]
    move_along_normal = float(n.dot(dmove))
    meas = abs(second["cam_dist"]) - abs(first["cam_dist"])
    # 垂直距离 D=(c−p)·n，故 ΔD=dmove·n=move_along_normal（沿 +法向远离→变大，靠近→变小）。
    # 深度尺度 = 深度测得 ΔD(meas) / TF 算得 ΔD(pred)，应 ≈ +1.0。
    pred = move_along_normal
    scale = float(meas / pred) if abs(pred) > 1e-4 else float("nan")
    cosc = float(np.clip(abs(first["normal_base"].dot(second["normal_base"])), -1.0, 1.0))
    return {
        "move_base_m": [float(v) for v in dmove],
        "move_norm_m": float(np.linalg.norm(dmove)),
        "move_along_normal_m": move_along_normal,
        "measured_cam_dist_delta_m": float(meas),
        "predicted_cam_dist_delta_m": float(pred),
        "depth_scale_ratio": scale,
        "normal_consistency_deg": float(np.degrees(np.arccos(cosc))),
        "baseZ_to_normal_deg": float(np.degrees(np.arccos(np.clip(abs(n[2]), -1.0, 1.0)))),
    }


def run_depth_offset_check(tf_buffer, base_frame, eef_frame, tcfg, config_path, existing_cfg, n_points):
    """触点↔深度 Z 偏置自检：触 n 个【空发光板】点(TF 读 link_base<-link_tcp，与标定同一把尺子)，
    拟触点平面，与步骤2的深度平面比【法向夹角】+【沿法向有符号 Z 偏移】，并把偏移存进
    DEPTH_TOUCH_Z_OFFSET。用来量"深度绝对 tare 偏置 + 手眼平移"造成的恒定 Z 偏差。"""
    n_dep = np.asarray(cfg_get(tcfg, "BOARD_SURFACE_NORMAL_BASE", "2(深度平面)"), dtype=float)
    ctr_dep = np.asarray(cfg_get(tcfg, "BOARD_SURFACE_CENTROID_BASE", "2(深度平面)"), dtype=float)
    n_dep = n_dep / np.linalg.norm(n_dep)

    print("\n" + "=" * 72)
    print("=== 触点↔深度 Z 偏置自检（TF link_base<-link_tcp，与标定/执行同一把尺子）===")
    print(f"= 在【空发光板】上触 {n_points} 个点，尽量铺开。输入 'u' 撤销上一个。")
    print("=" * 72)
    pts = []
    i = 0
    while i < n_points:
        p = require_pose(tf_buffer, f"[自检 {i+1}/{n_points}] 吸嘴尖轻触【空发光板】某点",
                         base_frame, eef_frame, allow_undo=(i > 0))
        if isinstance(p, str) and p == "UNDO":
            i -= 1
            pts.pop()
            print("  ↩️ 已撤销，重触上一个点。")
            continue
        pts.append(p)
        i += 1
    pts = np.asarray(pts, dtype=float)

    ctr_t = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - ctr_t)
    n_t = vh[-1] / np.linalg.norm(vh[-1])
    if n_t[2] < 0:
        n_t = -n_t
    resid_t = (pts - ctr_t).dot(n_t)
    ang = float(np.degrees(np.arccos(np.clip(abs(n_dep.dot(n_t)), -1.0, 1.0))))
    d = (pts - ctr_dep).dot(n_dep)          # 触点到深度平面有符号距离(沿深度法向)
    offset = float(np.mean(d))

    print("\n--- 结果 ---")
    print("  触点平面拟合残差 std/max: {:.2f} / {:.2f} mm".format(
        resid_t.std() * 1000, np.abs(resid_t).max() * 1000))
    print("  触点法向 vs 深度法向 夹角: {:.3f}°".format(ang))
    print("  各触点到深度平面(沿法向)有符号距离 (mm):")
    for p, dd in zip(pts, d):
        print("    [{:+.3f} {:+.3f} {:+.3f}] : {:+.2f} mm".format(p[0], p[1], p[2], dd * 1000))
    print("  >> 偏移均值 = {:+.2f} mm  (负=真实触点面在深度平面【下方】，即深度把面读高了)".format(offset * 1000))
    print("     std = {:.2f} mm  (大则还有残余倾斜/触点噪声)".format(float(np.std(d)) * 1000))

    existing_cfg = write_config_merge(config_path, existing_cfg, {
        "DEPTH_TOUCH_Z_OFFSET": {
            "value_m": offset,   # = mean((touch − depth_centroid)·n_dep)，负=深度平面在真实面上方
            "convention": "corrected_board_z = depth_board_z + value_m  (value_m<0 lowers depth heights to touch/kinematics datum)",
            "n_points": int(n_points),
            "fit_std_m": float(np.std(d)),
            "normal_angle_deg": ang,
            "touch_plane_resid_std_m": float(resid_t.std()),
        }
    })
    print("\n  💾 已存 DEPTH_TOUCH_Z_OFFSET.value_m = {:+.5f} m 到 config。".format(offset))
    print("=" * 72)


def parse_cell_overrides(tcfg):
    """读 BOARD_CELL_OVERRIDES_BOARD（稀疏列表，每项 {col,row,board:[x,y,(z)]}）→ dict[(row,col)]=[x,y,z]。
    board 为该格在 board 系的覆盖坐标；放置只用 XY。非法项忽略。"""
    out = {}
    ov = tcfg.get("BOARD_CELL_OVERRIDES_BOARD") if isinstance(tcfg, dict) else None
    if not isinstance(ov, list):
        return out
    for rec in ov:
        try:
            r = int(rec["row"])
            c = int(rec["col"])
            b = [float(v) for v in rec["board"]]
            if 0 <= r < ROWS and 0 <= c < COLS and len(b) >= 2:
                out[(r, c)] = b
        except Exception:
            continue
    return out


def run_edit_cell_centers(tf_buffer, base_frame, eef_frame, tcfg, config_path, existing_cfg):
    """单独修改白板【放置格心】(BOARD_CENTERS_14x10_BOARD)：交互选 (col,row)，吸嘴对准该格凸起中心
    回车读 TF（link_base<-link_eef）→ 转 board 系覆盖该格 XY；覆盖量累加进 BOARD_CELL_OVERRIDES_BOARD
    （稀疏），并即时重写 BOARD_CENTERS_14x10_BOARD。只改 XY（z=凸起高度、放置深度 PLACE_Z_MAP 不变）。
    需已有步骤 2/3 的坐标系+网格；重跑步骤 3 会重派生网格并清空这些逐格覆盖。"""
    bp = cfg_get(tcfg, "BOARD_POSE_BASE", "3")
    P = np.asarray(bp["origin"], dtype=float)
    rpy = tuple(float(v) for v in bp["rpy"])
    R_mat = euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")[:3, :3]
    origin = np.asarray(cfg_get(tcfg, "BOARD_ORIGIN_BOARD", "3"), dtype=float)
    row_vec = np.asarray(cfg_get(tcfg, "BOARD_ROW_STEP_BOARD", "3"), dtype=float)
    col_vec = np.asarray(cfg_get(tcfg, "BOARD_COL_STEP_BOARD", "3"), dtype=float)
    bump_height_map = cfg_get(tcfg, "BOARD_BUMP_HEIGHT_MAP_14x10", "3")

    overrides = parse_cell_overrides(tcfg)   # dict[(row,col)] = [x,y,z]（board 系）

    def build_centers():
        """仿射网格 XY + 逐格覆盖 → 140 格心；z 一律用凸起高度图（覆盖不改 z）。"""
        centers = []
        for r in range(ROWS):
            line = []
            for c in range(COLS):
                p = point_on_grid(origin, row_vec, col_vec, r, c)
                cx, cy = float(p[0]), float(p[1])
                if (r, c) in overrides:
                    ov = overrides[(r, c)]
                    cx, cy = float(ov[0]), float(ov[1])
                line.append([cx, cy, float(bump_height_map[r][c])])
            centers.append(line)
        return centers

    def overrides_to_records():
        recs = []
        for (r, c), b in sorted(overrides.items()):
            z = float(b[2]) if len(b) > 2 else float(bump_height_map[r][c])
            recs.append({"col": int(c), "row": int(r),
                         "board": [float(b[0]), float(b[1]), z]})
        return recs

    def persist(cfg):
        return write_config_merge(config_path, cfg, {
            "BOARD_CELL_OVERRIDES_BOARD": overrides_to_records(),
            "BOARD_CENTERS_14x10_BOARD": build_centers(),
        })

    print("\n" + "=" * 72)
    print("=== 单独修改白板放置格心 (BOARD_CENTERS_14x10_BOARD) ===")
    print("= 选 (col,row) → 吸嘴对准该格凸起中心回车读 TF → 覆盖该格放置 XY。")
    print("= 只改 XY（放置朝向/深度不变：z=凸起高度, 放置深度仍用步骤5 PLACE_Z_MAP）。")
    print(f"= 网格尺寸 {COLS} 列 x {ROWS} 行，col∈[0,{COLS-1}], row∈[0,{ROWS-1}]。")
    print(f"= 当前已有 {len(overrides)} 个格被覆盖。命令：list=查看, undo=撤销上一次, q=结束。")
    print("=" * 72)

    history = []   # 撤销栈：(row, col, prev_override_or_None)
    while not rospy.is_shutdown():
        ans = input("\n选择要修改的格 col,row（或 list / undo / q）: ").strip().lower()
        if ans in ("q", "quit", "结束", ""):
            break
        if ans in ("list", "ls", "l"):
            if not overrides:
                print("  （暂无覆盖）")
            for (r, c), b in sorted(overrides.items()):
                p = point_on_grid(origin, row_vec, col_vec, r, c)
                print("  (c{},r{}) 覆盖 XY=[{:.4f}, {:.4f}]  网格 XY=[{:.4f}, {:.4f}]  Δ=[{:+.1f}, {:+.1f}] mm".format(
                    c, r, b[0], b[1], p[0], p[1], (b[0] - p[0]) * 1000.0, (b[1] - p[1]) * 1000.0))
            continue
        if ans in ("u", "undo", "撤销"):
            if not history:
                print("  无可撤销。")
                continue
            r, c, prev = history.pop()
            if prev is None:
                overrides.pop((r, c), None)
                print(f"  ↩️ 已撤销 (c{c},r{r}) 的修改（恢复为网格 XY）。")
            else:
                overrides[(r, c)] = prev
                print(f"  ↩️ 已撤销 (c{c},r{r}) 的修改（恢复上一次覆盖值）。")
            existing_cfg = persist(existing_cfg)
            print("  💾 已写入 config。")
            continue

        parts = [p for p in ans.replace("，", ",").replace(",", " ").split() if p]
        if len(parts) != 2:
            print("  请输入两个整数：col,row（例如 3,5）。")
            continue
        try:
            col, row = int(parts[0]), int(parts[1])
        except ValueError:
            print("  col/row 必须是整数。")
            continue
        if not (0 <= col < COLS and 0 <= row < ROWS):
            print(f"  越界：col∈[0,{COLS-1}], row∈[0,{ROWS-1}]。")
            continue

        p_grid = point_on_grid(origin, row_vec, col_vec, row, col)
        cur = overrides.get((row, col))
        if cur is not None:
            print("  当前覆盖 XY=[{:.4f}, {:.4f}]（网格 XY=[{:.4f}, {:.4f}]）".format(
                cur[0], cur[1], p_grid[0], p_grid[1]))
        else:
            print("  当前用网格 XY=[{:.4f}, {:.4f}]".format(p_grid[0], p_grid[1]))

        p_base = require_pose(tf_buffer, f"  吸嘴对准格 (col={col}, row={row}) 的凸起中心",
                              base_frame, eef_frame, allow_undo=True)
        if isinstance(p_base, str) and p_base == "UNDO":
            print("  ↩️ 放弃本次修改（未改动该格）。")
            continue

        b = to_table(R_mat, P, p_base)   # board 系 [x,y,z]
        prev = overrides.get((row, col))
        history.append((row, col, list(prev) if prev is not None else None))
        overrides[(row, col)] = [float(b[0]), float(b[1]), float(b[2])]
        dx = (b[0] - p_grid[0]) * 1000.0
        dy = (b[1] - p_grid[1]) * 1000.0
        print("  ✅ (c{},r{}) 新 XY=[{:.4f}, {:.4f}]  相对网格 Δ=[{:+.1f}, {:+.1f}] mm".format(
            col, row, b[0], b[1], dx, dy))
        existing_cfg = persist(existing_cfg)
        print("  💾 已写入 config（即时落盘）。")

    existing_cfg = persist(existing_cfg)
    print(f"\n🎉 完成：共 {len(overrides)} 个格被覆盖，已写入 BOARD_CELL_OVERRIDES_BOARD 和 "
          "BOARD_CENTERS_14x10_BOARD。")
    print("=" * 72)


def parse_cell_spec(spec, rows=ROWS, cols=COLS):
    """把格子选择串解析成有序 [(row,col), ...]（走位顺序即此顺序）。支持：
       'all' / 空 → 全部 rows*cols 格，蛇形 row-major（减少往返）；
       'r<idx>' → 整行（如 r3）；'c<idx>' → 整列（如 c5）；
       'col,row' → 单格（按 col,row 顺序，与标定提示一致）；
       多个 token 用 ';' 或空白分隔。越界/非法 token 跳过，重复格去重。"""
    spec = str(spec).strip().lower()

    def snake_all():
        out = []
        for r in range(rows):
            crange = range(cols) if r % 2 == 0 else range(cols - 1, -1, -1)
            for c in crange:
                out.append((r, c))
        return out

    if spec in ("", "all", "全部"):
        return snake_all()

    out = []
    seen = set()

    def add(r, c):
        if 0 <= r < rows and 0 <= c < cols and (r, c) not in seen:
            seen.add((r, c))
            out.append((r, c))

    for tok in spec.replace("；", ";").replace("，", ",").replace(";", " ").split():
        tok = tok.strip()
        if not tok:
            continue
        if tok in ("all", "全部"):
            for r, c in snake_all():
                add(r, c)
        elif tok.startswith("r") and tok[1:].isdigit():
            r = int(tok[1:])
            for c in range(cols):
                add(r, c)
        elif tok.startswith("c") and tok[1:].isdigit():
            c = int(tok[1:])
            for r in range(rows):
                add(r, c)
        elif "," in tok:
            a, b = tok.split(",", 1)
            try:
                col, row = int(a), int(b)
            except ValueError:
                continue
            add(row, col)
    return out


def run_cell_tour(tf_buffer, base_frame, eef_frame, tcfg):
    """步骤8（独立验证模式）：驱动机械臂依次悬停到白板各格中心【上方设定高度】，肉眼核对标定 XY 是否对准。
    与生产控制器同一条路径：原生 /xarm/move_line（set_mode 0 / set_state 0），位姿用 BOARD_POSE_BASE
    把 board 系格心换算到 base 系，工具朝下（roll=π，与放置一致）。只读取标定结果，不修改任何配置。
    交互：按回车走下一个格；支持选定格子（all / r<行> / c<列> / 'col,row;...'）与设置每格上方高度。"""
    import math
    try:
        from xarm_msgs.srv import SetInt16, Move, SetAxis, TCPOffset
        from xarm_msgs.msg import RobotMsg
    except Exception as e:
        print(f"❌ 无法导入 xarm_msgs（确认已 catkin_make 且 source devel/setup.bash）：{e}")
        return

    # ---- 读取标定几何（与控制器消费的同一批键）----
    bp = tcfg.get("BOARD_POSE_BASE")
    centers = tcfg.get("BOARD_CENTERS_14x10_BOARD")
    if not isinstance(bp, dict) or "origin" not in bp or "rpy" not in bp:
        print("❌ 配置缺少 BOARD_POSE_BASE，请先完成步骤 3。")
        return
    if not isinstance(centers, list) or not centers or not isinstance(centers[0], list):
        print("❌ 配置缺少 BOARD_CENTERS_14x10_BOARD，请先完成步骤 3。")
        return
    P = np.asarray(bp["origin"], dtype=float)
    rpy = [float(v) for v in bp["rpy"]]
    R_bb = euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")[:3, :3]   # board -> base 旋转
    rows = len(centers)
    cols = len(centers[0])

    # 每格基准 Z：默认用格心凸起顶面（board_centers 的 z）；verify_z_ref=place 时改用 PLACE_Z_MAP（放置面）。
    z_ref = str(rospy.get_param("~verify_z_ref", "bump")).strip().lower()
    place_map = tcfg.get("PLACE_Z_MAP_14x10") if z_ref == "place" else None
    if z_ref == "place" and (not isinstance(place_map, list) or len(place_map) != rows):
        print("  ⚠️ verify_z_ref=place 但缺 PLACE_Z_MAP_14x10，回退用凸起顶面作基准。")
        place_map = None

    cell_xyz = []
    for r in range(rows):
        line = []
        for c in range(cols):
            x = float(centers[r][c][0])
            y = float(centers[r][c][1])
            top = float(place_map[r][c]) if place_map is not None else float(centers[r][c][2])
            line.append((x, y, top))
        cell_xyz.append(line)
    max_top = max(cell_xyz[r][c][2] for r in range(rows) for c in range(cols))

    # ---- 参数 ----
    height_above = float(rospy.get_param("~verify_height", 0.05))
    spec = str(rospy.get_param("~verify_cells_spec", "all"))
    speed = float(rospy.get_param("~verify_speed", 60.0))
    acc = float(rospy.get_param("~verify_acc", 300.0))
    travel_clear = float(rospy.get_param("~verify_travel_clearance", 0.08))
    do_set_tcp = bool(rospy.get_param("~verify_set_tcp_offset", True))
    # 工具下扎朝向（board 系）：roll=π 让工具 Z 指向板内（朝下），与放置朝向一致。
    tool_R = euler_matrix(math.pi, 0.0, 0.0, axes="sxyz")[:3, :3]

    # ---- xArm 原生服务（calibrate_tool.launch 的 realMove_exec 已起 ns=xarm 的驱动）----
    print("\n等待 xArm 原生服务 /xarm/move_line ...")
    try:
        rospy.wait_for_service("/xarm/move_line", timeout=15.0)
    except Exception:
        print("❌ 未发现 /xarm/move_line（确认机器人在线、robot_ip 正确）。")
        return
    set_mode = rospy.ServiceProxy("/xarm/set_mode", SetInt16)
    set_state = rospy.ServiceProxy("/xarm/set_state", SetInt16)
    motion_ctrl = rospy.ServiceProxy("/xarm/motion_ctrl", SetAxis)
    move_line = rospy.ServiceProxy("/xarm/move_line", Move)
    set_tcp = rospy.ServiceProxy("/xarm/set_tcp_offset", TCPOffset)

    # 当前原生位姿（解缠 roll/pitch/yaw，避免手腕兜大圈）。
    native = {"pose": None}

    def on_state(msg):
        if len(msg.pose) >= 6:
            native["pose"] = list(msg.pose[:6])

    rospy.Subscriber("/xarm/xarm_states", RobotMsg, on_state)
    try:
        rospy.wait_for_message("/xarm/xarm_states", RobotMsg, timeout=5.0)
    except Exception:
        print("  ⚠️ 暂未收到 /xarm/xarm_states；首个移动不做角度解缠（影响不大）。")

    def unwrap(cur, tgt):
        d = tgt - cur
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return cur + d

    # ---- 设工具坐标偏置 = 吸盘尖（link_tcp），让 move_line 控制 link_tcp（与标定测量的点一致）----
    # link_eef ≡ link6（joint_eef 为单位变换），故 firmware 工具偏置=TF(link_eef→eef_frame)。
    if do_set_tcp:
        try:
            tr = tf_buffer.lookup_transform("link_eef", eef_frame, rospy.Time(0), rospy.Duration(3.0))
            ox = tr.transform.translation.x * 1000.0
            oy = tr.transform.translation.y * 1000.0
            oz = tr.transform.translation.z * 1000.0
            resp = set_tcp(ox, oy, oz, 0.0, 0.0, 0.0)
            if getattr(resp, "ret", 0) != 0:
                print(f"  ⚠️ set_tcp_offset 返回 ret={resp.ret}；沿用控制器内已保存的工具偏置。")
            else:
                print("  已设 xArm 工具偏置(TCP) = link_eef→{} = ({:.2f}, {:.2f}, {:.2f}) mm，"
                      "move_line 控制 {}（已存盘，持久生效）。".format(eef_frame, ox, oy, oz, eef_frame))
        except Exception as e:
            print(f"  ⚠️ 读 TF/设工具偏置失败（{e}）；沿用控制器内已保存的工具偏置。")

    def board_to_native(bx, by, bz):
        pos_base = R_bb.dot(np.array([bx, by, bz], dtype=float)) + P
        M = np.eye(4)
        M[:3, :3] = R_bb.dot(tool_R)
        r, p, y = euler_from_matrix(M, axes="sxyz")
        cur = native["pose"]
        if cur is not None:
            r = unwrap(cur[3], r)
            p = unwrap(cur[4], p)
            y = unwrap(cur[5], y)
        return [pos_base[0] * 1000.0, pos_base[1] * 1000.0, pos_base[2] * 1000.0, r, p, y]

    def do_move(bx, by, bz, label):
        pose = board_to_native(bx, by, bz)
        try:
            resp = move_line(pose, speed, acc, 0.0, 0.0)
        except Exception as e:
            print(f"  ❌ move_line 调用失败({label}): {e}")
            return False
        if getattr(resp, "ret", 0) != 0:
            print(f"  ❌ move_line 失败({label}): ret={resp.ret} msg={resp.message}")
            return False
        return True

    prev = {"xy": None}

    def goto(r, c):
        """up-over-down 安全走位：先在上一格 XY 升到安全平面，横移到目标 XY，再下降到目标上方高度。"""
        x, y, top = cell_xyz[r][c]
        hover_z = top + height_above
        safe_z = max(max_top + travel_clear, hover_z)   # 横移用的全局安全平面（board 系）
        if prev["xy"] is not None:
            if not do_move(prev["xy"][0], prev["xy"][1], safe_z, "lift"):
                return False
        if not do_move(x, y, safe_z, "over"):
            return False
        if not do_move(x, y, hover_z, "descend"):
            return False
        prev["xy"] = (x, y)
        return True

    # ---- 组装格子列表 ----
    cells = parse_cell_spec(spec, rows, cols)
    if not cells:
        print(f"⚠️ 选择 '{spec}' 解析为空，默认走全部。")
        cells = parse_cell_spec("all", rows, cols)

    print("\n" + "=" * 78)
    print("=== 步骤 8：白板格心巡检（驱动真机悬停核对标定 XY）===")
    print(f"= 网格 {cols} 列 x {rows} 行；本次将走 {len(cells)} 个格，悬停在每格上方 {height_above*1000:.0f} mm。")
    print(f"= 基准面: {'放置面 PLACE_Z_MAP' if place_map is not None else '格心凸起顶面'}；"
          f"速度 {speed:.0f} mm/s, 加速度 {acc:.0f} mm/s²。")
    print("= 交互命令：回车=下一个 | b=上一个 | r=重走本格 | g col,row=跳到某格 | h=改高度 |")
    print("=           sel=重选格子 | list=看剩余 | q=退出。工具下扎朝向同放置（朝下）。")
    print("=" * 78)
    print("⚠️ 该步骤会【自动驱动真机】移动。请确保白板上无方块、周围无障碍、急停在手边。")
    confirm = input("确认现场安全后，输入 GO 回车开始（其它任意输入=取消）: ").strip().lower()
    if confirm != "go":
        print("已取消，未移动机械臂。")
        return

    # 让 move_line 阻塞到运动完成再返回（驱动每次调用动态读此参数），保证回车后臂到位才提示下一步。
    rospy.set_param("/xarm/wait_for_finish", True)

    # 切到原生位置模式（calibrate_tool 默认起 MoveIt；这里接管为 xArm 原生 move_line）。
    try:
        motion_ctrl(8, 1)
        rospy.sleep(0.2)
        set_mode(0)
        rospy.sleep(0.2)
        set_state(0)
        rospy.sleep(0.3)
    except Exception as e:
        print(f"⚠️ 切换原生模式失败: {e}（仍尝试移动；若不动请检查 set_mode/set_state）。")

    # 用当前位姿(base→board)初始化走位锚点：让首个移动先在原地竖直抬到安全平面再横移，避免贴板斜插。
    cur = native["pose"]
    if cur is not None:
        base_pos = np.array([cur[0] / 1000.0, cur[1] / 1000.0, cur[2] / 1000.0], dtype=float)
        bxy = R_bb.T.dot(base_pos - P)
        prev["xy"] = (float(bxy[0]), float(bxy[1]))

    i = 0
    if not goto(*cells[i]):
        print("❌ 首个移动失败，终止巡检。检查机器人是否使能/有报错。")
        return

    while not rospy.is_shutdown():
        r, c = cells[i]
        cmd = input(
            f"\n[{i+1}/{len(cells)}] 现处于 (col={c}, row={r}) 上方 {height_above*1000:.0f} mm。"
            "回车=下一个 | b=上一个 | r=重走 | g col,row | h[=高度m] | sel | list | q: "
        ).strip().lower()

        if cmd in ("q", "quit", "退出"):
            break
        elif cmd in ("", "n", "next", "下一个"):
            if i + 1 >= len(cells):
                if ask_yes_no("已是最后一个，重头再走一遍？", default=False):
                    i = 0
                    goto(*cells[i])
                else:
                    break
            else:
                i += 1
                goto(*cells[i])
        elif cmd in ("b", "back", "上一个"):
            if i > 0:
                i -= 1
                goto(*cells[i])
            else:
                print("  已是第一个。")
        elif cmd in ("r", "redo", "重走"):
            goto(*cells[i])
        elif cmd in ("list", "ls"):
            rem = ", ".join(f"(c{cc},r{rr})" for rr, cc in cells[i + 1:i + 21])
            print(f"  剩余 {len(cells) - i - 1} 格，接下来：{rem}{' ...' if len(cells) - i - 1 > 20 else ''}")
        elif cmd.startswith("h"):
            rest = cmd[1:].strip().lstrip("=").strip()
            if rest == "":
                rest = input("  输入每格上方高度(米，例 0.05；也可填 mm 如 30): ").strip()
            try:
                hv = float(rest)
            except ValueError:
                print("  无效数字。")
                continue
            if hv > 1.0:           # 看起来是 mm
                hv = hv / 1000.0
            if hv < 0.0:
                print("  高度不能为负。")
                continue
            height_above = hv
            print(f"  ✅ 高度改为 {height_above*1000:.0f} mm，重走本格。")
            goto(*cells[i])
        elif cmd.startswith("g"):
            sub = parse_cell_spec(cmd[1:].strip(), rows, cols)
            if not sub:
                print("  跳转格无效，格式 'g col,row'（如 g 3,5）。")
                continue
            tr_, tc_ = sub[0]
            cells.insert(i + 1, (tr_, tc_))
            i += 1
            goto(*cells[i])
        elif cmd.startswith("sel"):
            rest = cmd[3:].strip()
            if rest == "":
                rest = input("  新的格子选择(all / r3 / c5 / '0,0;9,13'): ").strip()
            new = parse_cell_spec(rest, rows, cols)
            if not new:
                print("  选择无效，保持原列表。")
                continue
            cells = new
            i = 0
            goto(*cells[i])
        else:
            print("  未识别命令。回车=下一个，q=退出。")

    # 收尾：升到安全平面，方便取下/离开。
    if prev["xy"] is not None:
        do_move(prev["xy"][0], prev["xy"][1], max_top + travel_clear, "final_lift")
    print("\n🎉 格心巡检结束。机械臂已升到安全高度；如需继续用 MoveIt 请重启 calibrate_tool.launch。")
    print("=" * 78)


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


ALL_STEPS = [1, 2, 3, 4, 5, 6, 7]

STEP_TITLES = {
    1: "拍照起点位姿 + 相机内参",
    2: "深度拟合白板平面（板面法向，base 系）",
    3: "白板凸起网格 + 由网格派生 board_frame (BOARD_POSE_BASE)",
    4: "方块厚度（触顶面+触旁边裸面取差，同波纹管）",
    5: "放置 Z：分批摆方块按格心采深度方块顶面",
    6: "波纹管长度补偿 (tcp_pick/place_offset_z)",
    7: "抓取单应性 (发光板 pixel->board-XY, PICK_HOMOGRAPHY)",
}


def parse_steps(spec):
    """把 '4' / '3,4,5' / '2-6' / 'all' 解析成步骤集合；非法 token 忽略。"""
    spec = str(spec).strip().lower()
    if spec in ("", "all", "全部"):
        return set(ALL_STEPS)
    out = set()
    for tok in spec.replace("，", ",").replace(" ", ",").split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            if "-" in tok:
                a, b = tok.split("-", 1)
                for s in range(int(a), int(b) + 1):
                    out.add(s)
            else:
                out.add(int(tok))
        except ValueError:
            continue
    return {s for s in out if s in ALL_STEPS}


def choose_steps():
    """通过 ~steps 参数或交互选择要执行的步骤；其余步骤从现有配置读取。"""
    val = str(rospy.get_param("~steps", "")).strip()
    if val:
        steps = parse_steps(val)
        if steps:
            return steps
        print(f"⚠️ ~steps='{val}' 无法解析，转为交互选择。")

    print("\n要执行哪些步骤？（未选中的步骤将从现有配置读取其结果）")
    for s in ALL_STEPS:
        print(f"  {s} = {STEP_TITLES[s]}")
    while True:
        ans = input("输入步骤，如 '4' / '3,4,5' / '2-6' [回车=全部]: ").strip()
        steps = parse_steps(ans)
        if steps:
            return steps
        print("输入无效，请重试。")


def warn_stale_dependencies(run):
    """重跑较早步骤而不重跑依赖它的步骤时，旧数据会失配，提示并确认。"""
    downstream = {2: {3, 4, 5, 6}, 3: {4, 5, 6}}
    stale = set()
    for s, deps in downstream.items():
        if s in run:
            stale |= {d for d in deps if d not in run}
    if not stale:
        return
    print("\n⚠️ 依赖提醒：你重跑了较早的步骤，但以下依赖它的步骤未重跑，")
    print("   它们的现有数据是在【旧的 table_frame / 板面】下测得的，可能已失配：")
    print("   步骤 {}".format(", ".join(str(s) for s in sorted(stale))))
    print("   建议把这些步骤也一并重跑。")
    if not ask_yes_no("仍要继续（保留这些步骤的旧数据）？", default=False):
        print("已取消。请调整 ~steps 后重试。")
        sys.exit(0)


def cfg_get(tcfg, key, step_label):
    """从现有配置取必需键；缺失则报错退出（提示用户把该步骤纳入 ~steps）。"""
    val = tcfg.get(key) if isinstance(tcfg, dict) else None
    if val is None:
        raise SystemExit(
            "❌ 现有配置缺少 '{}'，无法跳过步骤 {}。\n"
            "   首次标定或缺数据时，请在 ~steps 里包含该步骤（或运行全部步骤）。".format(key, step_label)
        )
    return val


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

    # 触点记录源：默认改用固件 /xarm/xarm_states.pose（消除 URDF↔固件运动学系统偏差）。
    # 对所有走 require_pose 的模式生效（步骤1/3/5/6/7、depth_offset_check、edit_cells）。
    setup_firmware_pose(tf_buffer, base_frame, eef_frame)

    config_path = rospy.get_param("~config_path", DEFAULT_CONFIG_PATH)
    block_samples = int(rospy.get_param("~block_samples", 3))

    # 深度采板面相关参数（替代旧的触点 Step 2 / Step 6）。
    depth_topic = rospy.get_param("~depth_topic", DEPTH_TOPIC_DEFAULT)
    color_topic = rospy.get_param("~color_topic", COLOR_TOPIC_DEFAULT)
    use_color_roi = bool(rospy.get_param("~board_roi_use_color", True))
    camera_frame = rospy.get_param("~camera_frame", CAMERA_FRAME_DEFAULT)
    capture_frames = int(rospy.get_param("~board_capture_frames", 50))
    roi_z = rospy.get_param("~board_roi_z", [0.10, 1.50])
    roi_z = [float(roi_z[0]), float(roi_z[1])]
    param_roi = parse_roi(rospy.get_param("~board_roi", None))
    roi_force_interactive = bool(rospy.get_param("~board_roi_force_interactive", False))
    ransac_thresh = float(rospy.get_param("~board_ransac_thresh_m", 0.004))
    ransac_iters = int(rospy.get_param("~board_ransac_iters", 300))
    min_inliers_frac = float(rospy.get_param("~board_min_inliers_frac", 0.3))
    do_verify = bool(rospy.get_param("~board_verify_scale", True))

    existing_cfg = load_existing_config(config_path)
    prior_roi = None
    try:
        prior_roi = existing_cfg.get("tetris", {}).get("BOARD_DEPTH_ROI", {}).get("roi", None)
        prior_roi = [int(v) for v in prior_roi] if prior_roi else None
    except Exception:
        prior_roi = None

    tcfg = existing_cfg.get("tetris", {}) if isinstance(existing_cfg, dict) else {}
    if not isinstance(tcfg, dict):
        tcfg = {}

    # 触点↔深度 Z 偏置自检（独立模式）：~depth_offset_check:=true 时只跑自检并退出。
    if bool(rospy.get_param("~depth_offset_check", False)):
        run_depth_offset_check(tf_buffer, base_frame, eef_frame, tcfg, config_path, existing_cfg,
                               int(rospy.get_param("~check_points", 5)))
        return

    # 单独修改白板放置格心（独立模式）：~edit_cells:=true 时只跑逐格覆盖并退出（需已有步骤2/3结果）。
    if bool(rospy.get_param("~edit_cells", False)):
        run_edit_cell_centers(tf_buffer, base_frame, eef_frame, tcfg, config_path, existing_cfg)
        return

    # 步骤8 白板格心巡检（独立验证模式）：~verify_cells:=true 时驱动真机依次悬停到各格核对标定，
    # 然后退出（需已有步骤 2/3 结果；只读不写配置）。
    if bool(rospy.get_param("~verify_cells", False)):
        run_cell_tour(tf_buffer, base_frame, eef_frame, tcfg)
        return

    run = choose_steps()
    warn_stale_dependencies(run)

    print("\n" + "=" * 94)
    print("=== 俄罗斯方块标定工具：单帧深度拟板面 + 白板 A/B/C 多模式标定 ===")
    print("= 本次执行步骤: {}（其余步骤从现有配置读取）".format(
        ", ".join(str(s) for s in sorted(run))))
    print(f"= 板面法向/放置面: 深度采集 {capture_frames} 帧中值 + RANSAC（ROI 框选）")
    print("= 凸起/方块/原点仍用吸盘轻触；板面法向与放置高度改由深度采集。")
    print("= (新增) 如果标定失误，可以输入 'u' 撤销并重新记录上一个点。")
    print("=" * 94 + "\n")

    # Step 1: 拍照起点位姿 + 相机内参。
    if 1 in run:
        start_pose = require_pose(tf_buffer, "[步骤 1] 请将机械臂移动到【视觉拍照起点高度】", base_frame, eef_frame)
        cam_fx, cam_fy, cam_cx, cam_cy = get_camera_info()
    else:
        start_pose = None
        if 2 in run:
            # 步骤 2 的深度反投影需要内参；步骤 1 跳过时单独获取一次。
            cam_fx, cam_fy, cam_cx, cam_cy = get_camera_info()
        else:
            cam_fx = float(cfg_get(tcfg, "CAM_FX", "1"))
            cam_fy = float(cfg_get(tcfg, "CAM_FY", "1"))
            cam_cx = float(cfg_get(tcfg, "CAM_CX", "1"))
            cam_cy = float(cfg_get(tcfg, "CAM_CY", "1"))
            print("ℹ️ 跳过步骤 1，沿用现有相机内参。")
    intr = (cam_fx, cam_fy, cam_cx, cam_cy)

    if 2 in run and cv2 is None and param_roi is None and prior_roi is None and not roi_force_interactive:
        print("⚠️ 未能导入 cv2，无法交互框选 ROI；请用 ~board_roi 参数给定矩形后重跑。")

    # Step 2: 单机位深度拟板面，法向定义 table_frame 竖直方向 (D1)。
    # 同一平面后续也用于放置面 Z (Step 6)。
    verify_report = None
    if 2 in run:
        print("\n--- 步骤 2：深度拟合【发光板平坦表面】(定义 board_frame Z 轴/竖直方向 + z=0 基准) ---")
        print("把相机摆到能清楚看到【发光板空白平坦区】的位置，保持静止后按回车开始采集。")
        input("  就绪后按回车... ")
        if not tf_buffer.can_transform(base_frame, camera_frame, rospy.Time(0), rospy.Duration(5.0)):
            rospy.logerr("TF 缺失: %s <- %s（确认手眼发布在线）", base_frame, camera_frame)
            sys.exit(1)

        depth0 = grab_median_depth(depth_topic, capture_frames)
        # 框选底图：默认用彩色图（深度已对齐到彩色，ROI 坐标通用），无深度区压暗作提示。
        roi_display = None
        if use_color_roi and cv2 is not None:
            color_bgr = grab_color_bgr(color_topic)
            if color_bgr is not None:
                roi_display = make_roi_display(depth0, color_bgr)
                print("  框选底图=彩色图；灰暗区=无深度，请把 ROI 框在【明亮(有深度)】的平坦板面上。")
        roi, roi_src = resolve_roi(depth0, prior_roi, param_roi, roi_force_interactive,
                                   display_img=roi_display)
        print(f"  ROI={roi}（来源:{roi_src}），深度范围={roi_z} m")

        first = fit_plane_capture(tf_buffer, depth0, roi, roi_z, intr,
                                  camera_frame, base_frame, ransac_thresh, ransac_iters,
                                  min_inliers_frac)
        surface_centroid_base = first["centroid_base"]
        surface_normal_base = first["normal_base"]
        surf_rms = first["rms"]
        surf_max = first["max_abs"]
        inlier_count = int(first["inlier_count"])
        total = int(first["total"])
        inliers_base = first["inliers_base"]

        print("\n✅ 白板平面拟合完成（法向作为 board_frame Z 轴/竖直方向）：")
        print("  内点/总点: {}/{}".format(inlier_count, total))
        print("  normal(base): [{:.6f}, {:.6f}, {:.6f}]".format(*surface_normal_base))
        print("  centroid(base): [{:.6f}, {:.6f}, {:.6f}]".format(*surface_centroid_base))
        print("  residual RMS/max: {:.3f} / {:.3f} mm".format(surf_rms * 1000.0, surf_max * 1000.0))

        # 走位差分验证/深度尺度（不改变所存平面，仅报告）。
        if do_verify and ask_yes_no("是否做【走位差分验证/深度尺度】？(手动 free-drive 挪一小段再采)", default=True):
            try:
                input("  把相机沿任意方向(建议沿法向)挪 2~5cm，保持仍看白板，静止后按回车... ")
                depth1 = grab_median_depth(depth_topic, capture_frames)
                second = fit_plane_capture(tf_buffer, depth1, roi, roi_z, intr,
                                           camera_frame, base_frame, ransac_thresh, ransac_iters,
                                           min_inliers_frac)
                verify_report = verify_motion_report(first, second)
                print("  位移|Δ|={:.1f}mm, 沿法向={:.1f}mm".format(
                    verify_report["move_norm_m"] * 1000.0, verify_report["move_along_normal_m"] * 1000.0))
                print("  深度尺度比(应≈1.0): {:.4f}".format(verify_report["depth_scale_ratio"]))
                print("  两次法向夹角(应≈0): {:.3f}°".format(verify_report["normal_consistency_deg"]))
                print("  base-Z 与板面法向夹角: {:.3f}°".format(verify_report["baseZ_to_normal_deg"]))
            except Exception as e:
                print(f"  ⚠️ 验证步失败（不影响标定）: {e}")
                verify_report = None

        # 检查点：步骤2完成即落盘（板面法向/平面/ROI）。
        _inl2 = first["inliers_base"]
        _idx2 = (np.linspace(0, len(_inl2) - 1, 300).astype(int)
                 if len(_inl2) > 300 else np.arange(len(_inl2)))
        existing_cfg = write_config_merge(config_path, existing_cfg, {
            "BOARD_SURFACE_NORMAL_BASE": [float(x) for x in surface_normal_base],
            "BOARD_SURFACE_CENTROID_BASE": [float(x) for x in surface_centroid_base],
            "BOARD_PLANE_FIT_RMS_M": float(surf_rms),
            "BOARD_PLANE_FIT_MAX_ABS_M": float(surf_max),
            "BOARD_PLANE_INLIERS": int(first["inlier_count"]),
            "BOARD_PLANE_TOTAL_PTS": int(first["total"]),
            "BOARD_PLANE_SAMPLES_BASE": [[float(v) for v in p] for p in _inl2[_idx2]],
            "BOARD_DEPTH_ROI": {
                "roi": [int(v) for v in roi], "source": str(roi_src),
                "z_range_m": [float(roi_z[0]), float(roi_z[1])],
                "capture_frames": int(capture_frames), "ransac_thresh_m": float(ransac_thresh),
                "depth_topic": str(depth_topic), "camera_frame": str(camera_frame),
            },
            "BOARD_DEPTH_VERIFY": verify_report,
        })
        print("  💾 步骤2参数已保存到 config（崩溃也不丢）。")
    else:
        print("\n--- 步骤 2：跳过，沿用现有白板平面拟合结果 ---")
        surface_normal_base = np.asarray(cfg_get(tcfg, "BOARD_SURFACE_NORMAL_BASE", "2"), dtype=float)
        surface_centroid_base = np.asarray(cfg_get(tcfg, "BOARD_SURFACE_CENTROID_BASE", "2"), dtype=float)
        surf_rms = float(tcfg.get("BOARD_PLANE_FIT_RMS_M", 0.0) or 0.0)
        surf_max = float(tcfg.get("BOARD_PLANE_FIT_MAX_ABS_M", 0.0) or 0.0)
        inlier_count = int(tcfg.get("BOARD_PLANE_INLIERS", 0) or 0)
        total = int(tcfg.get("BOARD_PLANE_TOTAL_PTS", 0) or 0)
        inliers_base = np.asarray(cfg_get(tcfg, "BOARD_PLANE_SAMPLES_BASE", "2"), dtype=float)
        dz = tcfg.get("BOARD_DEPTH_ROI", {}) or {}
        roi = dz.get("roi") or param_roi or prior_roi or [0, 0, 0, 0]
        roi = [int(v) for v in roi]
        roi_src = "persisted"
        roi_z = [float(v) for v in dz.get("z_range_m", roi_z)]
        capture_frames = int(dz.get("capture_frames", capture_frames))
        ransac_thresh = float(dz.get("ransac_thresh_m", ransac_thresh))
        depth_topic = dz.get("depth_topic", depth_topic)
        camera_frame = dz.get("camera_frame", camera_frame)
        verify_report = tcfg.get("BOARD_DEPTH_VERIFY")
        first = {
            "normal_base": surface_normal_base, "centroid_base": surface_centroid_base,
            "rms": surf_rms, "max_abs": surf_max,
            "inliers_base": inliers_base, "inlier_count": inlier_count, "total": total,
        }
        print("  normal(base): [{:.6f}, {:.6f}, {:.6f}]".format(*surface_normal_base))

    # Step 3: 白板凸起网格采样（在 base 系采点）→ 由网格派生 board_frame
    # （origin=网格原点, Z=板面法向, X≈行轴）。去掉了旧的手动 table_frame 原点/X：
    # 坐标系完全由白板标定决定，白板与发光板是否对齐都不影响放置朝向。
    if 3 in run:
        board_mode = choose_board_mode()
        board_sample_col_row = board_samples_for_mode(board_mode)
        height_interpolation = choose_height_interpolation(board_mode)

        print("\n--- 步骤 3：白色底盘凸起网格标定（base 系采点，随后派生 board_frame）---")
        print("坐标显示为 (col,row)，因为底盘是 10 列 x 14 行。")
        print(f"当前模式 {board_mode}: 需要采 {len(board_sample_col_row)} 个凸起中心。")
        if board_mode == "C":
            print("模式 C 会采完整 140 点，耗时较长，但能最好反映白板局部翘曲。如果误按，输入 'u' 撤销！")

        sample_rows = []
        sample_cols = []
        sample_points_base = []
        i = 0
        # 边采边存盘 + 重启续采：相机掉线/手滑重启 launch 后不必重标整堆凸起。
        progress_path = _step3_progress_path(config_path)
        saved = _load_step3_progress(progress_path, board_mode)
        if saved:
            saved = saved[:len(board_sample_col_row)]
            ans = input(f"  🔁 检测到上次未完成的凸起进度（已采 {len(saved)}/{len(board_sample_col_row)} 点，模式 {board_mode}）。续采？[Y/n]: ").strip().lower()
            if ans not in ("n", "no"):
                for c, r, x, y, z in saved:
                    sample_cols.append(int(c))
                    sample_rows.append(int(r))
                    sample_points_base.append(np.array([x, y, z], dtype=float))
                i = len(sample_points_base)
                print(f"  ▶️ 已恢复 {i} 点，从第 {i+1} 点继续。")
            else:
                _clear_step3_progress(progress_path)
                print("  已丢弃旧进度，从头开始。")
        while i < len(board_sample_col_row):
            col, row = board_sample_col_row[i]
            p_base = require_pose(
                tf_buffer,
                f"[步骤 3.{i+1}/{len(board_sample_col_row)}] 吸盘对准【白色底盘凸起中心】 (col={col}, row={row})",
                base_frame, eef_frame, allow_undo=(i > 0))
            if isinstance(p_base, str) and p_base == "UNDO":
                i -= 1
                sample_cols.pop()
                sample_rows.pop()
                sample_points_base.pop()
                _save_step3_progress(progress_path, board_mode, sample_cols, sample_rows, sample_points_base)
                prev_col, prev_row = board_sample_col_row[i]
                print(f"  ↩️ 已撤销！退回上一个点 (col={prev_col}, row={prev_row}) 重新记录。")
                continue
            sample_cols.append(col)
            sample_rows.append(row)
            sample_points_base.append(p_base)
            _save_step3_progress(progress_path, board_mode, sample_cols, sample_rows, sample_points_base)
            i += 1

        _clear_step3_progress(progress_path)
        sample_points_base = np.asarray(sample_points_base, dtype=float)
        sample_rows = np.asarray(sample_rows, dtype=float)
        sample_cols = np.asarray(sample_cols, dtype=float)

        # 在 base 系拟仿射网格 → 原点与行/列方向（base），据此派生 board_frame。
        origin_base, row_vec_base, col_vec_base, _, _ = fit_affine_grid(sample_rows, sample_cols, sample_points_base)
        P, R_mat, rpy = derive_board_frame(origin_base, row_vec_base, col_vec_base,
                                           surface_normal_base, surface_centroid_base, tcfg)

        # 样点转入 board 系，重拟网格用于存储 + 残差。
        sample_points = np.asarray([to_table(R_mat, P, pb) for pb in sample_points_base], dtype=float)
        origin, row_vec, col_vec, xy_rms, grid_z_rms = fit_affine_grid(sample_rows, sample_cols, sample_points)

        sample_records = []
        for col, row, p_tab, p_base in zip(sample_cols.astype(int), sample_rows.astype(int),
                                           sample_points, sample_points_base):
            sample_records.append({
                "col": int(col), "row": int(row),
                "board": [float(p_tab[0]), float(p_tab[1]), float(p_tab[2])],
                "link_base": [float(p_base[0]), float(p_base[1]), float(p_base[2])],
            })

        # 高度模型（board 系 z）。
        q_coef = q_height_map = q_std = q_max = None
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
            pred = np.asarray([bump_height_map[int(r)][int(c)]
                               for r, c in zip(sample_rows.astype(int), sample_cols.astype(int))])
            residual = sample_points[:, 2] - pred
            height_std = float(np.std(residual))
            height_max_abs = float(np.max(np.abs(residual)))

        height_model_info = {
            "selected_type": height_model_type,
            "idw": {"power": 2.0,
                    "residual_std_m_at_samples": float(height_std) if height_model_type == "idw" else None,
                    "residual_max_abs_m_at_samples": float(height_max_abs) if height_model_type == "idw" else None},
            "quadratic": {"available": q_coef is not None,
                          "z =": "a0 + ar*r + ac*c + arr*r^2 + acc*c^2 + arc*r*c, r/c normalized to 0..1",
                          "coefficients": [float(v) for v in q_coef] if q_coef is not None else [],
                          "residual_std_m": float(q_std) if q_std is not None else None,
                          "residual_max_abs_m": float(q_max) if q_max is not None else None},
            "selected_residual_std_m": float(height_std),
            "selected_residual_max_abs_m": float(height_max_abs),
        }

        print("\n✅ board_frame 由网格派生：")
        print("  origin(base): [{:.6f}, {:.6f}, {:.6f}]".format(*P))
        print("  rpy(base):    [{:.6f}, {:.6f}, {:.6f}]".format(*rpy))
        print("  网格 XY 拟合 RMS: {:.3f} mm".format(xy_rms * 1000.0))

        # 检查点：步骤3完成即落盘（坐标系 + 触点网格 + 凸起高度，最费时、最不该丢）。
        existing_cfg = write_config_merge(config_path, existing_cfg, {
            "BOARD_POSE_BASE": {"origin": [float(P[0]), float(P[1]), float(P[2])],
                                "rpy": [float(rpy[0]), float(rpy[1]), float(rpy[2])]},
            "BOARD_ROWS": ROWS, "BOARD_COLS": COLS,
            "BOARD_CALIBRATION_MODE": {
                "mode": str(board_mode), "sample_count": int(len(board_sample_col_row)),
                "description": {"A": "9 points", "B": "25 points", "C": "140 points"}.get(
                    board_mode, f"{len(board_sample_col_row)} points")},
            "BOARD_SAMPLE_FORMAT": "samples are prompted as (col,row), stored internally as row/col; coords in board_frame",
            "BOARD_SAMPLES_BOARD": sample_records,
            "BOARD_ORIGIN_BOARD": [float(origin[0]), float(origin[1]), float(origin[2])],
            "BOARD_ROW_STEP_BOARD": [float(row_vec[0]), float(row_vec[1]), float(row_vec[2])],
            "BOARD_COL_STEP_BOARD": [float(col_vec[0]), float(col_vec[1]), float(col_vec[2])],
            "BOARD_GRID_FIT_RMS_XY_M": float(xy_rms), "BOARD_GRID_FIT_RMS_Z_M": float(grid_z_rms),
            "BOARD_BUMP_HEIGHT_MODEL": height_model_info,
            "BOARD_BUMP_HEIGHT_MAP_14x10": bump_height_map,
            # 网格已重派生，旧的逐格手动覆盖（~edit_cells 写入）已失配，清空。
            "BOARD_CELL_OVERRIDES_BOARD": [],
        })
        print("  💾 步骤3参数已保存到 config（触点网格/凸起高度，崩溃也不丢）。")
    else:
        print("\n--- 步骤 3：跳过，沿用现有 board_frame / 网格 ---")
        bp = cfg_get(tcfg, "BOARD_POSE_BASE", "3")
        P = np.asarray(bp["origin"], dtype=float)
        rpy = tuple(float(v) for v in bp["rpy"])
        R_mat = euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")[:3, :3]
        origin = np.asarray(cfg_get(tcfg, "BOARD_ORIGIN_BOARD", "3"), dtype=float)
        row_vec = np.asarray(cfg_get(tcfg, "BOARD_ROW_STEP_BOARD", "3"), dtype=float)
        col_vec = np.asarray(cfg_get(tcfg, "BOARD_COL_STEP_BOARD", "3"), dtype=float)
        bump_height_map = cfg_get(tcfg, "BOARD_BUMP_HEIGHT_MAP_14x10", "3")
        height_model_info = tcfg.get("BOARD_BUMP_HEIGHT_MODEL", {}) or {}
        sample_records = tcfg.get("BOARD_SAMPLES_BOARD", []) or []
        bcm = tcfg.get("BOARD_CALIBRATION_MODE", {}) or {}
        board_mode = str(bcm.get("mode", "?"))
        board_sample_col_row = sample_records
        xy_rms = float(tcfg.get("BOARD_GRID_FIT_RMS_XY_M", 0.0) or 0.0)
        grid_z_rms = float(tcfg.get("BOARD_GRID_FIT_RMS_Z_M", 0.0) or 0.0)
        height_model_type = str(height_model_info.get("selected_type", "?"))
        height_interpolation = height_model_type
        height_std = float(height_model_info.get("selected_residual_std_m") or 0.0)
        height_max_abs = float(height_model_info.get("selected_residual_max_abs_m") or 0.0)
        print("  origin(base): [{:.6f}, {:.6f}, {:.6f}]".format(*P))

    # Step 4: 方块厚度（触点差值，与波纹管同理）→ block_thickness。
    # 触【方块顶面】+ 触【旁边裸表面(方块所坐的面)】，沿板面法向取差 = 真实厚度(恒正)。
    # 不再用方块顶面的 board-z 绝对值（那受 board z=0 零点偏置影响，曾测出负厚度）。
    if 4 in run:
        print("\n--- 步骤 4：方块厚度（触顶面 + 触旁边裸面，取差，同波纹管）---")
        print("每个方块两次触碰：先轻触【方块顶面】，再触【方块旁边的裸表面(方块所坐的面)】。")
        print("建议换不同颜色/不同位置多采几个，取中位数。")
        # 投影法向：优先 align_tool_to_board 的对齐法向，否则步骤2板面法向（与波纹管一致）。
        align_cfg = tcfg.get("TOOL_BOARD_ALIGN", {}) or {}
        align_normal = align_cfg.get("normal_base")
        if align_normal:
            n_up = normalize(np.asarray(align_normal, dtype=float), "aligned board normal")
        else:
            n_up = normalize(np.asarray(surface_normal_base, dtype=float), "board normal")

        thicknesses = []
        num_blocks = max(1, block_samples)
        i = 0
        while i < num_blocks:
            p_top = require_pose(
                tf_buffer, f"[步骤 4.{i+1}/{num_blocks}.1] 吸嘴轻触【方块顶面】",
                base_frame, eef_frame, allow_undo=(i > 0))
            if isinstance(p_top, str) and p_top == "UNDO":
                i -= 1
                thicknesses.pop()
                print(f"  ↩️ 已撤销，重测第 {i+1} 个方块（从触顶面开始）。")
                continue
            p_surf = require_pose(
                tf_buffer, f"[步骤 4.{i+1}/{num_blocks}.2] 触【方块旁边的裸表面】(方块所坐的面)",
                base_frame, eef_frame, allow_undo=True)
            if isinstance(p_surf, str) and p_surf == "UNDO":
                print("  ↩️ 已撤销，重测本方块（从触顶面开始）。")
                continue
            t = float(np.dot(p_top - p_surf, n_up))   # 沿法向：顶面 − 裸面 = 厚度(应为正)
            if t <= 1e-4:
                print(f"  ⚠️ 量出厚度 = {t*1000:.1f} mm，异常(可能顶面/裸面触反了/触太轻)，重测本方块。")
                continue
            thicknesses.append(t)
            print(f"  方块 {i+1} 厚度 = {t*1000:.2f} mm")
            i += 1

        block_thickness = float(np.median(thicknesses))
        print(f"  ✅ 方块厚度中位 = {block_thickness*1000:.2f} mm（差值法，恒正）")

        # 检查点：步骤4完成即落盘（方块厚度）。
        existing_cfg = write_config_merge(config_path, existing_cfg, {
            "BLOCK_THICKNESS": float(block_thickness),
            "PICK_Z": float(block_thickness),
        })
        print("  💾 步骤4参数已保存到 config。")
    else:
        print("\n--- 步骤 4：跳过，沿用现有方块厚度 ---")
        block_thickness = float(cfg_get(tcfg, "BLOCK_THICKNESS", "4"))
        print(f"  block_thickness = {block_thickness:.6f} m")

    # Step 5: 放置 Z 标定 —— 在白板上【分批摆方块】，按格心采深度【方块顶面】→ 直接得每格放置 Z。
    # 方块顶面平整、深度干净；格心远离方块缝隙，不受缝隙影响；自动含板面翘曲+凸起+多格架桥。
    # 覆盖判据基准用【空白板的深度基准】base_map（先采一遍空板深度）：纯深度对深度，零点与方块顶面
    # 一致，免疫触点(步骤3/4)与深度的零点偏差（触点曾量出负方块厚/近零凸起，与深度对不上）。
    place_top_map = None
    place_top_info = {}
    if 5 in run:
        print("\n--- 步骤 5：分批摆方块，按格心采深度【方块顶面】→ 放置 Z ---")
        inl_base_s5 = first["inliers_base"]
        if len(inl_base_s5) > 2000:
            sel = np.linspace(0, len(inl_base_s5) - 1, 2000).astype(int)
            inl_base_s5 = inl_base_s5[sel]
        surface_points = (inl_base_s5 - P) @ R_mat   # base→board（逐行 R_mat.T·(p-P)）
        board_surface_z = float(np.median(surface_points[:, 2]))
        surf_a, surf_b, surf_c, board_surf_std = fit_plane_xyz(surface_points)
        print("  发光板平面 z(board) 中值: {:.6f} m（=board z≈0 基准，非白板表面）, 平面残差 std: {:.3f} mm".format(
            board_surface_z, board_surf_std * 1000.0))
        use_bump_height_for_place = bool(tcfg.get("USE_BUMP_HEIGHT_FOR_PLACE", True))

        if not tf_buffer.can_transform(base_frame, camera_frame, rospy.Time(0), rospy.Duration(5.0)):
            rospy.logerr("TF 缺失: %s <- %s（确认手眼发布在线）", base_frame, camera_frame)
            sys.exit(1)

        place_z_margin = float(rospy.get_param("~place_z_margin", 0.0))
        try:
            txt = input(f"放置高度额外余量 place_z_margin，单位 m [默认 {place_z_margin:.4f}]: ").strip()
            if txt:
                place_z_margin = float(txt)
        except Exception:
            pass

        # 覆盖判据 & 采样窗口参数。基准用【空白板的深度基准】base_map[r][c]：有方块 ⟺ 深度顶面比
        # 空板基准高出 [cover_min, cover_max]。纯深度对深度，免疫触点(步骤3/4)与深度的零点偏差
        # （触点曾量出负方块厚/近零凸起，与深度零点对不上，导致空格也被判成覆盖）。
        cover_min = float(rospy.get_param("~place_cover_min_m", 0.005))
        cover_max = float(rospy.get_param("~place_top_z_span_m", 0.040))
        win_half = int(rospy.get_param("~place_sample_win_half", 2))
        min_valid = int(rospy.get_param("~place_sample_min_valid", 5))
        show_overlay = bool(rospy.get_param("~place_show_overlay", True)) and cv2 is not None
        overlay_save_dir = str(rospy.get_param("~place_overlay_save_dir", "/tmp"))
        use_empty_baseline = bool(rospy.get_param("~place_empty_baseline", True))
        # 格心投影优先级：① 现场点白板四角拟合的单应性(当前机位、白板本身，最准)；
        # ② PICK_HOMOGRAPHY(抓取区/别的机位标的，用到白板上是外推，可能不准)；③ 手眼 3D 投影(受 ~cm 手眼误差)。
        print("  覆盖判据: 深度顶面高出【空板深度基准】{:.1f}~{:.1f} mm 视为有方块；格心采样窗口 {}x{} px。".format(
            cover_min * 1000.0, cover_max * 1000.0, 2 * win_half + 1, 2 * win_half + 1))
        H_inv = None
        if bool(rospy.get_param("~place_corner_clicks", True)) and cv2 is not None:
            print("\n  先【点白板四角】当场拟合投影单应性(免手眼/抓取单应性的误差)。请确保白板空、相机别再动。")
            input("  把相机停在采集机位后按回车，开始点角...")
            color_h = grab_color_bgr(color_topic)
            if color_h is not None:
                H_inv = place_homography_from_clicks(color_h, origin, row_vec, col_vec, ROWS, COLS)
        if H_inv is not None:
            print("  ✅ 格心投影用【点角单应性】(当前机位+白板, 最准)——之后相机不要移动！")
        elif bool(rospy.get_param("~place_use_homography", True)):
            H_inv = load_homography_inv(tcfg)
            if H_inv is not None:
                print("  ⚠️ 回退 PICK_HOMOGRAPHY(抓取区标的, 用到白板是外推, 可能偏)——相机须在 affine 拍照位姿。")
        if H_inv is None:
            print("  ℹ️ 无单应性，格心投影回退手眼 3D 投影(受手眼 ~cm 误差影响)。")

        # 逐格基准 base_map：优先用【空白板深度】(与方块顶面同一深度零点)，回退触点凸起 bump_height_map。
        base_map = [[float(bump_height_map[r][c]) for c in range(COLS)] for r in range(ROWS)]
        if use_empty_baseline:
            print("\n  先采【空白板】深度基准：请确保白板上【没有任何方块】。")
            input("  清空白板后按回车采空板基准...")
            depth_e = grab_median_depth(depth_topic, capture_frames)
            R_cam_e, t_cam_e = lookup_R_t(tf_buffer, base_frame, camera_frame)
            er, ec, ez = [], [], []
            for r in range(ROWS):
                for c in range(COLS):
                    p_board = point_on_grid(origin, row_vec, col_vec, r, c)
                    p_guess = np.array([p_board[0], p_board[1], float(bump_height_map[r][c])], dtype=float)
                    uv = cell_center_pixel(p_guess, H_inv, P, R_mat, R_cam_e, t_cam_e, intr)
                    if uv is None:
                        continue
                    zz = sample_window_median_depth(depth_e, uv[0], uv[1], win_half, min_valid)
                    if zz is None:
                        continue
                    bz = float(pixel_depth_to_board(uv[0], uv[1], zz, P, R_mat, R_cam_e, t_cam_e, intr)[2])
                    base_map[r][c] = bz
                    er.append(r); ec.append(c); ez.append(bz)
            if ez:
                # 个别无深度格用 IDW 从有效空板格补，保证每格都有基准。
                filled = idw_interpolate(er, ec, ez, ROWS, COLS)
                got = {(r, c) for r, c in zip(er, ec)}
                for r in range(ROWS):
                    for c in range(COLS):
                        if (r, c) not in got:
                            base_map[r][c] = float(filled[r][c])
                ea = np.asarray(ez) * 1000.0
                print("  ✅ 空板基准: 有效 {}/{} 格; z(mm) 最小/中位/最大 = {:.1f}/{:.1f}/{:.1f}".format(
                    len(ez), ROWS * COLS, float(np.min(ea)), float(np.median(ea)), float(np.max(ea))))
            else:
                print("  ⚠️ 空板基准一格都没采到深度，回退用触点凸起 bump_height_map 作基准。")

        place_top_z = [[None] * COLS for _ in range(ROWS)]
        covered = [[False] * COLS for _ in range(ROWS)]
        n_target = ROWS * COLS
        batch = 0
        while not rospy.is_shutdown():
            n_cov = sum(1 for r in range(ROWS) for c in range(COLS) if covered[r][c])
            if n_cov >= n_target:
                break
            remaining = [(c, r) for r in range(ROWS) for c in range(COLS) if not covered[r][c]]
            print(f"\n  进度：已覆盖 {n_cov}/{n_target}，剩余 {len(remaining)} 格。")
            preview = ", ".join(f"(c{c},r{r})" for c, r in remaining[:20])
            print(f"  仍需覆盖(col,row)：{preview}{' ...' if len(remaining) > 20 else ''}")
            ans = input("  把方块摆到【未覆盖】格上（可重叠/超界，无妨），静止后回车采集；q=提前结束: ").strip().lower()
            if ans in ("q", "quit", "结束"):
                print("  ⚠️ 提前结束，剩余未覆盖格将用 IDW 从已覆盖格插值。")
                break
            batch += 1
            depth_b = grab_median_depth(depth_topic, capture_frames)
            color_b = grab_color_bgr(color_topic) if show_overlay else None
            R_cam, t_cam = lookup_R_t(tf_buffer, base_frame, camera_frame)   # camera → base
            newly = 0
            newly_cells = set()
            dz_samples = []          # 诊断：top_z − 空板基准 (m)，仅统计采到有效深度的格
            for r in range(ROWS):
                for c in range(COLS):
                    if covered[r][c]:
                        continue
                    cell_base_z = float(base_map[r][c])             # dz 比较基准（深度空板）
                    cell_proj_z = float(bump_height_map[r][c])      # 投影高度（触点真实面，免疫深度 tare）
                    p_board = point_on_grid(origin, row_vec, col_vec, r, c)
                    # 投影到【触点真实面】(物理格心位置)，确保采样像素落在该格上；空格采到基准面、
                    # 有方块采到方块顶(盖住整格)。深度的 tare 偏置只进 dz 的两端、做差时抵消，不进投影。
                    p_guess = np.array([p_board[0], p_board[1], cell_proj_z], dtype=float)
                    uv = cell_center_pixel(p_guess, H_inv, P, R_mat, R_cam, t_cam, intr)
                    if uv is None:
                        continue
                    z_meas = sample_window_median_depth(depth_b, uv[0], uv[1], win_half, min_valid)
                    if z_meas is None:
                        continue
                    top_z = float(pixel_depth_to_board(uv[0], uv[1], z_meas, P, R_mat, R_cam, t_cam, intr)[2])
                    dz = top_z - cell_base_z
                    dz_samples.append(dz)
                    if cover_min <= dz <= cover_max:
                        place_top_z[r][c] = top_z
                        covered[r][c] = True
                        newly += 1
                        newly_cells.add((r, c))
            print(f"  本批新增覆盖 {newly} 格，累计 {n_cov + newly}/{n_target}。")
            if dz_samples:
                arr = np.asarray(dz_samples) * 1000.0
                garbage = int(np.sum(np.abs(arr) > 100.0))   # |dz|>100mm=深度飞点/空洞被填成远背景
                p5, p50, p95 = np.percentile(arr, [5, 50, 95])
                print("  诊断: 采到深度 {} 格; (顶面−空板基准) mm p5/中位/p95 = {:.1f}/{:.1f}/{:.1f}; "
                      "窗口 [{:.1f},{:.1f}]mm; 飞点(|dz|>100mm) {} 格(已被窗口排除)".format(
                          len(dz_samples), float(p5), float(p50), float(p95),
                          cover_min * 1000.0, cover_max * 1000.0, garbage))
            else:
                print("  诊断: 没有任何未覆盖格采到有效深度（投影越界 / 窗口内无深度点）。")
            if show_overlay:
                show_coverage_overlay(depth_b, color_b, P, R_mat, R_cam, t_cam, intr,
                                      origin, row_vec, col_vec, bump_height_map, block_thickness,
                                      covered, newly_cells, batch, overlay_save_dir, H_inv=H_inv)
            if newly == 0:
                print("  ⚠️ 本批没采到新格：检查方块是否摆在未覆盖格、相机是否看得到、阈值是否合适。")

        n_cov = sum(1 for r in range(ROWS) for c in range(COLS) if covered[r][c])
        n_interp = n_target - n_cov
        cov_rows = [r for r in range(ROWS) for c in range(COLS) if covered[r][c]]
        cov_cols = [c for r in range(ROWS) for c in range(COLS) if covered[r][c]]
        cov_z = [place_top_z[r][c] for r in range(ROWS) for c in range(COLS) if covered[r][c]]
        if not cov_z:
            raise SystemExit("❌ 步骤5未采到任何方块顶面，无法生成放置 Z。\n"
                             "   步骤1~4参数已保存到 config（不丢）；修正摆放/相机/阈值后用 ~steps:=5 单独补这一步即可。")
        # IDW 在整数采样点保留实测值，仅对未覆盖格插值。
        place_top_map = idw_interpolate(cov_rows, cov_cols, cov_z, ROWS, COLS)
        place_top_info = {
            "method": "depth_block_top_per_cell",
            "baseline": ("empty_board_depth_per_cell" if use_empty_baseline
                         else "per_cell_bump_height_map (touch)"),
            "batches": int(batch),
            "covered_cells": int(n_cov),
            "interpolated_cells": int(n_interp),
            "cover_min_m": float(cover_min),
            "cover_max_m": float(cover_max),
            "block_thickness_m": float(block_thickness),
            "sample_win_half_px": int(win_half),
            "sample_min_valid": int(min_valid),
            "board_surface_z_m": float(board_surface_z),
            "capture_frames": int(capture_frames),
        }

        # 检查点：步骤5完成即落盘（实测方块顶面放置图 + 放置面平面）。
        existing_cfg = write_config_merge(config_path, existing_cfg, {
            "BOARD_SURFACE_Z": float(board_surface_z),
            "BOARD_SURFACE_PLANE": {"model": "z = a*x + b*y + c in board_frame",
                                    "a": float(surf_a), "b": float(surf_b), "c": float(surf_c),
                                    "residual_std_m": float(board_surf_std)},
            "BOARD_PLACE_TOP_MAP_14x10": [[float(v) for v in r] for r in place_top_map],
            "BOARD_PLACE_EMPTY_TOP_MAP_14x10": [[float(v) for v in r] for r in base_map],
            "BOARD_PLACE_TOP_INFO": place_top_info,
        })
        print("  💾 步骤5参数已保存到 config（实测方块顶面 + 空板基准，崩溃也不丢）。")
        print("\n✅ 放置 Z（方块顶面）采集完成：覆盖 {}/{}，插值 {} 格，共 {} 批。".format(
            n_cov, n_target, n_interp, batch))
        if n_interp:
            print("  ⚠️ 有 {} 格未实测、由 IDW 插值，上机放置前请重点核对这些格。".format(n_interp))
    else:
        print("\n--- 步骤 5：跳过，沿用现有放置面/放置高度设置 ---")
        board_surface_z = float(tcfg.get("BOARD_SURFACE_Z", 0.0) or 0.0)
        sp = cfg_get(tcfg, "BOARD_SURFACE_PLANE", "5")
        surf_a = float(sp.get("a", 0.0))
        surf_b = float(sp.get("b", 0.0))
        surf_c = float(sp.get("c", board_surface_z))
        board_surf_std = float(sp.get("residual_std_m", 0.0) or 0.0)
        use_bump_height_for_place = bool(tcfg.get("USE_BUMP_HEIGHT_FOR_PLACE", True))
        place_z_margin = float(tcfg.get("PLACE_Z_MARGIN", 0.0) or 0.0)
        place_top_map = tcfg.get("BOARD_PLACE_TOP_MAP_14x10")
        place_top_info = tcfg.get("BOARD_PLACE_TOP_INFO", {}) or {}
        if place_top_map is not None:
            print("  采用现有方块顶面放置图 BOARD_PLACE_TOP_MAP_14x10（放置 Z = 顶面 z + 余量）。")
        else:
            print(f"  无方块顶面图，回退 legacy 放置高度：use_bump={use_bump_height_for_place}。")
        print(f"  place_z_margin={place_z_margin:.4f} m")

    # 派生输出：放置高度图/中心点/legacy 字段。无论跑哪些步骤都按当前(新或旧)值重算，
    # 以保证它们与 block_thickness / 网格 / 放置面设置一致。
    place_z_map = []
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            if place_top_map is not None:
                # 实测方块顶面（已含凸起+厚度+多格架桥）：放置 Z = 顶面 z + 余量。
                row.append(float(place_top_map[r][c] + place_z_margin))
            else:
                # legacy 回退：凸起/平面高度 + 方块厚度 + 余量。
                if use_bump_height_for_place:
                    base_z = bump_height_map[r][c]
                else:
                    p_grid = point_on_grid(origin, row_vec, col_vec, r, c)
                    base_z = surf_a * p_grid[0] + surf_b * p_grid[1] + surf_c
                row.append(float(base_z + block_thickness + place_z_margin))
        place_z_map.append(row)

    # 逐格手动覆盖（BOARD_CELL_OVERRIDES_BOARD, 由 ~edit_cells 模式写入）：重跑步骤3会重派生网格、
    # 旧覆盖失配 → 清空；否则沿用并叠加到格心 XY（z=凸起高度、放置深度 PLACE_Z_MAP 不受影响）。
    cell_overrides = {} if (3 in run) else parse_cell_overrides(tcfg)
    board_centers = []
    for r in range(ROWS):
        line = []
        for c in range(COLS):
            p = point_on_grid(origin, row_vec, col_vec, r, c)
            # x/y from affine grid (or per-cell override), z from selected height map
            cx, cy = float(p[0]), float(p[1])
            if (r, c) in cell_overrides:
                ov = cell_overrides[(r, c)]
                cx, cy = float(ov[0]), float(ov[1])
            line.append([cx, cy, float(bump_height_map[r][c])])
        board_centers.append(line)

    legacy_grid = float(0.5 * (np.linalg.norm(row_vec[:2]) + np.linalg.norm(col_vec[:2])))
    legacy_origin_x = float(origin[0])
    legacy_origin_y = float(origin[1])
    place_z_legacy = float(np.median(np.asarray(place_z_map)))
    hover_z = place_z_legacy + 0.04
    pick_z = float(block_thickness)

    # Step 6: 波纹管长度补偿（可选）。所有几何标定都用硬吸嘴(link_tcp)完成；波纹管挂在
    # 吸嘴尖下方、软、会压缩。这里只把它换算成一个【下扎 Z 偏置】，不影响 XY/单应性/射线
    # 求交（那些是几何位置，波纹管不改变；积木高度视差另由控制/路径节点处理）。
    # 推导：tcp_*_offset_z = 波纹管自由长度 L − 期望密封压缩量 δ（正值=抬高吸嘴给波纹管让位）。
    bellows_cfg = None
    if 6 in run:
        print("\n--- 步骤 6：波纹管长度补偿（两次触碰测量）---")
        print("所有几何标定都用硬吸嘴(link_tcp)完成；波纹管挂在吸嘴尖下方、软、会压缩。")
        print("先拆波纹管用吸嘴尖触碰发光板，再装上波纹管触碰同一点，两次 link_tcp 高度差即自由长度。")
        print("它只换算成下扎 Z 偏置(tcp_pick/place_offset_z)，不进入 XY/单应性/射线求交。")
        prev_pick = float(tcfg.get("BELLOWS_PICK_COMPRESSION_M", 0.004) or 0.004)
        prev_place = float(tcfg.get("BELLOWS_PLACE_COMPRESSION_M", 0.003) or 0.003)
        # 投影轴优先用 align_tool_to_board 写入的对齐法向；缺失时回退步骤2的板面法向。
        align_cfg = tcfg.get("TOOL_BOARD_ALIGN", {}) or {}
        align_normal = align_cfg.get("normal_base")
        if align_normal:
            n_up = normalize(np.asarray(align_normal, dtype=float), "aligned board normal")
            print("  投影轴：使用 align_tool_to_board 写入的板面法向 TOOL_BOARD_ALIGN.normal_base。")
        else:
            n_up = normalize(np.asarray(surface_normal_base, dtype=float), "board normal")
            print("  投影轴：未找到 TOOL_BOARD_ALIGN，回退用步骤2的板面法向。")

        bellows_len = None
        while True:
            p_nozzle = require_pose(tf_buffer, "[步骤 6.1] 【拆掉波纹管】，用吸嘴尖轻触发光板上某一点", base_frame, eef_frame, allow_undo=False)
            p_bellows = require_pose(tf_buffer, "[步骤 6.2] 【装上波纹管】，在同一点用波纹管尖轻触发光板", base_frame, eef_frame, allow_undo=True)
            if isinstance(p_bellows, str) and p_bellows == "UNDO":
                print("  ↩️ 已撤销，重新测量（从吸嘴触碰开始）。")
                continue
            # 沿板面法向投影两次 link_tcp 的差 = 波纹管自由长度（对两次落点的微小横向差不敏感）。
            L = float(np.dot(p_bellows - p_nozzle, n_up))
            if L <= 1e-4:
                print(f"  ⚠️ 量出的长度 = {L*1000:.1f} mm，异常（可能两次顺序反了/触碰太轻）。")
                if ask_yes_no("重新测量？", default=True):
                    continue
                print("  跳过波纹管步骤（不写入）。")
                break
            bellows_len = L
            break

        if bellows_len is not None:
            print(f"  ✅ 波纹管自由长度 L = {bellows_len*1000:.1f} mm")
            bellows_pick_press = ask_float("抓取期望密封压缩量 δ_pick，单位 m", prev_pick)
            bellows_place_press = ask_float("放置期望压缩量 δ_place（别太大以免压乱已放方块），单位 m", prev_place)
            tcp_pick_offset_z = bellows_len - bellows_pick_press
            tcp_place_offset_z = bellows_len - bellows_place_press
            bellows_cfg = {
                "BELLOWS_FREE_LENGTH_M": float(bellows_len),
                "BELLOWS_PICK_COMPRESSION_M": float(bellows_pick_press),
                "BELLOWS_PLACE_COMPRESSION_M": float(bellows_place_press),
                # = L − δ；控制/路径节点会从 /tetris/TCP_*_OFFSET_Z 自动读取（launch 同名参数可覆盖）。
                "TCP_PICK_OFFSET_Z": float(tcp_pick_offset_z),
                "TCP_PLACE_OFFSET_Z": float(tcp_place_offset_z),
            }
            print("\n✅ 波纹管补偿（正值=抬高吸嘴给波纹管让位，已写入 /tetris，节点自动读取）：")
            print(f"  tcp_pick_offset_z  = {tcp_pick_offset_z:.4f} m")
            print(f"  tcp_place_offset_z = {tcp_place_offset_z:.4f} m")

            # 检查点：步骤6完成即落盘（波纹管补偿）。
            existing_cfg = write_config_merge(config_path, existing_cfg, bellows_cfg)
            print("  💾 步骤6参数已保存到 config。")

    # Step 7: 抓取单应性 (发光板 pixel->board-XY)。整合自 pick_affine：在【整流彩色图】上点选黑点 + 逐个触点
    # 取 board-XY → findHomography → 写 PICK_HOMOGRAPHY（控制节点抓取 XY 用它）。依赖步骤3的 board_frame。
    # ⚠️ 单应性绑定相机【拍照位姿】——标定时相机摆在生产视觉那个机位；改坐标系(步骤3)后必须重标本步。
    if 7 in run:
        print("\n--- 步骤 7：抓取单应性标定 (发光板 pixel->board-XY → PICK_HOMOGRAPHY) ---")
        # 点数可变：max_ph 为上限，min_ph 为下限；findHomography 至少要 4 点，故 min_ph 强制 >=4。
        max_ph = int(rospy.get_param("~pick_homography_points", 9))
        min_ph = max(4, int(rospy.get_param("~pick_homography_min_points", 4)))
        max_ph = max(min_ph, max_ph)
        rect_topic = rospy.get_param("~rect_color_topic", "/camera/color/image_rect_color")
        if cv2 is None:
            print("  ⚠️ cv2 不可用，跳过步骤7。")
        else:
            print(f"  把相机摆到【拍照位姿】，取整流彩色图 {rect_topic}（需 image_proc 在跑）...")
            img = grab_color_bgr(rect_topic)
            if img is None:
                print("  ⚠️ 取整流彩色图失败，跳过步骤7（确认 image_proc 在跑、话题对）。")
            else:
                # 相机此刻在【拍照位姿/初始点】=视觉与单应性同一机位；在此读取工具朝向，
                # 换算到 board 系后写盘，供控制节点抓取下扎复用其 roll/pitch（消除工具相对
                # 板面微小倾角经 TCP 偏移投影造成的抓偏）。须在用户挪臂压黑点之前读。
                tool_R_base = get_eef_rotmat(tf_buffer, base_frame, eef_frame)
                if tool_R_base is None:
                    print("  ⚠️ 读取初始点工具朝向失败，本次将不写 PICK_TOOL_RPY_BOARD。")
                print(f"  在【发光板/标定纸】上点选黑点(带放大镜)：点数可变，至少 {min_ph} 个、至多 {max_ph} 个，点够后回车结束。")
                clicks = click_pixels(img, max_ph, "pick homography: click black dots (zoom)", min_points=min_ph)
                if not clicks:
                    print("  已取消，跳过步骤7。")
                else:
                    print(f"\n  点选完成 {len(clicks)} 个。现在逐个把吸嘴尖压到对应物理黑点、回车记录('u'撤销)。")
                    rows = []
                    i = 0
                    while i < len(clicks):
                        u, v = clicks[i]
                        p = require_pose(
                            tf_buffer, f"[步骤 7.{i+1}/{len(clicks)}] 吸嘴尖压到像素({u},{v})对应的物理黑点",
                            base_frame, eef_frame, allow_undo=(i > 0))
                        if isinstance(p, str) and p == "UNDO":
                            i -= 1
                            rows.pop()
                            print("  ↩️ 已撤销，重记上一个点。")
                            continue
                        bxy = to_table(R_mat, P, p)
                        rows.append({"u": int(u), "v": int(v), "board": [float(bxy[0]), float(bxy[1])]})
                        i += 1
                    src = np.array([[r["u"], r["v"]] for r in rows], dtype=np.float64)
                    dst = np.array([r["board"] for r in rows], dtype=np.float64)
                    Hpk, _ = cv2.findHomography(src, dst)   # pixel -> board-XY
                    if Hpk is None:
                        print("  ⚠️ 单应性拟合失败，未写入。")
                    else:
                        res = []
                        for r in rows:
                            q = Hpk.dot([r["u"], r["v"], 1.0])
                            q = q / q[2]
                            res.append(((q[0] - r["board"][0]) ** 2 + (q[1] - r["board"][1]) ** 2) ** 0.5)
                        rms = float(np.sqrt(np.mean(np.square(res))))
                        updates = {"PICK_HOMOGRAPHY": {
                            "enabled": True,
                            "model": "x = (H00*u + H01*v + H02)/W; y = (H10*u + H11*v + H12)/W; W = H20*u + H21*v + H22",
                            "matrix": [float(x) for x in Hpk.flatten()],
                            "rms_m": rms,
                            "sample_count": int(len(rows)),
                        }}
                        # 初始点工具朝向换算到 board 系；控制节点取其 roll/pitch 作为抓取下扎朝向。
                        if tool_R_base is not None:
                            R_tool_board = R_mat.T.dot(tool_R_base)   # base->board · 工具(base) = 工具(board)
                            M4 = np.eye(4)
                            M4[:3, :3] = R_tool_board
                            rpy_b = euler_from_matrix(M4, axes="sxyz")
                            updates["PICK_TOOL_RPY_BOARD"] = [float(rpy_b[0]), float(rpy_b[1]), float(rpy_b[2])]
                            print("  🧭 初始点工具朝向(board系 rpy)=[{:.4f}, {:.4f}, {:.4f}]；".format(*rpy_b)
                                  + "控制节点抓取下扎将用其 roll/pitch（替代写死的 pi,0）。")
                        # 禁用旧的 affine 防止冲突（与 pick_affine 一致）。
                        paff = (existing_cfg.get("tetris", {}) or {}).get("PICK_AFFINE_CORRECTION")
                        if isinstance(paff, dict):
                            paff = dict(paff)
                            paff["enabled"] = False
                            updates["PICK_AFFINE_CORRECTION"] = paff
                        existing_cfg = write_config_merge(config_path, existing_cfg, updates)
                        print(f"  ✅ 抓取单应性拟合完成，RMS={rms*1000:.3f} mm，已写入 PICK_HOMOGRAPHY。")
                        # 逐点残差(mm)：findHomography 用最小二乘、不剔离群点，单个坏点会同时抬高 RMS
                        # 并带歪 H；打印每点残差与最大点(点号与触点顺序 7.N 一致)，便于定位是否需重标某点。
                        res_mm = [x * 1000.0 for x in res]
                        imax = int(np.argmax(res_mm))
                        print("  逐点残差(mm): " + ", ".join(
                            f"#{j+1}={e:.2f}" for j, e in enumerate(res_mm)))
                        print("  最大残差 #{} = {:.2f} mm（像素({},{})）；若明显离群于其余点，建议重标该点。".format(
                            imax + 1, res_mm[imax], rows[imax]["u"], rows[imax]["v"]))
                        print("  💾 步骤7参数已保存到 config（控制节点抓取 XY 用它；改坐标系后须重标）。")

    inl_all = first["inliers_base"]
    if len(inl_all) > 300:
        _sel = np.linspace(0, len(inl_all) - 1, 300).astype(int)
        plane_samples_out = inl_all[_sel]
    else:
        plane_samples_out = inl_all

    config = {
        "tetris": {
            "calibration_frames": {"base_frame": str(base_frame), "eef_frame": str(eef_frame)},

            # board_frame 在 base 系的位姿（由白板网格派生：原点=网格原点, Z=板面法向, X≈行轴）。
            # 控制/路径/单应性节点把它作为常量加载，不再发布/查询 table_frame TF。
            "BOARD_POSE_BASE": {
                "origin": [float(P[0]), float(P[1]), float(P[2])],
                "rpy": [float(rpy[0]), float(rpy[1]), float(rpy[2])],
            },

            "BOARD_SURFACE_NORMAL_BASE": [float(x) for x in surface_normal_base],
            "BOARD_SURFACE_CENTROID_BASE": [float(x) for x in surface_centroid_base],
            "BOARD_PLANE_FIT_RMS_M": float(surf_rms),
            "BOARD_PLANE_FIT_MAX_ABS_M": float(surf_max),
            "BOARD_PLANE_INLIERS": int(first["inlier_count"]),
            "BOARD_PLANE_TOTAL_PTS": int(first["total"]),
            "BOARD_PLANE_SAMPLES_BASE": [[float(v) for v in p] for p in plane_samples_out],
            "BOARD_DEPTH_ROI": {
                "roi": [int(v) for v in roi],
                "source": str(roi_src),
                "z_range_m": [float(roi_z[0]), float(roi_z[1])],
                "capture_frames": int(capture_frames),
                "ransac_thresh_m": float(ransac_thresh),
                "depth_topic": str(depth_topic),
                "camera_frame": str(camera_frame),
            },
            "BOARD_DEPTH_VERIFY": verify_report,

            # Legacy-compatible parameters (board-local).
            "GRID_SIZE": legacy_grid,
            "BOARD_ORIGIN_X": legacy_origin_x,
            "BOARD_ORIGIN_Y": legacy_origin_y,
            "PICK_Z": pick_z,
            "PLACE_Z": place_z_legacy,
            "HOVER_Z": float(hover_z),
            "CAM_FX": float(cam_fx), "CAM_FY": float(cam_fy),
            "CAM_CX": float(cam_cx), "CAM_CY": float(cam_cy),

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
                }.get(board_mode, f"{len(board_sample_col_row)} points"),
            },
            "BOARD_SAMPLE_FORMAT": "samples are prompted as (col,row), stored internally as row/col; coords in board_frame",
            "BOARD_SAMPLES_BOARD": sample_records,
            "BOARD_ORIGIN_BOARD": [float(origin[0]), float(origin[1]), float(origin[2])],
            "BOARD_ROW_STEP_BOARD": [float(row_vec[0]), float(row_vec[1]), float(row_vec[2])],
            "BOARD_COL_STEP_BOARD": [float(col_vec[0]), float(col_vec[1]), float(col_vec[2])],
            "BOARD_CENTERS_14x10_BOARD": board_centers,
            # 逐格手动覆盖：步骤3重跑则清空，否则原样保留（board_centers 已叠加它）。
            "BOARD_CELL_OVERRIDES_BOARD": ([] if (3 in run) else (tcfg.get("BOARD_CELL_OVERRIDES_BOARD") or [])),
            "BOARD_GRID_FIT_RMS_XY_M": float(xy_rms),
            "BOARD_GRID_FIT_RMS_Z_M": float(grid_z_rms),

            "BOARD_SURFACE_Z": float(board_surface_z),
            "BOARD_SURFACE_PLANE": {
                "model": "z = a*x + b*y + c in board_frame",
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

    # 实测方块顶面放置图（步骤5新法的产物）。None 时不写，避免清空已有数据。
    if place_top_map is not None:
        config["tetris"]["BOARD_PLACE_TOP_MAP_14x10"] = [[float(v) for v in r] for r in place_top_map]
        config["tetris"]["BOARD_PLACE_TOP_INFO"] = place_top_info

    if bellows_cfg is not None:
        config["tetris"].update(bellows_cfg)

    # 最终合并写回（原子）：写完整集（含派生：放置图/中心点/legacy）。各步检查点已增量落盘，
    # 这里再写一次确保派生字段齐全；只更新本次产出的键，其余键原样保留。
    out_cfg = write_config_merge(config_path, existing_cfg, config["tetris"])

    print("\n🎉 增强标定完成！")
    print("配置已保存:", config_path)
    print("\n关键结果：")
    print(f"  板面深度采集: ROI={roi}（{roi_src}）, 内点 {first['inlier_count']}/{first['total']}")
    if verify_report is not None:
        print(f"  深度尺度比: {verify_report['depth_scale_ratio']:.4f}, "
              f"base-Z↔法向: {verify_report['baseZ_to_normal_deg']:.3f}°")
    print(f"  白板平面 residual RMS/max: {surf_rms*1000:.3f} / {surf_max*1000:.3f} mm")
    print(f"  board 法向(base) = board_frame Z 轴: {[round(float(v), 6) for v in surface_normal_base]}")
    print(f"  方块厚度 block_thickness(=PICK_Z, 平面回退用): {pick_z:.6f} m")
    print(f"  白板模式: {board_mode}, 采样点数: {len(board_sample_col_row)}")
    print(f"  白板高度模型: {height_model_type}, residual std/max: {height_std*1000:.3f} / {height_max_abs*1000:.3f} mm")
    print(f"  9/25/140 点网格 XY 拟合 RMS: {xy_rms*1000:.3f} mm")
    print(f"  底板无凸起平面拟合残差 std: {board_surf_std*1000:.3f} mm")
    if place_top_map is not None and place_top_info:
        print("  放置 Z(方块顶面深度): 覆盖 {}/{} 格, 插值 {} 格, 共 {} 批".format(
            place_top_info.get("covered_cells", "?"), ROWS * COLS,
            place_top_info.get("interpolated_cells", "?"), place_top_info.get("batches", "?")))
    print(f"  PLACE_Z extra margin: {place_z_margin*1000:.3f} mm")
    if bellows_cfg is not None:
        print(f"  波纹管补偿: 推荐 tcp_pick_offset_z={bellows_cfg['TCP_PICK_OFFSET_Z']:.4f} m, "
              f"tcp_place_offset_z={bellows_cfg['TCP_PLACE_OFFSET_Z']:.4f} m（记得同步到 launch）")
    print("\n建议：")
    print("  - 如果白板局部翘曲明显，优先用模式 C=140 点。")
    print("  - 放置 Z 由步骤5实测方块顶面得到（PLACE_Z=顶面+余量）；摆方块时尽量摆正、压实到凸起上。")
    print("  - 抓取 Z 由视觉/路径节点用 RealSense 深度在线获得，无需再标定抓取面。")
    print("  - 控制/路径节点把 BOARD_POSE_BASE 作为常量加载，放置读取 BOARD_CENTERS_14x10_BOARD 和 PLACE_Z_MAP_14x10。")
    print("  - ⚠️ 坐标系已改为 board_frame(由网格派生)；改动后必须重跑 pick_affine 重标单应性，再上机核对！")


if __name__ == "__main__":
    main()