#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo
from tf.transformations import euler_from_matrix

def get_eef_pose(tf_buffer):
    """读取当前机械臂末端的物理坐标 (相对于 link_base)"""
    try:
        # 获取最新的变换
        trans = tf_buffer.lookup_transform('link_base', 'link_eef', rospy.Time(0), rospy.Duration(3.0))
        return np.array([trans.transform.translation.x,
                         trans.transform.translation.y,
                         trans.transform.translation.z])
    except Exception as e:
        rospy.logerr(f"TF 变换获取失败，请确认机械臂状态: {e}")
        return None

def main():
    rospy.init_node('calibration_tool_node', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0) # 等待 TF 树建立

    print("\n" + "="*50)
    print("=== 俄罗斯方块桌面坐标系 自动标定工具 (完美厚度分离版) ===")
    print("= 请在 UFactory Studio 中将机械臂调至【示教模式】 =")
    print("="*50 + "\n")

    # 步骤 1：记录视觉高度与内参
    input("[步骤 1] 请将机械臂移动到【视觉拍照起点高度】，然后按回车键...")
    start_pose = get_eef_pose(tf_buffer)
    
    print("正在监听相机内参 /camera/color/camera_info ...")
    try:
        cam_msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=3.0)
        cam_fx, cam_fy = cam_msg.K[0], cam_msg.K[4]
        cam_cx, cam_cy = cam_msg.K[2], cam_msg.K[5]
        print(f"✅ 成功获取相机内参: fx={cam_fx:.2f}, fy={cam_fy:.2f}")
    except:
        print("❌ 未收到相机消息，将使用代码中的默认内参。")
        cam_fx, cam_fy, cam_cx, cam_cy = 911.8016, 911.2428, 634.7139, 357.0596

    print("\n--- 下面建立真正的【桌面零平面 (Z=0)】 ---")
    input("[步骤 2] 请将吸盘紧贴【散落方块所在的桌面】的任意靠左下位置(作为P点原点)，按回车键...")
    P = get_eef_pose(tf_buffer)

    input("\n[步骤 3] 沿桌面向上(远离机器人底座方向)移动，紧贴桌面 (X点)，按回车键...")
    X = get_eef_pose(tf_buffer)

    input("\n[步骤 4] 回到 P 点附近，向右(白色底盘方向)移动，紧贴桌面 (Y点)，按回车键...")
    Y = get_eef_pose(tf_buffer)

    print("\n--- 下面获取【真实厚度】与【物理映射】 ---")
    input("[步骤 5] 拿一个方块平放在桌面上，将吸盘紧贴【方块的顶面】，按回车键...")
    Block_Pick = get_eef_pose(tf_buffer)

    input("\n[步骤 6] (定平面X/Y) 请将吸盘对准【白色底盘左上角第一格中心(凸起点)】，按回车键...")
    B_Origin = get_eef_pose(tf_buffer)

    input("\n[步骤 7] (定平面Z) 请将吸盘紧贴【白色底盘内任意无凸起的平坦表面】，按回车键...")
    Board_Surface = get_eef_pose(tf_buffer)

    # ===== 核心计算 1：求解桌面的倾斜补偿矩阵 =====
    vX = (X - P)
    vX = vX / np.linalg.norm(vX)
    vTemp = (Y - P)
    
    vZ = np.cross(vX, vTemp)
    vZ = vZ / np.linalg.norm(vZ)
    if vZ[2] < 0: vZ = -vZ # 强制让 Z 轴朝上

    vY = np.cross(vZ, vX)
    vY = vY / np.linalg.norm(vY)

    R_mat = np.eye(3)
    R_mat[:, 0] = vX
    R_mat[:, 1] = vY
    R_mat[:, 2] = vZ
    
    R_4x4 = np.eye(4)
    R_4x4[0:3, 0:3] = R_mat
    rpy = euler_from_matrix(R_4x4, axes='sxyz')

    # ===== 核心计算 2：坐标系逆投影 =====
    # 将所有的测量点投影到刚刚算出来的纯平桌面坐标系下
    Origin_table = np.dot(R_mat.T, B_Origin - P)
    Surface_table = np.dot(R_mat.T, Board_Surface - P)
    Pick_table = np.dot(R_mat.T, Block_Pick - P)
    Cam_table = np.dot(R_mat.T, start_pose - P)

    # 解析出完美参数
    # X和Y必须用带有凸起的那个原点坐标
    board_x = Origin_table[0]
    board_y = Origin_table[1]
    
    # 厚度必须用无凸起的平面坐标
    board_thickness = Surface_table[2]  
    block_thickness = Pick_table[2]
    table_z_in_cam = Cam_table[2]

    # 根据物理规律计算吸盘的运动高度
    pick_z = block_thickness                            # 抓取高度 = 方块顶面
    place_z = board_thickness + block_thickness         # 放置高度 = 底盘表面 + 方块顶面
    hover_z = place_z + 0.1                             # 安全高度 = 放置上方 10cm

    # ===== 生成配置文件 =====
    config = {
        'tetris': {
            'table_tf': {
                'x': float(P[0]), 'y': float(P[1]), 'z': float(P[2]),
                'roll': float(rpy[0]), 'pitch': float(rpy[1]), 'yaw': float(rpy[2])
            },
            'GRID_SIZE': 0.017,
            'BOARD_ORIGIN_X': float(board_x), 
            'BOARD_ORIGIN_Y': float(board_y), 
            'PICK_Z': float(pick_z),                 # 动态计算出的抓取高度
            'PLACE_Z': float(place_z),               # 动态计算出的放置高度
            'HOVER_Z': float(hover_z),               
            'CAM_FX': float(cam_fx), 'CAM_FY': float(cam_fy),
            'CAM_CX': float(cam_cx), 'CAM_CY': float(cam_cy),
            'TABLE_Z_IN_CAMERA': float(table_z_in_cam)
        }
    }

    with open('/root/catkin_ws/src/tly/config/tetris_config.yaml', 'w', encoding='utf-8') as f:
        yaml.dump(config, f, allow_unicode=True, default_flow_style=False)

    print("\n🎉 标定完成！极其精确的物理高度计算如下：")
    print(f"🔹 测得方块自身厚度: {block_thickness*1000:.1f} mm")
    print(f"🔹 测得白色底盘纯平表面厚度: {board_thickness*1000:.1f} mm")
    print(f"🎯 最终系统抓取高度 (PICK_Z): {pick_z:.4f} m")
    print(f"🎯 最终系统放置高度 (PLACE_Z): {place_z:.4f} m")
    print("参数已自动保存为 tetris_config.yaml。您可以开始完美装配了！")

if __name__ == '__main__':
    main()