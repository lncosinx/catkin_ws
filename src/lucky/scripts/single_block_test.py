#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray, Bool

# 与 strategy_node.cpp 一致：Point.x = col, Point.y = row
BASE_SHAPES = {
    0: [(0, 0), (1, 0), (2, 0), (3, 0)],
    1: [(0, 0), (1, 0), (0, 1), (1, 1)],
    2: [(0, 0), (1, 0), (2, 0), (1, 1)],
    3: [(1, 0), (1, 1), (1, 2), (0, 2)],
    4: [(0, 0), (0, 1), (0, 2), (1, 2)],
    5: [(0, 0), (1, 0), (1, 1), (2, 1)],
    6: [(1, 0), (2, 0), (0, 1), (1, 1)],
}


def normalize_shape(shape):
    min_x = min(x for x, y in shape)
    min_y = min(y for x, y in shape)
    out = [(x - min_x, y - min_y) for x, y in shape]
    out.sort(key=lambda p: (p[1], p[0]))
    return out


def rotate_shape(base, way):
    if way == 0:
        rot = list(base)
    elif way == 1:
        rot = [(-y, x) for x, y in base]
    elif way == 2:
        rot = [(-x, -y) for x, y in base]
    elif way == 3:
        rot = [(y, -x) for x, y in base]
    else:
        raise ValueError('target_way must be 0/1/2/3')
    return normalize_shape(rot)


def compute_cells_from_center(shape_type, way, target_row, target_col, allow_round=False):
    if shape_type not in BASE_SHAPES:
        raise ValueError('unknown shape_type={}'.format(shape_type))

    coords = rotate_shape(BASE_SHAPES[shape_type], way)
    sum_r = int(round(target_row * 4.0))
    sum_c = int(round(target_col * 4.0))
    shape_sum_r = sum(y for x, y in coords)
    shape_sum_c = sum(x for x, y in coords)

    start_r_f = (sum_r - shape_sum_r) / 4.0
    start_c_f = (sum_c - shape_sum_c) / 4.0
    aligned = abs(start_r_f - round(start_r_f)) <= 1e-6 and abs(start_c_f - round(start_c_f)) <= 1e-6

    if not aligned:
        off_r = shape_sum_r / 4.0
        off_c = shape_sum_c / 4.0
        msg = (
            '目标中心 row={:.3f}, col={:.3f} 对 shape={}, way={} 不是合法的4格中心；'
            '该 way 的合法中心应为 row=整数+{:.2f}, col=整数+{:.2f}. '
            '计算得到 start_r={:.3f}, start_c={:.3f}。请改 target_row/target_col，或直接设置 target_cells。'
        ).format(target_row, target_col, shape_type, way, off_r, off_c, start_r_f, start_c_f)
        if not allow_round:
            raise ValueError(msg)
        rospy.logwarn(msg + ' allow_round_target_cells=true，继续四舍五入。')

    start_r = int(round(start_r_f))
    start_c = int(round(start_c_f))
    cells = [(start_r + y, start_c + x) for x, y in coords]
    return cells, sum(r for r, c in cells), sum(c for r, c in cells)


class SingleBlockTester:
    def __init__(self):
        rospy.init_node('single_block_tester', anonymous=True)
        self.robot_idle = False
        rospy.Subscriber('/robot_status', Bool, self.status_callback)
        self.plan_pub = rospy.Publisher('/tetris_plan', Int32MultiArray, queue_size=1, latch=False)
        self.has_sent = False

        self.target_row = rospy.get_param('~target_row', 6.0)
        self.target_col = rospy.get_param('~target_col', 3.5)
        self.target_way = int(rospy.get_param('~target_way', 0))
        self.use_detected_angle = bool(rospy.get_param('~use_detected_angle', True))
        self.override_pick_angle = int(rospy.get_param('~override_pick_angle', 0))
        self.use_extended_plan = bool(rospy.get_param('~use_extended_plan', True))
        self.target_cells_override = rospy.get_param('~target_cells', [])
        self.allow_round_target_cells = bool(rospy.get_param('~allow_round_target_cells', False))

        rospy.loginfo('⏳ 正在建立通信，请稍候 1 秒...')
        rospy.sleep(1.0)
        rospy.Subscriber('/vision/board_state', Int32MultiArray, self.vision_callback)
        rospy.loginfo('🎯 单块抓取/放置测试节点已启动，等待视觉结果...')
        rospy.loginfo('📌 目标放置参数: row=%.2f, col=%.2f, way=%d, use_detected_angle=%s, extended_plan=%s',
                      self.target_row, self.target_col, self.target_way, self.use_detected_angle, self.use_extended_plan)

    def status_callback(self, msg):
        self.robot_idle = (msg.data is False)

    def parse_single_block(self, data):
        if len(data) <= 147:
            return None
        num_blocks = int(data[147])
        if num_blocks != 1:
            rospy.logwarn_throttle(1.0, '当前检测到 %d 个方块，等待单块测试场景...', num_blocks)
            return None
        idx = 148
        remaining = len(data) - idx
        if remaining < 4:
            rospy.logwarn('视觉消息长度不足，无法解析单块信息。len=%d', len(data))
            return None
        stride = 6 if remaining >= 6 else 4
        shape_type = int(data[idx])
        pick_u = int(data[idx + 1])
        pick_v = int(data[idx + 2])
        detected_ang = int(data[idx + 3])
        if stride == 6:
            geom_u = int(data[idx + 4])
            geom_v = int(data[idx + 5])
            has_geom = True
        else:
            geom_u = pick_u
            geom_v = pick_v
            has_geom = False
        return shape_type, pick_u, pick_v, detected_ang, geom_u, geom_v, has_geom, stride

    def get_target_cells(self, shape_type):
        if self.target_cells_override:
            cells = [(int(x[0]), int(x[1])) for x in self.target_cells_override]
            if len(cells) != 4:
                raise ValueError('target_cells must contain exactly 4 cells')
            return cells, sum(r for r, c in cells), sum(c for r, c in cells)
        return compute_cells_from_center(shape_type, self.target_way, self.target_row, self.target_col, self.allow_round_target_cells)

    def vision_callback(self, msg):
        if self.has_sent:
            return
        parsed = self.parse_single_block(list(msg.data))
        if parsed is None:
            return
        shape_type, pick_u, pick_v, detected_ang, geom_u, geom_v, has_geom, stride = parsed
        pick_ang = detected_ang if self.use_detected_angle else self.override_pick_angle
        try:
            cells, sum_r, sum_c = self.get_target_cells(shape_type)
        except Exception as e:
            rospy.logerr('无法生成目标 cells：%s', str(e))
            return

        rospy.loginfo('锁定目标: shape=%d | pick=(%d,%d) geom=(%d,%d) has_geom=%s | detected_ang=%d° use_pick_ang=%d° stride=%d',
                      shape_type, pick_u, pick_v, geom_u, geom_v, has_geom, detected_ang, pick_ang, stride)
        rospy.loginfo('目标 cells=%s, center=(%.2f, %.2f), way=%d, sum_r=%d, sum_c=%d',
                      cells, sum_r / 4.0, sum_c / 4.0, self.target_way, sum_r, sum_c)

        fake_plan = Int32MultiArray()
        if self.use_extended_plan:
            payload = [1, shape_type, self.target_way, sum_r, sum_c, pick_u, pick_v, pick_ang, geom_u, geom_v]
            for r, c in cells:
                payload.extend([r, c])
            fake_plan.data = payload
            rospy.loginfo('将发送 extended-17-pick+geom 计划，len=%d data=%s', len(fake_plan.data), fake_plan.data)
        else:
            fake_plan.data = [1, shape_type, self.target_way, sum_r, sum_c, pick_u, pick_v, pick_ang]
            rospy.logwarn('将发送 old-7 计划，hybrid/target_cells 测试不建议使用。len=%d data=%s', len(fake_plan.data), fake_plan.data)

        rospy.loginfo('等待 /tetris_plan 至少有 1 个订阅者...')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.plan_pub.get_num_connections() == 0:
            rate.sleep()
        rospy.loginfo('检测到控制节点已订阅 /tetris_plan，准备发送测试计划。')
        if not self.robot_idle:
            rospy.logwarn_throttle(1.0, '控制节点还没有空闲，等待中...')
            return
        self.plan_pub.publish(fake_plan)
        rospy.loginfo('🚀 单块测试指令已发送给控制节点。')
        self.has_sent = True
        rospy.sleep(1.0)
        rospy.signal_shutdown('single test plan sent')


if __name__ == '__main__':
    try:
        SingleBlockTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass