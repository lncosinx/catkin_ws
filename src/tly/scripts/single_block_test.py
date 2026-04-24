#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32MultiArray, Bool

class SingleBlockTester:
    def __init__(self):
        rospy.init_node('single_block_tester', anonymous=True)
        self.robot_idle = False
        rospy.Subscriber('/robot_status', Bool, self.status_callback)

        # 单次测试不使用 latch，避免下次重新启动时旧计划被再次执行
        self.plan_pub = rospy.Publisher('/tetris_plan', Int32MultiArray, queue_size=1, latch=False)
        self.has_sent = False

        # 可通过 launch 调整目标放置位置与放置朝向
        self.target_row = rospy.get_param("~target_row", 6.0)
        self.target_col = rospy.get_param("~target_col", 3.5)
        self.target_way = int(rospy.get_param("~target_way", 0))  # 0/1/2/3 -> 0/90/180/270
        self.use_detected_angle = bool(rospy.get_param("~use_detected_angle", True))
        self.override_pick_angle = int(rospy.get_param("~override_pick_angle", 0))

        rospy.loginfo("⏳ 正在建立通信，请稍候 1 秒...")
        rospy.sleep(1.0)

        rospy.Subscriber('/vision/board_state', Int32MultiArray, self.vision_callback)
        rospy.loginfo("🎯 单块抓取/放置测试节点已启动，等待视觉结果...")
        rospy.loginfo(
            "📌 目标放置参数: row=%.2f, col=%.2f, way=%d, use_detected_angle=%s",
            self.target_row, self.target_col, self.target_way, self.use_detected_angle
        )
        rospy.loginfo("⚠️  检测到单个方块后将发送一次测试指令，然后自动退出。")

    def status_callback(self, msg):
        self.robot_idle = (msg.data == False)

    def vision_callback(self, msg):
        if self.has_sent:
            return

        data = list(msg.data)
        if len(data) <= 147:
            return

        num_blocks = data[147]
        if num_blocks != 1:
            rospy.logwarn_throttle(1.0, "当前检测到 %d 个方块，等待单块测试场景...", num_blocks)
            return

        idx = 148
        if len(data) < idx + 4:
            rospy.logwarn("视觉消息长度不足，无法解析单块信息。")
            return

        shape_type = int(data[idx])
        u = int(data[idx + 1])
        v = int(data[idx + 2])
        detected_ang = int(data[idx + 3])

        pick_ang = detected_ang if self.use_detected_angle else self.override_pick_angle
        sum_r = int(round(self.target_row * 4.0))
        sum_c = int(round(self.target_col * 4.0))

        rospy.loginfo(
            "锁定目标: shape=%d | pixel=(u:%d, v:%d) | detected_ang=%d° | use_pick_ang=%d°",
            shape_type, u, v, detected_ang, pick_ang
        )
        rospy.loginfo(
            "目标放置: row=%.2f, col=%.2f, way=%d -> sum_r=%d, sum_c=%d",
            self.target_row, self.target_col, self.target_way, sum_r, sum_c
        )

        # 兼容当前控制节点 / 策略节点的计划格式：
        # [总步数(1), 形状ID, 旋转状态, 放置重心sum_r, 放置重心sum_c, u, v, 抓取角度]
        fake_plan = Int32MultiArray()
        fake_plan.data = [1, shape_type, self.target_way, sum_r, sum_c, u, v, pick_ang]
        rospy.loginfo("等待 /tetris_plan 至少有 1 个订阅者...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.plan_pub.get_num_connections() == 0:
            rate.sleep()

        rospy.loginfo("检测到控制节点已订阅 /tetris_plan，准备发送测试计划。")
        if not self.robot_idle:
            rospy.logwarn_throttle(1.0, "控制节点还没有空闲，等待中...")
            return
        self.plan_pub.publish(fake_plan)
        rospy.loginfo("🚀 单块测试指令已发送给控制节点。")
        self.has_sent = True

        # 单次测试发完即退出；下次由你重新启动 launch
        rospy.sleep(1.0)
        rospy.signal_shutdown("single test plan sent")


if __name__ == '__main__':
    try:
        SingleBlockTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass