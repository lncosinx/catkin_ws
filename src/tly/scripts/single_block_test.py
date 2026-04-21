#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray

class SingleBlockTester:
    def __init__(self):
        rospy.init_node('single_block_tester', anonymous=True)
        
        # 🔴 修改 1：加上 latch=True，保证消息不丢失
        self.plan_pub = rospy.Publisher('/tetris_plan', Int32MultiArray, queue_size=1, latch=True)
        self.has_sent = False
        
        rospy.loginfo("⏳ 正在建立与控制器的底层通信链路，请稍候 1 秒...")
        
        # 🔴 修改 2：强制等待 1 秒钟，让 ROS Master 把 TCP 握手做完！
        rospy.sleep(1.0) 
        
        rospy.Subscriber('/vision/board_state', Int32MultiArray, self.vision_callback)
        rospy.loginfo("🎯 标定测试节点已启动！正在等待视觉画面...")
        rospy.loginfo("⚠️  警告：检测到方块后机械臂将立刻移动，请注意安全！")

    def vision_callback(self, msg):
        if self.has_sent:
            return  # 只执行一次，防止疯狂发指令机械臂抽搐

        data = msg.data
        if len(data) <= 147:
            return
            
        num_blocks = data[147]
        if num_blocks == 0:
            return

        # 提取视野中看到的【第一个】方块的信息
        idx = 148
        shape_type = data[idx]
        u = data[idx+1]
        v = data[idx+2]
        ang = data[idx+3]

        rospy.loginfo(f"锁定目标: 形状 ID {shape_type} | 像素点 (u:{u}, v:{v}) | 角度 {ang}°")

        # 构造一条伪造的动作指令欺骗 xarm_controller_node
        # 格式: [总步数(1), 形状ID, 旋转状态, 放置重心X, 放置重心Y, u, v, 抓取角度]
        # 放置重心给了 24 和 16 (相当于放到棋盘第 6 行第 4 列的安全区域)
        fake_plan = Int32MultiArray()
        fake_plan.data = [1, shape_type, 0, 24, 16, u, v, ang]

        self.plan_pub.publish(fake_plan)
        rospy.loginfo("🚀 强制抓取指令已发送给底层控制器！")
        self.has_sent = True

if __name__ == '__main__':
    try:
        SingleBlockTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass