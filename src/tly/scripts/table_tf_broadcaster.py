#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('table_tf_broadcaster')
    
    # 阻塞等待参数服务器加载配置
    while not rospy.has_param('/tetris/table_tf/x') and not rospy.is_shutdown():
        rospy.sleep(0.5)
        rospy.loginfo("Waiting for /tetris/table_tf parameters...")

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_msg = geometry_msgs.msg.TransformStamped()

    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "link_base"
    tf_msg.child_frame_id = "table_frame"

    # 从参数服务器读取标定工具算出来的参数
    tf_msg.transform.translation.x = rospy.get_param('/tetris/table_tf/x')
    tf_msg.transform.translation.y = rospy.get_param('/tetris/table_tf/y')
    tf_msg.transform.translation.z = rospy.get_param('/tetris/table_tf/z')

    roll = rospy.get_param('/tetris/table_tf/roll')
    pitch = rospy.get_param('/tetris/table_tf/pitch')
    yaw = rospy.get_param('/tetris/table_tf/yaw')

    q = quaternion_from_euler(roll, pitch, yaw)
    tf_msg.transform.rotation.x = q[0]
    tf_msg.transform.rotation.y = q[1]
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]

    broadcaster.sendTransform(tf_msg)
    rospy.loginfo("Published static transform for 'table_frame'.")
    rospy.spin()

if __name__ == '__main__':
    main()