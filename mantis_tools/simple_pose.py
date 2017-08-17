#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped

msg_out = PoseStamped()

def timer_callback(et):
  msg_out.header.stamp = et.current_real
  pose_pub.publish(msg_out)


if __name__ == '__main__':
  # Initialize
  node = rospy.init_node('pub_py')
  pose_pub = rospy.Publisher('/mavel/setpoint/position', PoseStamped, queue_size=10)

  # Loop here until quit
  try:
    rospy.loginfo("Started publisher node...")
    timer_msg = rospy.Timer(rospy.Duration(1.0 / 50.0), timer_callback)

    msg_out.header.frame_id = "world"
    msg_out.pose.position.z = 1.0
    msg_out.pose.position.x = 0.0
    msg_out.pose.orientation.w = 1.0

    rospy.spin()

  except rospy.ROSInterruptException:
    # Shutdown
    rospy.loginfo("Shutting down subscriber!")
    timer_msg.shutdown()
    node.shutdown()
