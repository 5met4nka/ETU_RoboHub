#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def overflow_callback(msg):
    rospy.loginfo(f"Received overflow notification: {msg.data}")

def overflow_listener():
    rospy.init_node('overflow_listener_node', anonymous=True)
    rospy.Subscriber('number_overflow', String, overflow_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        overflow_listener()
    except rospy.ROSInterruptException:
        pass
