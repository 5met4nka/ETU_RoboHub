#!/usr/bin/env python3

import rospy
import time

def time_publisher():
    rospy.init_node('time_publisher_node', anonymous=True)
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        rospy.loginfo(f"Current time: {current_time}")
        rate.sleep()

if __name__ == '__main__':
    try:
        time_publisher()
    except rospy.ROSInterruptException:
        pass
