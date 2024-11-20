#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def even_number_publisher():
    rospy.init_node('even_number_publisher_node', anonymous=True)
    pub = rospy.Publisher('even_numbers_topic', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Гц
    num = 0

    while not rospy.is_shutdown():
        pub.publish(num)
        num += 2
        rate.sleep()

if __name__ == '__main__':
    try:
        even_number_publisher()
    except rospy.ROSInterruptException:
        pass
