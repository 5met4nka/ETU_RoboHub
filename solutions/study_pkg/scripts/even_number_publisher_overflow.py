#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, String

def even_number_publisher():
    rospy.init_node('even_number_publisher_node', anonymous=True)
    even_pub = rospy.Publisher('even_numbers', Int32, queue_size=10)
    overflow_pub = rospy.Publisher('number_overflow', String, queue_size=10)
    rate = rospy.Rate(10)
    
    even_number = 0
    
    while not rospy.is_shutdown():
        even_pub.publish(even_number)

        if even_number >= 100:
            overflow_pub.publish("Overflow occurred: resetting counter.")

            even_number = 0
        else:
            even_number += 2

        rate.sleep()

if __name__ == '__main__':
    try:
        even_number_publisher()
    except rospy.ROSInterruptException:
        pass
