#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

if __name__ == '__main__':
    rospy.init_node("number_publisher", anonymous=True)

    pub = rospy.Publisher("/number", Int64, queue_size=10)

    # read frequency through parameter
    # publish_frequency = rospy.get_param("/number_publish_frequency")

    rate = rospy.Rate(1)

    # number = rospy.get_param("/number_to_publish")
    # rospy.set_param("/another_param", "Hello")

    while not rospy.is_shutdown():
        msg = Int64()
        msg.data = 2
        pub.publish(msg)
        rate.sleep()

