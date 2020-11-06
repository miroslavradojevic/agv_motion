#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Message received: {}".format(msg))

if __name__ == '__main__':
    rospy.init_node("receiver")

    sub = rospy.Subscriber("/test_message", String, callback)

    rospy.spin() # will keep script running until node is stopped