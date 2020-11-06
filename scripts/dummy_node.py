#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    rospy.init_node("dummy")

    rospy.loginfo("{} node started...".format(rospy.get_name()))
    
    # rospy.sleep(1)
    
    rate = rospy.Rate(10) # 10Hz rate

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()