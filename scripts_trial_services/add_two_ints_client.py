#!/usr/bin/env python
import rospy
from rospy_tutorials.srv import AddTwoInts

if __name__ == '__main__':
    rospy.init_node("add_two_ints_client")

    # block until service is advertised, wait for the service
    rospy.wait_for_service("/add_two_ints")

    try:
        # create client
        add_two_ints = rospy.ServiceProxy("/add_two_ints", AddTwoInts)
        # call the client
        resp = add_two_ints(2, 6)
        rospy.loginfo("Sum is: " + str(resp.sum))
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
