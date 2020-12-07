#!/usr/bin/env python
import rospy
import rosservice
from webots_ros.srv import get_float

if __name__ == '__main__':
    rospy.init_node("test_get_velocity")
    services = rosservice.get_service_list()
    services = [s for s in services if all(c in s for c in ["agilex_scout_", "motor_", "get_velocity"])]

    for service in services:
        rospy.wait_for_service(service)
        rospy.loginfo(service)
        client = rospy.ServiceProxy(service, get_float)
        try:
            resp = client(True)
            rospy.loginfo(str(service) + " --> " + str(resp.value))
        except rospy.ServiceException as e:
                rospy.logerr("Service failed: " + str(e))
