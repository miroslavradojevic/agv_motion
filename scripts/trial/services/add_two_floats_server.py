#!/usr/bin/env python
import rospy
from agv_motion.srv import AddTwoFloats
from agv_motion.srv import AddTwoFloatsResponse

def handle_add_two_floats(req):
    res = req.a+req.b
    rospy.loginfo("{} + {} = {}".format(req.a, req.b, res))
    return AddTwoFloatsResponse(res)


def add_two_floats_server():
    rospy.init_node("add_two_floats_server")
    service = rospy.Service("/add_two_floats", AddTwoFloats, handle_add_two_floats)
    rospy.loginfo("Service 'add_two_floats' started...")
    rospy.spin()

if __name__ == '__main__':
    add_two_floats_server()