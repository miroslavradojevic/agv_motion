#!/usr/bin/env python
from agv_motion.srv import AddTwoFloats
import rospy
import sys

def add_two_floats_client(x, y):
    # block until service is advertised
    rospy.wait_for_service("add_two_floats")
    try:
        # create client
        add_two_floats = rospy.ServiceProxy("add_two_floats", AddTwoFloats)
        resp = add_two_floats(x, y) # call the client
        return resp.sum
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))


if __name__ == '__main__':
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print("{} [x y]".format(sys.argv[0]))
        sys.exit(1)

    print("Requesting {}+{}".format(x, y))
    z = add_two_floats_client(x, y)
    print("{} + {} = {}".format(x, y, z))