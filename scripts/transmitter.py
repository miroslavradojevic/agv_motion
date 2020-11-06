#!/usr/bin/env python
import rospy 
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node("transmitter")
    
    # message name, message type, queue
    pub = rospy.Publisher("/test_message", String, queue_size=10)

    rate = rospy.Rate(2) # Hertz

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "This is test messate of the python publisher"
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Node {} stopped".format(rospy.get_name()))