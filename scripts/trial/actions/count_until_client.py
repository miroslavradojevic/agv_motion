#!/usr/bin/env python
import rospy
import actionlib

from agv_motion.msg import CountUntilAction
from agv_motion.msg import CountUntilGoal

class CountUntilClient:
    def __init__(self):
        self._ac = actionlib.SimpleActionClient(
            "/count_until", 
            CountUntilAction)
        self._ac.wait_for_server()  # wait for server to be up (before sending a goal)
        rospy.loginfo("Action server is up, client can send new goals")

    def send_goal_and_get_result(self):
        goal = CountUntilGoal(max_number=15, wait_duration=0.75)
        self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        # done_cb is triggered when server sets goal to terminal state - succeeded
        rospy.loginfo("Goal has been sent")

        rospy.sleep(2)
        self._ac.cancel_goal()
        
        ## syncronous
        ## block until sever sends the results
        ## if it takes more than duration, then stop waiting and exit
        # success = self._ac.wait_for_result(rospy.Duration(30.0))  
        # if not success: 
        #     rospy.logwarn("Timeout reached")

        # rospy.loginfo(self._ac.get_result())

    def done_callback(self, status, result):
        # http://docs.ros.org/en/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        rospy.loginfo("Status is : " + str(status)) 
        rospy.loginfo("Result is : " + str(result))

    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)


if __name__ == '__main__':
    rospy.init_node('count_until_client')

    client = CountUntilClient()

    client.send_goal_and_get_result()

    rospy.spin() # needs to keep on runnning in asynchronous mode (when done_cb is used)