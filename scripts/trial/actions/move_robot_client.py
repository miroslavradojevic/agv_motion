#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Empty
from agv_motion.msg import MoveRobotAction
from agv_motion.msg import MoveRobotGoal

class MoveRobotClient:
    def __init__(self):
        self._ac = actionlib.SimpleActionClient("/move_robot", MoveRobotAction)
        self._ac.wait_for_server()
        rospy.loginfo("Server is up, can send a goal...")

        # add topic subscriber that can serve for cancellation
        self.sub = rospy.Subscriber("/cancel_move", Empty, self.cancel_move_callback)
    
    def cancel_move_callback(self, req):
        rospy.loginfo("Received message to cancel goal")
        self._ac.cancel_goal()


    def send_goal(self):
        goal = MoveRobotGoal()
        goal.position = 99
        goal.velocity = 2
        self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal was sent")

        # rospy.sleep(3.0)
        # self._ac.cancel_goal()

    def done_callback(self, status, result):
        rospy.loginfo("Status: {}".format(status))
        rospy.loginfo("Result: {}".format(result))

    def feedback_callback(self, feedback):
        rospy.loginfo("Feedback: {}".format(feedback))

if __name__=='__main__':
    rospy.init_node('move_robot_client')
    client = MoveRobotClient()
    client.send_goal()
    rospy.spin()