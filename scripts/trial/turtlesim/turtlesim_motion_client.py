#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Empty
from agv_motion.msg import MoveTurtlesimAction
from agv_motion.msg import MoveTurtlesimGoal

class MoveTurtlesimClient:
    def __init__(self):
        self._ac = actionlib.SimpleActionClient("/turtlesim_action", MoveTurtlesimAction)
        self._ac.wait_for_server()
        rospy.loginfo("Server is up, can send a goal...")

        # add topic subscriber that can serve for cancellation
        self.sub = rospy.Subscriber("/cancel_move", Empty, self.cancel_move_callback)
    
    def cancel_move_callback(self, req):
        rospy.loginfo("Cancel goal")
        self._ac.cancel_goal()

    def send_goal(self, x_goal, y_goal):
        goal = MoveTurtlesimGoal()
        goal.x = x_goal
        goal.y = y_goal
        self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        rospy.loginfo("Goal " + str(goal) + " was sent")

    def done_callback(self, status, result):
        rospy.loginfo("Status: {}".format(status))
        rospy.loginfo("Result: {}".format(result))

    def feedback_callback(self, feedback):
        rospy.loginfo("Feedback: {}".format(feedback))
    

if __name__=='__main__':
    rospy.init_node('turtlesim_move_client') # , anonymous=True

    # read parameters defining goal coordinates
    x_goal = rospy.get_param('/x_goal')
    rospy.loginfo("x goal: {}".format(x_goal))

    y_goal = rospy.get_param('/y_goal')
    rospy.loginfo("y goal: {}".format(y_goal))

    client = MoveTurtlesimClient()
    client.send_goal(x_goal, y_goal)
    rospy.spin()