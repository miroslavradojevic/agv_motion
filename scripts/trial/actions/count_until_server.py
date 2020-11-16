#!/usr/bin/env python
import rospy
import actionlib

from agv_motion.msg import CountUntilAction
from agv_motion.msg import CountUntilGoal
from agv_motion.msg import CountUntilResult
from agv_motion.msg import CountUntilFeedback

class CountUntilServer:
    def __init__(self):  # constructor
        # initalize simple action server
        self._as = actionlib.SimpleActionServer(
            "/count_until", 
            CountUntilAction, 
            execute_cb=self.on_goal, # callback on goal
            auto_start=False)  # so that it is started manually
        self._as.start()
        self._counter = 0
        rospy.loginfo("Simple Action Server has been started")

    # with this function class can receive goals
    def on_goal(self, goal):
        rospy.loginfo("A goal has been received")
        rospy.loginfo(goal)

        max_number = goal.max_number
        wait_duration = goal.wait_duration

        self._counter = 0
        rate = rospy.Rate(1.0/wait_duration)

        success = False
        preempted = False

        while not rospy.is_shutdown(): # self._counter < max_number:
            self._counter += 1

            if self._as.is_preempt_requested(): # handle preempt request
                preempted = True
                break

            if self._counter > 9: # handle goal abortion
                break

            if self._counter >= max_number: # handle goal success
                success = True
                break

            rospy.loginfo(self._counter)
            
            feedback = CountUntilFeedback()
            feedback.percentage = round((float(self._counter) / float(max_number)) * 100.0, 1)
            self._as.publish_feedback(feedback) # send feedback to the client

            rate.sleep()

        result = CountUntilResult()
        result.count = self._counter  # server finished handling the goal
        rospy.loginfo("Send goal result to client")

        if preempted:
            rospy.loginfo("Preempted")
            self._as.set_preempted(result)
        elif success:
            rospy.loginfo("Success")
            # result will be sent when the goal state is set to succeeded
            self._as.set_succeeded(result)  # result is sent back using topic
        else:
            rospy.loginfo("Failure")
            self._as.set_aborted(result)



if __name__ == '__main__':
    rospy.init_node("count_until_server")
    server = CountUntilServer()
    rospy.spin()
