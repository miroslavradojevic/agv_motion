#!/usr/bin/env python
import rospy
import actionlib
from agv_motion.msg import MoveRobotAction
from agv_motion.msg import MoveRobotFeedback
from agv_motion.msg import MoveRobotResult

class MoveRobotServer:
    def __init__(self):
        self._as = actionlib.SimpleActionServer("/move_robot", MoveRobotAction, execute_cb=self.on_goal, auto_start=False)
        self._as.start()
        self._current_position = 50
        rospy.loginfo("Server has been started with position " + str(self._current_position))

    def send_feedback(self):
        feedback = MoveRobotFeedback()
        feedback.current_position = self._current_position
        self._as.publish_feedback(feedback)


    def on_goal(self, goal):
        rospy.loginfo("Goal received")
        rospy.loginfo(goal)

        goal_position = goal.position
        velocity = goal.velocity

        success = False
        preempted = False
        invalid_parameters = False
        message = ""

        if goal_position == self._current_position:
            success = True
            message = "Current position is already correct"

        rate = rospy.Rate(1.0)

        if goal_position<0 or goal_position >100:
            message = "Invalid goal position"
            invalid_parameters = True

        while not rospy.is_shutdown() and not success and not invalid_parameters:
            
            if self._as.is_preempt_requested():
                if goal_position == self._current_position:
                    message = "Preempted but already at goal position"
                    success = True
                    break
                else:
                    message = "Preempted and stopped execution"
                    preempted = True
                    break

            diff = goal_position - self._current_position

            if diff == 0: 
                # check goal termination
                message = "Success"
                success = True
                break
            elif diff < 0: 
                # move robot along the line
                if abs(diff) >= velocity:
                    self._current_position -= velocity
                else:
                    self._current_position -= abs(diff) # to perfectly match the goal position
            elif diff > 0: 
                # move robot along the line
                if diff >= velocity:
                    self._current_position += velocity
                else:
                    self._current_position += diff
            
            # publish feedback after current position is updated
            self.send_feedback()

            rate.sleep()

        # send result
        result = MoveRobotResult()
        result.position = self._current_position
        result.message = message 
        rospy.loginfo("Send goal result to client")

        if preempted:
            rospy.loginfo("Preempted")
            self._as.set_preempted(result)
        elif success:
            rospy.loginfo("Success")
            self._as.set_succeeded(result)
        else:
            rospy.loginfo("Aborted")
            self._as.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node("move_robot_server")
    server = MoveRobotServer()
    rospy.spin()
