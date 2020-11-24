#!/usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from agv_motion.msg import MoveTurtlesimAction
from agv_motion.msg import MoveTurtlesimFeedback
from agv_motion.msg import MoveTurtlesimResult


class TurtlesimMotionServer:
    def __init__(self):
        self._as = actionlib.SimpleActionServer("/turtlesim_action", MoveTurtlesimAction, execute_cb=self.on_goal, auto_start=False)
        self._as.start()

        # parameters
        self.TOLERANCE = 0.01
        self.XMIN = 1.0
        self.XMAX = 10.0
        self.YMIN = 1.0
        self.YMAX = 10.0
        self.K_LINEAR = 1.0
        self.K_ANGULAR = 3.0


        self._x = None
        self._y = None
        self._yaw = None
        self._received_position = False
        self._trajectory_length = 0

        self._turtlesim_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self._turtlesim_sub = rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        rospy.logwarn("waiting position...")
        while True:
            if self._received_position:
                rospy.loginfo("received position")
                break
        
        rospy.loginfo("Server has been started with robot at ({}, {} | {})".format(self._x, self._y, self._yaw))

    def send_feedback(self):
        feedback = MoveTurtlesimFeedback()
        feedback.trajectory_length = self._trajectory_length
        self._as.publish_feedback(feedback)

    def on_goal(self, goal):
        rospy.loginfo("Goal " + str(goal) + " received")

        goal_x = goal.x
        goal_y = goal.y

        success = False
        preempted = False
        invalid_parameters = False
        out_of_boundaries = False
        message = ""

        if abs(math.sqrt((goal_x-self._x)**2 + (goal_y-self._y)**2)) < self.TOLERANCE:
            message = "Current position is already at the goal"
            success = True

        if not (self.XMIN <= goal_x <= self.XMAX) or not (self.YMIN <= goal_y <= self.YMAX):
            message = "Invalid goal position"
            invalid_parameters = True

        rate = rospy.Rate(10.0)

        self._trajectory_length = 0.0
        velocity_message = Twist()

        while not rospy.is_shutdown() and not success and not invalid_parameters and not out_of_boundaries:
            
            if self._as.is_preempt_requested():
                if abs(math.sqrt((goal_x-self._x)**2 + (goal_y-self._y)**2)) < self.TOLERANCE:
                    message = "Preempted but already at goal position"
                    success = True
                    break
                else:
                    message = "Preempted and stopped execution"
                    preempted = True
                    break
            
            if not (self.XMIN <= self._x <= self.XMAX) or not (self.YMIN <= self._y <= self.YMAX):
                message = "Out of boundaries"
                out_of_boundaries = True
                break

            diff = abs(math.sqrt((goal_x-self._x)**2 + (goal_y-self._y)**2)) 
            desired_angle_goal = math.atan2(goal_y-self._y, goal_x-self._x)
            
            if diff < self.TOLERANCE:
                message = "Success - reached the goal"
                success = True
                break
            else:
                self._trajectory_length += diff

                # execute move to goal
                linear_speed = self.K_LINEAR * diff
                angular_speed = self.K_ANGULAR * (desired_angle_goal-self._yaw)

                velocity_message.linear.x = linear_speed
                velocity_message.angular.z = angular_speed

                self._turtlesim_pub.publish(velocity_message)
            
            # publish after each update
            self.send_feedback()

            rate.sleep() 

        # stop
        velocity_message.linear.x = 0
        velocity_message.angular.z = 0
        self._turtlesim_pub.publish(velocity_message)

        # send result
        result = MoveTurtlesimResult()
        result.message = message 
        rospy.loginfo("Send goal result to client")

        if preempted:
            rospy.loginfo("Preempted")
            self._as.set_preempted(result)
        elif success:
            rospy.loginfo("Success")
            self._as.set_succeeded(result)
        elif out_of_boundaries:
            rospy.loginfo("Aborted - out of boundaries")
            self._as.set_aborted(result)
        else:
            rospy.loginfo("Aborted - invalid goal parameters")
            self._as.set_aborted(result)


    def pose_callback(self, pose_message):
        self._x = pose_message.x
        self._y = pose_message.y
        self._yaw = pose_message.theta
        self._received_position = True

if __name__ == '__main__':
    rospy.init_node("turtlesim_move_server") # , anonymous=True
    server = TurtlesimMotionServer()
    rospy.spin()