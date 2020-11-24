#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
import numpy
from std_srvs.srv import Empty

# TODO: encapsulate it in class
# class TurtleSim:
#     def __init__(self):
#         rospy.init_node("turtlesim_control")


x = 0
y = 0
yaw = 0
received_position = False


def pose_callback(pose_message):
    global x, y, yaw, received_position
    # http://docs.ros.org/en/melodic/api/turtlesim/html/msg/Pose.html
    # rostopic info /turtle1/pose
    # rosmsg show turtlesim/Pose
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta
    received_position = True

def move(publisher, speed, distance, is_forward):
    global x, y

    # http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
    # rostopic info /turtle1/cmd_vel
    # rosmsg show geometry_msgs/Twist
    velocity_message = Twist()
    
    # current location
    x0 = x
    y0 = y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)

    # distance_moved = 0.0

    loop_rate = rospy.Rate(10)  # velocity published at 10Hz

    while True:
        # Turtlesim moves forward
        publisher.publish(velocity_message) # this will effectively push the turtlesim

        distance_moved = abs(math.sqrt((x-x0) ** 2 + (y-y0) ** 2)) # euclidean distance

        rospy.loginfo("distance moved: {} | {}".format(distance_moved, distance))
        
        loop_rate.sleep()

        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    # stop
    velocity_message.linear.x = 0
    publisher.publish(velocity_message)

def rotate(publisher, angular_speed_degree, relative_angle_degree, clockwise):
    global yaw 

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    theta0 = yaw

    angular_speed = math.radians(abs(angular_speed_degree))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    # angle_moved_degree = 0.0

    loop_rate = rospy.Rate(30)  # velocity published at Hz

    t0 = rospy.Time.now().to_sec()

    while True:
        # Turtlesim rotates
        publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()

        angle_moved_degree = (t1-t0) * angular_speed_degree

        rospy.loginfo("angle moved: {}".format(angle_moved_degree))

        loop_rate.sleep()

        if (angle_moved_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break
    # stop
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

def go_to_goal(publisher, x_goal, y_goal):
    global x, y, yaw

    velocity_message = Twist()
    
    while True:
        K_linear = 1.0
        distance = abs(math.sqrt((x_goal-x)**2 + (y_goal-y)**2))
        linear_speed = K_linear * distance

        K_angular = 5.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = K_angular * (desired_angle_goal-yaw)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        publisher.publish(velocity_message)

        # rospy.loginfo("x: {} ({}), y: {} ({})".format(x, x_goal, y, y_goal))
        
        if (distance < 0.02):
            rospy.loginfo("reached")
            break
    
    # stop
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

def set_desired_orientation(publisher, angular_speed_degree, desired_angle_degree):
    relative_angle_rad = math.radians(desired_angle_degree) - yaw # difference between desired and current angle
    
    clockwise = True if relative_angle_rad < 0 else False
    
    rotate(publisher, angular_speed_degree, math.degrees(abs(relative_angle_rad)), clockwise)

def spiral(publisher, wk, rk=0.0):
    global x, y

    # current location
    x0 = x
    y0 = y
    
    velocity_message = Twist()
    
    loop_rate = rospy.Rate(1)
    
    while abs(math.sqrt((x-x0)**2 + (y-y0)**2))<2.0: #(x<10.0) and (y<10.0):
        rk = rk + 0.25
        velocity_message.linear.x = rk
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = wk
        publisher.publish(velocity_message)
        loop_rate.sleep()
    
    # stop
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

def clean(publisher):
    go_to_goal(publisher, 1, 1)
    set_desired_orientation(publisher, 40, 0)
    for i in range(4):
        move(publisher, 2.0, 1.0, True)
        
        set_desired_orientation(publisher, 40, 90)
        
        move(publisher, 4.0, 8.0, True)

        set_desired_orientation(publisher, 40, 0)

        move(publisher, 2.0, 1.0, True)

        set_desired_orientation(publisher, 40, -90)

        move(publisher, 4.0, 8.0, True)

        set_desired_orientation(publisher, 40, 0)


if __name__ == '__main__':
    try:
        rospy.init_node("turtlesim_motion", anonymous=True)

        velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

        rospy.logwarn("waiting to receive first position...")
        while True:
            if received_position:
                rospy.loginfo("received first position")
                break

        # read parameter that defines movement mode
        motion_mode = rospy.get_param('/turtlesim_motion_mode')
        rospy.loginfo("motion mode: {}".format(motion_mode))

        if motion_mode=="move":
            move(velocity_publisher, 1.0, 4.0, True)
        elif motion_mode=="rotate":
            rotate(velocity_publisher, 30, 90, True)
        elif motion_mode=="orientation":
            set_desired_orientation(velocity_publisher, 20, 275)
        elif motion_mode=="goal":
            go_to_goal(velocity_publisher, 1.5, 9.5)
        elif motion_mode=="goal_random":
            for i in range(5):
                x1 = numpy.random.uniform(2.0, 8.5)
                y1 = numpy.random.uniform(2.0, 8.5)
                rospy.loginfo("goal x:{}, y:{}".format(x1, y1))
                go_to_goal(velocity_publisher, x1, y1)
                rospy.sleep(2.0)
        elif motion_mode=="spiral":
            spiral(velocity_publisher, 2, 0.0)
        elif motion_mode=="clean":
            clean(velocity_publisher)
        else:
            rospy.logerr("Could not find motion mode: {}".format(motion_mode))
        
    except rospy.ROSInterruptException:
        rospy.logwarn("node terminated")
