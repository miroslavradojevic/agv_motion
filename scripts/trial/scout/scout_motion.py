#!/usr/bin/env python
import rospy
import rosservice
import math
import csv
import os
import time
import tf
import numpy as np

from webots_ros.srv import set_float
from webots_ros.srv import get_float
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

x = None
y = None
yaw = None 
received_position = False

def odometry_callback(odometry_message):
    global x, y, yaw, received_position
    
    # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
    x = odometry_message.pose.pose.position.x
    y = odometry_message.pose.pose.position.y
    
    # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
    quaternion = (odometry_message.pose.pose.orientation.x,
    odometry_message.pose.pose.orientation.y,
    odometry_message.pose.pose.orientation.z,
    odometry_message.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    received_position = True

def get_velocities(velocity_clients, verbose=False):
    vel = {}
    try:
        for s in velocity_clients:
            resp = velocity_clients[s]['get'](True)
            # https://cyberbotics.com/doc/reference/ros-api
            vel[s] = resp.value
    except rospy.ServiceException as e:
        rospy.logerr("Service failed: " + str(e))

    if verbose:
        rospy.loginfo(vel)

    return vel

def set_velocities(clients_dict, velocity_value):
    try:
        if isinstance(velocity_value, dict):
            for k in velocity_value:
                if k in clients_dict:
                    resp = clients_dict[k]['set'](velocity_value[k])
                    if not resp.success:
                        rospy.logerr("Failed setting " + k + " wheel velocity to " + velocity_value[k])
                else:
                    rospy.logerr("Could not find key " + str(k) + " in " + str(clients_dict.keys()))
        else:
            for k in clients_dict:
                resp = clients_dict[k]['set'](velocity_value)
                if not resp.success:
                    rospy.logerr("Failed setting " + k + " wheel velocity to " + velocity_value)
    
    except rospy.ServiceException as e:
            rospy.logerr("Service failed: " + str(e))

def compute_velocities(v_lin, v_ang):
    v_left = v_lin + v_ang
    v_right = v_lin - v_ang
    v = {}
    v['fl'] = v_left
    v['fr'] = v_right
    v['rl'] = v_left
    v['rr'] = v_right
    return v

def create_velocity_clients(service_list):
    
    clients = {}

    for srv_name in service_list:
        # extract wheel dictionary key
        wheel_key = [r for r in srv_name.split("/") if "motor_" in r]
        wheel_key = wheel_key[0].replace("motor_", "") 

        activity_key = [r for r in srv_name.split("/") if "_velocity" in r]
        activity_key = activity_key[0].replace("_velocity", "")

        rospy.loginfo("Wait for " + srv_name)
        # rospy.wait_for_service(srv_name)
        rospy.logwarn(str(wheel_key) + ", " + str(activity_key) + " -> " + str(srv_name))

        # create dictionary with service clients
        if wheel_key not in clients:
            clients[wheel_key] = {}

        clients[wheel_key][activity_key] = rospy.ServiceProxy(srv_name, set_float if activity_key == "set" else get_float)    

    return clients

def measure_velocities(velocity_clients, frequency):
    rate = rospy.Rate(frequency)
    while True: 
        get_velocities(velocity_clients, True)
        rate.sleep()
    
def straight(publisher, velocity_value, distance_limit, is_forward=True, frequency=10.0, timeout_sec=50.0): # velocity_clients
    global x, y, yaw

    # current location
    x0 = x
    y0 = y
    path_xy = []

    # http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
    # rostopic info /cmd_vel
    # rosmsg show geometry_msgs/Twist
    velocity_message = Twist()

    if is_forward:
        velocity_message.linear.x = abs(velocity_value)
    else:
        velocity_message.linear.x = -abs(velocity_value)
    
    rate = rospy.Rate(frequency)
    
    t0 = rospy.Time.now().to_sec()
    
    while True: 
        # Do not use service requests to wheels
        # set_velocities(velocity_clients, velocity_value)
        publisher.publish(velocity_message) # /cmd_vel topic message will effectively control Scout
        
        path_xy.append((x, y))

        dist = abs(math.sqrt((x-x0) ** 2 + (y-y0) ** 2))  # euclidean distance

        rospy.loginfo("x={:.2f} y={:.2f} yaw={:.2f}[{:.2f}]".format(x, y, yaw, math.degrees(yaw)))

        if dist >= distance_limit:
            rospy.logwarn("Reached distance " + str(distance_limit))
            break

        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec)+"sec timeout")
            break

        rate.sleep()

    # stop requires set_velocity requests until speeds reached zero
    # consecutive set_velocity requests, loop is a workaround
    # for i in range(20):
    #     set_velocities(velocity_clients, 0.000)

    rospy.loginfo("Stopping...")
    
    # attempt_count = 0
    # while np.array(get_velocities(velocity_clients, True).values()).max() > 0.02:
    #     set_velocities(velocity_clients, 0.000)
    #     attempt_count += 1
    # rospy.loginfo("Finished after " + str(attempt_count) + " attempts")

    # stop
    velocity_message.linear.x = 0
    publisher.publish(velocity_message)

    return path_xy

def rotate(publisher, angular_velocity_value, relative_angle_degree, clockwise=True, frequency=10.0, timeout_sec=50.0):
    global x, y, yaw 

    yaw_cummulative = 0.0
    yaw_prev = yaw

    path_xy = []

    # rot_value = angular_velocity_value if clockwise else -angular_velocity_value
    # velocity_value = {'fl': rot_value, 'fr': -rot_value, 'rl': rot_value, 'rr': -rot_value}

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = math.radians(abs(angular_velocity_value))

    if clockwise:
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    rate = rospy.Rate(frequency)

    t0 = rospy.Time.now().to_sec()

    while True:
        # set_velocities(velocity_clients, velocity_value)
        publisher.publish(velocity_message)

        path_xy.append((x, y))
        
        yaw_curr = yaw

        rospy.loginfo("x={:.2f} y={:.2f} yaw={:.2f}".format(x, y, yaw))

        yaw_cummulative += abs(yaw_curr - yaw_prev)
        yaw_prev = yaw_curr

        if yaw_cummulative >= math.radians(abs(relative_angle_degree)):
            rospy.loginfo("Reached yaw=" + str(yaw_cummulative) + " | " + str(math.radians(abs(relative_angle_degree))) + " " + str(relative_angle_degree))
            break
        
        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec)+"sec timeout")
            break

        rate.sleep()
        
    rospy.loginfo("Stopping...")


    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

    # attempt_count = 0
    # while np.array(get_velocities(velocity_clients, True).values()).max() > 0.02:
    #     set_velocities(velocity_clients, 0.000)
    #     attempt_count += 1
    # rospy.loginfo("Finished after " + str(attempt_count) + " attempts")

    return path_xy

def angdiff(a1, a2):
    # https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference
    return 180 - abs(abs(a1 - a2) - 180)

def angdiff180(a1, a2):
    # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    d = a2 - a1
    if d > 180:
        d -= 360
    elif d < -180:
        d += 360 
    return d

def go_to_goal(publisher, x_goal, y_goal, frequency=10.0, timeout_sec=50.0):
    global x, y, yaw

    velocity_message = Twist()

    path_xy = []

    rate = rospy.Rate(frequency)

    t0 = rospy.Time.now().to_sec() 

    while True:
        # compute linear and angular speed
        K_linear = 0.12
        distance = abs(math.sqrt((x_goal-x)**2 + (y_goal-y)**2))
        linear_speed = K_linear * distance

        K_angular = 0.12
        yaw_goal = math.atan2(y_goal-y, x_goal-x)
        
        v0 = np.array([x_goal-x, y_goal-y])
        v1 = np.array([math.cos(yaw), math.sin(yaw)])
        cw = np.cross(v1, v0)>0
        dang = abs(angdiff(math.degrees(yaw_goal), math.degrees(yaw))) # -180 to +180
        
        angular_speed = K_angular * math.radians(dang)

        if cw:
            rospy.loginfo("dYaw = +{:.2f}".format(dang))
        else:
            rospy.loginfo("dYaw = -{:.2f}".format(dang))
            angular_speed = -angular_speed
        
        rospy.loginfo("yaw={:.2f}| {:.2f}rad [{:.2f}| {:.2f}deg] dist={:.2f}".format(yaw, yaw_goal, math.degrees(yaw), math.degrees(yaw_goal), distance))

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        publisher.publish(velocity_message)

        rospy.loginfo("v_lin={:.2f}  v_ang={:.2f}".format(linear_speed, angular_speed))

        # vel_value = compute_velocities(linear_speed, angular_speed)
        # set_velocities(velocity_clients, vel_value)

        path_xy.append((x, y))

        if distance < 0.02:
            rospy.loginfo("Reached goal, distance=" + str(distance))
            break
        
        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec)+"sec timeout")
            break

        rate.sleep()
    
    rospy.loginfo("Stopping...")

    # attempt_count = 0
    # while np.array(get_velocities(velocity_clients, True).values()).max() > 0.01:
    #     set_velocities(velocity_clients, 0.000)
    #     attempt_count += 1
    # rospy.loginfo("Finished after " + str(attempt_count) + " attempts")

    # stop
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

    return path_xy

def print_position(frequency=10.0, timeout_sec=10.0):
    global x, y, yaw

    rate = rospy.Rate(frequency)

    t0 = rospy.Time.now().to_sec() 

    while True:
        rospy.loginfo("x={:.2f} y={:.2f} yaw={:.2f} [{:.2f}]".format(x, y, yaw, math.degrees(yaw)))

        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec)+"sec timeout")
            break

        rate.sleep()

def spiral(publisher, vel_ang, distance_limit, frequency=10.0, timeout_sec=50.0):
    global x, y, yaw

    path_xy = []

    # current location
    x0 = x
    y0 = y

    velocity_message = Twist()
    
    rate = rospy.Rate(frequency)

    t0 = rospy.Time.now().to_sec() 
    
    while abs(math.sqrt((x-x0)**2 + (y-y0)**2))<distance_limit:
        
        vel_lin = 0.05 + abs(math.sqrt((x-x0)**2 + (y-y0)**2))/distance_limit * 1.0
        velocity_message.linear.x = vel_lin
        velocity_message.linear.y = 0
        velocity_message.linear.z = 0
        velocity_message.angular.x = 0
        velocity_message.angular.y = 0
        velocity_message.angular.z = vel_ang
        publisher.publish(velocity_message)

        rospy.loginfo("x={:.2f} y={:.2f} yaw={:.2f}".format(x, y, yaw))
        
        path_xy.append((x, y))

        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec) + "sec timeout")
            break

        rate.sleep()
    
    # stop
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)

    return path_xy

def set_desired_orientation(publisher, desired_angle_degree, frequency=50.0, timeout_sec=50.0):
    # relative_angle_rad = math.radians(desired_angle_degree) - yaw # difference between desired and current angle
    # clockwise = True if relative_angle_rad < 0 else False
    # rotate(publisher, angular_velocity_value, math.degrees(abs(relative_angle_rad)), clockwise, frequency, timeout_sec)
    global yaw

    velocity_message = Twist()

    rate = rospy.Rate(frequency)

    t0 = rospy.Time.now().to_sec() 

    while True:
        yaw_goal = math.radians(desired_angle_degree)
        
        v0 = np.array([math.cos(yaw_goal), math.sin(yaw_goal)])
        v1 = np.array([math.cos(yaw), math.sin(yaw)])
        cw = np.cross(v1, v0)>0
        d_yaw = abs(angdiff(math.degrees(yaw_goal), math.degrees(yaw))) # -180 to +180
        
        K_angular = 0.15
        angular_speed = K_angular * math.radians(d_yaw)

        if cw:
            rospy.loginfo("dYaw = +{:.2f}".format(d_yaw))
        else:
            rospy.loginfo("dYaw = -{:.2f}".format(d_yaw))
            angular_speed = -angular_speed
        
        rospy.loginfo("yaw={:.2f}| {:.2f}rad [{:.2f}| {:.2f}deg]".format(yaw, yaw_goal, math.degrees(yaw), math.degrees(yaw_goal)))

        velocity_message.linear.x = 0
        velocity_message.angular.z = angular_speed
        publisher.publish(velocity_message)

        rospy.loginfo("v_ang={:.2f}".format(angular_speed))

        if d_yaw < 0.01:
            rospy.loginfo("Reached goal, yaw=" + str(yaw))
            break
        
        t1 = rospy.Time.now().to_sec()
        if t1-t0 >= timeout_sec:
            rospy.loginfo("Reached " + str(timeout_sec)+"sec timeout")
            break

        rate.sleep()
    
    rospy.loginfo("Stopping...")

    # attempt_count = 0
    # while np.array(get_velocities(velocity_clients, True).values()).max() > 0.01:
    #     set_velocities(velocity_clients, 0.000)
    #     attempt_count += 1
    # rospy.loginfo("Finished after " + str(attempt_count) + " attempts")

    # stop
    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    publisher.publish(velocity_message)


def clean(publisher, frequency=10.0, timeout_sec=500.0):
    path = go_to_goal(publisher, 3.0, 3.0)

    set_desired_orientation(publisher, -90)

    for i in range(2):
        path = path + straight(publisher, 1.0, 3.0, True)

        set_desired_orientation(publisher, 180)
        
        path = path + straight(publisher, 1.0, 0.5, True)

        set_desired_orientation(publisher, 90)

        path = path + straight(publisher, 1.0, 3.0, True)

        set_desired_orientation(publisher, 180)

        path = path + straight(publisher, 1.0, 0.5, True)

        set_desired_orientation(publisher, -90)
    
    return path

if __name__ == '__main__':
    try:
        rospy.init_node("scout_motion", anonymous=True)
        
        # Note: services are not used, control is done using /cmd_vel topic
        rospy.loginfo("Searching services of webots '/agilex_scout_*' node")
        service_list = rosservice.get_service_list()
        
        velocity_services = [s for s in service_list if all(c in s for c in ["agilex_scout_", "motor_", "set_velocity"])]
        velocity_services += [s for s in service_list if all(c in s for c in ["agilex_scout_", "motor_", "get_velocity"])]
        if len(velocity_services) == 0:
            rospy.logwarn("Could not find set/get_velocity services of /agilex_scout_* node")
            exit(1)

        # get_velocity services for all wheels
        vel_clients = create_velocity_clients(velocity_services)

        # publish Scout move commands
        velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # subscribe to robot odometry
        sub = rospy.Subscriber("/odom", Odometry, odometry_callback)

        rospy.logwarn("Waiting to receive first odometry position...")
        while True:
            if received_position:
                rospy.loginfo("Received")
                break
        
        # read velocity parameter
        velocity = rospy.get_param("motion_velocity")
        frequency = rospy.get_param("motion_frequency")
        motion_mode = rospy.get_param("motion_mode")
        distance = rospy.get_param("motion_dist")

        path = []

        if motion_mode == "straight":
            path = straight(velocity_publisher, velocity, distance, True, frequency) # forward
            path = straight(velocity_publisher, velocity, distance, False, frequency) # backward
        elif motion_mode == "rotate":
            angle_degrees = 75
            path = rotate(velocity_publisher, velocity, angle_degrees, True, frequency) # clockwise
            path = rotate(velocity_publisher, velocity, angle_degrees, False, frequency) # counter-clockwise
        elif motion_mode == "goal":
            x_goal = 5.0
            y_goal = 5.0
            path = go_to_goal(velocity_publisher, x_goal, y_goal)
        elif motion_mode == "spiral":
            path = spiral(velocity_publisher, 0.15, distance, frequency)
        elif motion_mode == "clean":
            path = clean(velocity_publisher)
        elif motion_mode == "measure":
            print_position(frequency, 100)
        else:
            rospy.logerr("Motion mode " + str(motion_mode) +" not recognized")

        # export path to csv file
        outdir = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(outdir, str(__file__) + "_" + time.strftime("%Y%m%d-%H%M%S") + ".csv"), 'w') as f:
            wr = csv.writer(f, delimiter=',')
            f.write("# mode="+str(motion_mode)+"\n")
            f.write("# velocity="+str(velocity)+"\n")
            f.write("# frequency="+str(frequency)+"\n")
            f.write("# distance="+str(distance)+"\n")
            wr.writerows(path)

    except rospy.ROSInterruptException:
        rospy.logwarn("Terminated")


    # dict with clients used to set velocities
    # vel_clients = create_velocity_clients(service_set_velocity, service_get_velocity)
    # vel_clients['fr']['set'] gives client object that would set velocity value on the front-right wheel 

    # dict with clients used to get velocities
    # get_velocity_clients = {}

    # for srv_name in service_get_velocity:
    #     # extract dictionary key
    #     wheel_key = [r for r in srv_name.split("/") if "motor_" in r]
    #     wheel_key = wheel_key[0].replace("motor_", "")
    #     rospy.wait_for_service(srv_name)
    #     get_velocity_clients[wheel_key] = rospy.ServiceProxy(srv_name, get_float)
    #     # TODO: investigate why calling get_velocity client fails! 
    #     # getting velocity programatically fails, calling service from
    #     # the terminal works
    #     # try:
    #     #     resp = get_velocity_clients[wheel_key](True)
    #     #     print(str(wheel_key) + " -- " + str(resp.value))
    #     # except rospy.ServiceException as e:
    #     #         rospy.logerr("Service failed: " + str(e))
        
    # v_lin = 0.1
    # v_ang = 1.5
    # d_v_lin = 0.05

    # while True: 
    #     vel_value = compute_velocities(v_lin, v_ang)
    #     set_velocities(set_velocity_clients, vel_value)

    #     v_lin = min(v_lin + d_v_lin, 5.0) # limitation

    #     path_xy.append((x, y))

    #     dist = abs(math.sqrt((x-x0) ** 2 + (y-y0) ** 2))  # euclidean distance

    #     rospy.loginfo("x = " + str(x) + " y = " + str(y) + " yaw = " + str(yaw) + " dist = " + str(dist))

    #     if dist > distance_limit:
    #         rospy.loginfo("reached distance " + str(distance_limit))
    #         break

    #     t1 = rospy.Time.now().to_sec()
    #     if t1-t0 > timeout_sec:
    #         rospy.loginfo("reached "+str(timeout_sec)+"sec timeout")
    #         break

    #     rate.sleep()