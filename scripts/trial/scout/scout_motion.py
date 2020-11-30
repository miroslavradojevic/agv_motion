#!/usr/bin/env python
import rospy
import rosservice
import math
import csv
import os
import time
import tf

from webots_ros.srv import set_float
from webots_ros.srv import get_float
from nav_msgs.msg import Odometry

timeout_sec = 45.0
distance_limit = 5.0

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

# TODO: investigate why get_velocities breaks
def get_velocities(srv_dict):
    vel = {}
    for s in srv_dict:
        print(s)
        print(srv_dict[s])
        resp = srv_dict[s](True)
        # https://cyberbotics.com/doc/reference/ros-api
        vel[s] = resp.value
    return vel

def set_velocities(clients_dict, velocity_val):
    try:
        if isinstance(velocity_val, dict):
            for k in velocity_val:
                resp = clients_dict[k](velocity_val[k])
                if not resp.success:
                    rospy.logerr("failed setting " + k + " wheel velocity to " + velocity_val[k])
        else:
            for s in clients_dict:
                resp = clients_dict[s](velocity_val)
                if not resp.success:
                    rospy.logerr("failed setting " + s + " wheel velocity to " + velocity_val)
    
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


if __name__ == '__main__':
    rospy.init_node("scout_motion")

    service_list = rosservice.get_service_list()

    rospy.loginfo("Searching services of webots '/agilex_scout_*' node")

    # [s for s in service_list if "agilex_scout_" in s]
    # [r for r in service_names[0].split("/") if "agilex_scout_" in r]

    # set_velocity
    service_set_velocity = [s for s in service_list if all(
        c in s for c in ["agilex_scout_", "motor_", "set_velocity"])]

    # get_velocity
    service_get_velocity = [s for s in service_list if all(
        c in s for c in ["agilex_scout_", "motor_", "get_velocity"])]

    if len(service_set_velocity) == 0 or len(service_get_velocity) == 0:
        rospy.logwarn(
            "Could not find set/get_velocity services of '/agilex_scout_*' node")
        exit(1)

    # dict with clients used to set velocities
    set_velocity_clients = {}

    for srv_name in service_set_velocity:
        # extract dictionary key
        wheel_key = [r for r in srv_name.split("/") if "motor_" in r]
        wheel_key = wheel_key[0].replace("motor_", "")

        rospy.logwarn(str(wheel_key) + " --> " + str(srv_name))

        rospy.loginfo("Wait for " + srv_name)
        rospy.wait_for_service(srv_name)

        rospy.logwarn(str(wheel_key) + " --> " + str(srv_name))

        # create service client
        set_velocity_clients[wheel_key] = rospy.ServiceProxy(srv_name, set_float)

    # dict with clients used to get velocities
    get_velocity_clients = {}

    for srv_name in service_get_velocity:
        # extract dictionary key
        wheel_key = [r for r in srv_name.split("/") if "motor_" in r]
        wheel_key = wheel_key[0].replace("motor_", "")

        rospy.logwarn(str(wheel_key) + " --> " + str(srv_name))

        rospy.loginfo("Wait for " + srv_name)
        rospy.wait_for_service(srv_name)
        
        # create service client
        get_velocity_clients[wheel_key] = rospy.ServiceProxy(srv_name, set_float)

        # TODO: investigate why calling get_velocity client fails! 
        # getting velocity programatically fails, calling service from
        # the terminal works
        # try:
        #     resp = get_velocity_clients[wheel_key](True)
        #     print(str(wheel_key) + " -- " + str(resp.value))
        # except rospy.ServiceException as e:
        #         rospy.logerr("Service failed: " + str(e))
        

    # subscribe to robot odometry
    sub = rospy.Subscriber("/odom", Odometry, odometry_callback)

    rospy.logwarn("waiting to receive first odometry position...")
    while True:
        if received_position:
            rospy.loginfo("received")
            break

    # read velocity parameter
    vel_value = rospy.get_param("motion_velocity")
    fq_value = rospy.get_param("motion_frequency")



    rate = rospy.Rate(fq_value)

    x0 = x
    y0 = y
    path_xy = []
    t0 = rospy.Time.now().to_sec()


    if True:
        v_lin = 0.1
        v_ang = 1.5
        d_v_lin = 0.05

        while True: 
            print(v_lin, v_ang)

            vel_value = compute_velocities(v_lin, v_ang)
            set_velocities(set_velocity_clients, vel_value)

            v_lin = min(v_lin + d_v_lin, 5.0)

            path_xy.append((x, y))

            dist = abs(math.sqrt((x-x0) ** 2 + (y-y0) ** 2))  # euclidean distance

            rospy.loginfo("x = " + str(x) + " y = " + str(y) + " yaw = " + str(yaw) + " dist = " + str(dist))

            if dist > distance_limit:
                rospy.loginfo("reached distance " + str(distance_limit))
                break

            t1 = rospy.Time.now().to_sec()
            if t1-t0 > timeout_sec:
                rospy.loginfo("reached "+str(timeout_sec)+"sec timeout")
                break

            rate.sleep()

        # stop requires several set_velocity requests in a row, loop is a workaround
        for i in range(20):
            set_velocities(set_velocity_clients, 0.0)



    if False:

        while True:  # not rospy.is_shutdown():
            set_velocities(set_velocity_clients, vel_value)

            path_xy.append((x, y))

            dist = abs(math.sqrt((x-x0) ** 2 + (y-y0) ** 2))  # euclidean distance

            rospy.loginfo("x = " + str(x) + " y = " + str(y) + " yaw = " + str(yaw) + " dist = " + str(dist))

            if dist > distance_limit:
                rospy.loginfo("reached distance " + str(distance_limit))
                break

            t1 = rospy.Time.now().to_sec()
            if t1-t0 > timeout_sec:
                rospy.loginfo("reached "+str(timeout_sec)+"sec timeout")
                break

            rate.sleep()

        # stop requires several set_velocity requests in a row, loop is a workaround
        for i in range(20):
            set_velocities(set_velocity_clients, 0.0)

        # rotate
        while True:
            relative_angle_rad = 1.57 - yaw

            print(abs(relative_angle_rad))

            if abs(relative_angle_rad) < 0.02:
                print("reached")
                break

            rot_value = -1.0 * relative_angle_rad
            
            set_velocities(set_velocity_clients, {'fl': rot_value, 'fr': -rot_value, 'rl': rot_value, 'rr': -rot_value})
            
            rate.sleep()
            
        # stop
        for i in range(20):
            set_velocities(set_velocity_clients, 0.0)


    # export path to csv file
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.dirname(os.path.realpath(__file__))

    with open(os.path.join(outdir, str(__file__) + "_" + timestamp + ".csv"), 'w') as f:
        wr = csv.writer(f, delimiter=',')
        wr.writerows(path_xy)
