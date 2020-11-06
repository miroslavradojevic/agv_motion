#include <ros/ros.h>

int main (int argc, char** argv) {
    ros::init(argc, argv, "dummy_cpp");

    ros::NodeHandle nh;

    std::string node_name = ros::this_node::getName();

    ROS_INFO("node %s has been started", node_name.c_str());

    // ros::Duration(1.0).sleep();

    ros::Rate rate(10);

    while (ros::ok()) {
        ROS_INFO("Hello");
        rate.sleep();
    }

}