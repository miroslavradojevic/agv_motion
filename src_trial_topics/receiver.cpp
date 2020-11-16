#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String& msg) {
    ROS_INFO("Message received: %s", msg.data.c_str());
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "receiver");
    
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/test_message", 1000, callback);

    ros::spin(); // keep node running until it is shut down
}