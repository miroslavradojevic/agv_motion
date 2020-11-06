#include <ros/ros.h>
#include <std_msgs/String.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "transmitter", ros::init_options::AnonymousName);

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/test_message", 10);

    ros::Rate rate(3); // ensure that while block executes at 3Hz

    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = "This is test message of the cpp publisher";
        pub.publish(msg);
        rate.sleep();
    }
}