#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <agv_motion/CountUntilAction.h>
#include <agv_motion/CountUntilGoal.h>
#include <agv_motion/CountUntilResult.h>
#include <agv_motion/CountUntilFeedback.h>

class CountUntilClient
{
protected:
    ros::NodeHandle _nh;
    actionlib::SimpleActionClient<agv_motion::CountUntilAction> _ac;

public:
    CountUntilClient() : _ac("/count_until", true)
    {
        ROS_INFO("Wait for the action server to start...");

        _ac.waitForServer();

        ROS_INFO("Server is up.");
    }

    void sendGoal()
    {
        agv_motion::CountUntilGoal goal;
        goal.max_number = 12;
        goal.wait_duration = 0.3;
        _ac.sendGoal(
            goal,
            boost::bind(&CountUntilClient::doneCb, this, _1, _2), // necessary when using callbacks in class
            boost::bind(&CountUntilClient::activeCb, this),
            boost::bind(&CountUntilClient::feedbackCb, this, _1));
        ROS_INFO("Goal has been sent.");

        // Cancel the goal after 2 seconds
        // ros::Duration(2.0).sleep();
        // _ac.cancelGoal();
    }

    // when the goal reached terminal state (preempted, succeeded, aborted)
    void doneCb(const actionlib::SimpleClientGoalState &state,
                const agv_motion::CountUntilResultConstPtr &result)
    {
        ROS_INFO("Finished in state: %s", state.toString().c_str());
        ROS_INFO("Count result: %d", (int)result->count);
    }

    // when the goal went active
    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    void feedbackCb(const agv_motion::CountUntilFeedbackConstPtr &feedback)
    {
        ROS_INFO("Feedback received. Percentage: %lf", feedback->percentage);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "count_until_client");

    ROS_INFO("Starting client...");

    CountUntilClient client;

    client.sendGoal(); // send goal using client

    ros::spin();
}