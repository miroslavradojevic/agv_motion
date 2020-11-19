#include <ros.ros.h>
#include <actionlib/server/simple_action_server.h>

#include <agv_motion/CountUntilAction.h>
#include <agv_motion/CountUntilGoal.h>
#include <agv_motion/CountUntilResult.h>
#include <agv_motion/CountUntilFeedback.h>

class CountUntilServer {

    protected:
    ros::NodeHandle _nh;
    actionlib::SimpleActionServer<agv_motion::CountUntilAction> _as;
    int _counter;

    public:
    CountUntilServer():
        _as(_nh, 
            "/count_util", 
            boost::bind(&CountUntilServer::onGoal, this, _1), 
            false),
        _counter(0)
        {
            _as.start();
            ROS_INFO("Simple Action Server has been started");
        }

}