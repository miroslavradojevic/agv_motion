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

}