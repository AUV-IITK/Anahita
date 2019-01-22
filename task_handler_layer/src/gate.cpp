#include <gate.h>
#include <std_msgs/String.h>

gateTask::gateTask(): sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID"), th(30) {}

gateTask::~gateTask() {}

bool gateTask::setActive(bool status) {
    if (status) {

        nh_.setParam("/enable_pressure", true);
        depth_stabilise.setActive(true, "reference");

        ROS_INFO("Waiting for sidewardPID server to start, Gate Task.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, Gate Task.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Time limit exceeded for the sideward PID");
            // return false;
        }
    
    	ROS_INFO("Gate Front completed successfully");

        return true;
    }
    else {
        sidewardPIDClient.cancelGoal();
    	anglePIDClient.cancelGoal();
        depth_stabilise.setActive(false, "reference");
        nh_.setParam("/kill_signal", true);
    }
}
