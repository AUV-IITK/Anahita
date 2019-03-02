#include <gate.h>
#include <std_msgs/String.h>

gateTask::gateTask(): swayPIDClient("swayPID"), yawPIDClient("yawPID"), th(30) {}

gateTask::~gateTask() {}

bool gateTask::setActive(bool status) {
    if (status) {

        nh_.setParam("/enable_pressure", true);
        depth_stabilise.activate ("reference");

        ROS_INFO("Waiting for swayPID server to start, Gate Task");
        swayPIDClient.waitForServer();

        ROS_INFO("swayPID server started, sending goal, Gate Task");
        swayPIDgoal.target_sway = 0;
        swayPIDClient.sendGoal(swayPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for yawPID server to start.");
        yawPIDClient.waitForServer();

        ROS_INFO("yawPID server started, sending goal.");

        yawPIDGoal.target_yaw = 0;
        yawPIDClient.sendGoal(yawPIDGoal);

        /////////////////////////////////////////////////////

        if (!th.isAchieved(0, 15, "sway")) {
            ROS_INFO("Time limit exceeded for the sway PID");
        }
    
    	ROS_INFO("Gate Front completed successfully");

        return true;
    }
    else {
        swayPIDClient.cancelGoal();
    	yawPIDClient.cancelGoal();
        depth_stabilise.deActivate ();
        nh_.setParam("/kill_signal", true);
    }
}
