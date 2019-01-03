#include <gate.h>

gateTask::gateTask(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                    anglePIDClient("turnPID"), th(15) {}

gateTask::~gateTask() {}

void gateTask::setActive(bool status) {
    if (status) {
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

        nh_.setParam("/pwm_surge", 100);
        nh_.setParam("/pwm_surge", 100);

        sidewardPIDClient.cancelGoal();
        ros::Duration(6).sleep();

        anglePIDClient.cancelGoal();

        nh_.setParam("/kill_signal", true);
    }
    else { }
}
