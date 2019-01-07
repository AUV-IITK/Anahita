#include <gate.h>

gateTask::gateTask(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                    anglePIDClient("turnPID"), th(30), upwardPIDClient("upwardPID") {
	forward_sub_ = nh_.subscribe("/anahita/y_coordinate", 1, &gateTask::forwardCB, this);
}

gateTask::~gateTask() {}

bool gateTask::setActive(bool status) {
    if (status) {
        ROS_INFO("Waiting for sidewardPID server to start, Gate Task.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, Gate Task.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        // ROS_INFO("Waiting for upwardPID server to start, Gate Task.");
        // upwardPIDClient.waitForServer();

        // ROS_INFO("upwardPID server started, sending goal, Gate Task.");
        // upwardPIDgoal.target_depth = 0;
        // upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        nh_.setParam("/pwm_surge", 50);

        while(ros::ok()) {
            mtx.lock();
            if (forwardGoalReceived) {
                break;
            }
            mtx.unlock();
        }

        while(ros::ok()) {
            mtx.lock();
            if (forward_distance_ <= 200) {
                break;
            }
            mtx.unlock();
        }

        nh_.setParam("/pwm_surge", 0);	

        if (th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Time limit exceeded for the sideward PID");
            return false;
        }
    	nh_.setParam("/current_task", "gate_bottom");

        ROS_INFO("Forward distance less than 12");
        // ros::Duration(12).sleep();
        nh_.setParam("/pwm_surge", 50);
        if (!th.isDetected("gate_bottom", 15)) {
            ROS_INFO("Unable to detect gate");
            return false;
        }
        return true;
    }
    else {
        sidewardPIDClient.cancelGoal();
        // upwardPIDClient.cancelGoal();
    	anglePIDClient.cancelGoal();
        nh_.setParam("/kill_signal", true);
    }
}

void gateTask::forwardCB (const std_msgs::Float32ConstPtr &_msg) {
    mtx.lock();
    forward_distance_ = _msg->data;
    forwardGoalReceived = true;
    mtx.unlock();
}
