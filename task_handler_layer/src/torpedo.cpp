#include <torpedo.h>

Torpedo::Torpedo(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID"), th(15) {}

Torpedo::~Torpedo() {}

void Torpedo::setActive(bool status) {

    if (status) {

        ROS_INFO("Waiting for sidewardPID server to start, task torpedo.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, task torpedo.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for upwardPID server to start, task torpedo.");
        upwardPIDClient.waitForServer();

        ROS_INFO("upwardPID server started, sending goal, task torpedo.");
        upwardPIDgoal.target_depth = 0; // for gazebo
        upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start, task torpedo.");
        anglePIDClient.waitForServer();

        ROS_INFO("anglePID server started, sending goal, task torpedo.");

        anglePIDGoal.target_angle = 0;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////

        ROS_INFO("ForwardPID Client sending goal, task torpedo.");        
        forwardPIDgoal.target_distance = 50;
        forwardPIDClient.sendGoal(forwardPIDgoal);

        th.isAchieved(50, 15, "forward");

        ROS_INFO("Killing the thrusters");
	    nh_.setParam("/kill_signal", true);

        forwardPIDClient.cancelGoal();

        // fire the torpedo

        ROS_INFO("ForwardPID Client sending goal again, task torpedo.");        
        forwardPIDgoal.target_distance = 100;
        forwardPIDClient.sendGoal(forwardPIDgoal);
    }
    else {
        forwardPIDClient.cancelGoal();
        upwardPIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        anglePIDClient.cancelGoal();
        ROS_INFO("Closing Torpedo");
    }
}
