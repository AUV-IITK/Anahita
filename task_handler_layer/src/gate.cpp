#include <gate.h>
#include <std_msgs/String.h>

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

        if (!th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Time limit exceeded for the sideward PID");
            return false;
        }
    
    	ROS_INFO("Sideward Stabilised");

        nh_.setParam("/pwm_surge", 50);        
    	ROS_INFO("setting surge to 50");

        while(ros::ok()) {
            mtx.lock();
	        bool temp = forwardGoalReceived;
	        mtx.unlock();
            if (temp) {
                break;
            }           
        }

        while(ros::ok()) {
            mtx.lock();
	        double forward_distance = forward_distance_;
 	        mtx.unlock();
            if (forward_distance <= 250) {
                break;
            }
        }

        nh_.setParam("/pwm_surge", 0);	

        if (!th.isAchieved(0, 15, "sideward")) {
            ROS_INFO("Time limit exceeded for the sideward PID");
            return false;
        }
	    sidewardPIDClient.cancelGoal();
       	nh_.setParam("/current_task", "gate_bottom");

	    ros::Publisher task_pub = nh_.advertise<std_msgs::String>("/current_task", 1);

        std_msgs::String current_task;
        int pub_count = 0;
        ros::Rate loop_rate(10);

    	current_task.data = "gate_bottom";
	    while (ros::ok() && pub_count <= 5) {
		task_pub.publish(current_task);
		pub_count++;
		loop_rate.sleep();
	    }
	    pub_count = 0;
	    nh_.setParam("/current_task", "gate_bottom");
	    ROS_INFO("Current task: Gate Bottom");

        ROS_INFO("Forward distance less than 12");
        // ros::Duration(12).sleep();
        
	    nh_.setParam("/pwm_surge", 50);
        if (!th.isDetected("gate_bottom", 30)) {
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
