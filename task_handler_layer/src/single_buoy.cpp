#include <single_buoy.h>

singleBuoy::singleBuoy(): forwardPIDClient("forwardPID"), sidewardPIDClient("sidewardPID"), 
                        anglePIDClient("turnPID"), upwardPIDClient("upwardPID") 
{
    forward_sub_ = nh_.subscribe("/anahita/x_coordinate", 1, &singleBuoy::forwardCB, this);
    sideward_sub_ = nh_.subscribe("/anahita/y_coordinate", 1, &singleBuoy::sidewardCB, this);
    upward_sub_ = nh_.subscribe("/anahita/z_coordinate", 1, &singleBuoy::upwardCB, this);
    angle_sub_ = nh_.subscribe("/mavros/imu/yaw", 1, &singleBuoy::angleCB, this);    
    angleGoalReceived = false;
    spin_thread = new boost::thread(boost::bind(&singleBuoy::spinThread, this));
}
singleBuoy::~singleBuoy() {}

void singleBuoy::setActive(bool status) {

    if (status) {
        ros::Duration(1).sleep();

        ROS_INFO("Waiting for sidewardPID server to start, task buoy.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal, task buoy.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);

        ///////////////////////////////////////////////////

        // ROS_INFO("Waiting for upwardPID server to start.");
        // upwardPIDClient.waitForServer();

        // ROS_INFO("upwardPID server started, sending goal.");
        // upwardPIDgoal.target_depth = 0;
        // upwardPIDClient.sendGoal(upwardPIDgoal);

        ///////////////////////////////////////////////////

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        while (!angleGoalReceived) {}

        ROS_INFO("anglePID server started, sending goal.");

        anglePIDGoal.target_angle = angle_;
        anglePIDClient.sendGoal(anglePIDGoal);

        /////////////////////////////////////////////////////
        ROS_INFO("SETTING FRWARD PWM TO 50");
        nh_.setParam("/pwm_surge", 50);

        while(forward_distance_ >= 40) {
            ROS_INFO("to maintain the distance move forward");
            continue;
        }
       
        std::cout<<forward_distance_<<std::endl;

        sidewardPIDClient.cancelGoal();
        nh_.setParam("/pwm_sway", 0);
        // upwardPIDClient.cancelGoal();
        // ros::Duration(6).sleep();

        nh_.setParam("/pwm_surge", 50);

        ros::Duration(6).sleep();
        //////////////////////////////////////////////////////
        nh_.setParam("/pwm_surge", -50);
        
        ros::Duration(8).sleep();
        
        ROS_INFO("SidewardPID Client sending goal again, task buoy.");
        sidewardPIDgoal.target_distance = 0;
        sidewardPIDClient.sendGoal(sidewardPIDgoal);
        
        // ros::Duration(5).sleep();
        // ROS_INFO("UpwardPID Client sending goal again, task buoy.");
        // upwardPIDgoal.target_depth = 0;
        // upwardPIDClient.sendGoal(upwardPIDgoal);

        //////////////////////////////////////////////////////

        ROS_INFO("ForwardPID Client sending goal again, task buoy.");
        nh_.setParam("/kill_signal", true);
        bool temp;
        nh_.getParam("/kill_signal", temp);
        ROS_INFO("kill signal is  %d", temp);

        
        // forwardPIDgoal.target_distance = 100;
        // forwardPIDClient.sendGoal(forwardPIDgoal);
        // while(forward_distance_ <= 100) {
        //     continue;
        // }
        // ros::Duration(2).sleep();

        // forwardPIDClient.cancelGoal();
        

        // nh_.setParam("/pwm_forward_right", 0);
        // nh_.setParam("/pwm_forward_left", 0);
        // nh_.setParam("/pwm_sideward_front", 0);
        // nh_.setParam("/pwm_sideward_back", 0);
	
	    // nh_.setParam("/kill_signal", true);

        anglePIDClient.cancelGoal();
    }
    else {
        spin_thread->join();
        forwardPIDClient.cancelGoal();
        anglePIDClient.cancelGoal();
    }
}

void singleBuoy::spinThread() {
    ros::spin();
}

void singleBuoy::forwardCB(const std_msgs::Float32ConstPtr &_msg) {
    forward_distance_ = _msg->data;
}

void singleBuoy::sidewardCB(const std_msgs::Float32ConstPtr &_msg) {
    sideward_distance_ = _msg->data;
}

void singleBuoy::upwardCB(const std_msgs::Float32Ptr &_msg) {
    depth_ = _msg->data;
}

void singleBuoy::angleCB(const std_msgs::Float32Ptr &_msg) {
    angle_ = _msg->data;
    angleGoalReceived = true;
}
