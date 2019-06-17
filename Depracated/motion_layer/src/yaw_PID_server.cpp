#include <yaw_PID_server.h>

yawPIDAction::yawPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), yaw("YAW")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&yawPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&yawPIDAction::preemptCB, this));
    goal_ = 0;

    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/imu/yaw", 1, &yawPIDAction::callBack, this);

    double p, i, d, band;

    nh_.getParam("/yaw/p", p);
    nh_.getParam("/yaw/i", i);
    nh_.getParam("/yaw/d", d);
    nh_.getParam("/yaw/band", band);

    yaw.setPID(p, i, d, band                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        );

    as_.start();
    ROS_INFO("yaw_PID_server Initialised");
}

yawPIDAction::~yawPIDAction(void)
{
}

void yawPIDAction::goalCB()
{
    // ROS_INFO("Inside goal callback");
    goal_ = as_.acceptNewGoal()->target_yaw;

    goalReceived = true;
}

void yawPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());

    // set the action state to preempted
    as_.setPreempted();
}

void yawPIDAction::callBack(const std_msgs::Float32::ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive()) {
        return;
    }

    current_yaw_ = msg->data;

    if (goalReceived) {
        
        bool use_local_yaw = false;
        nh_.getParam("/use_local_yaw", use_local_yaw);

        bool use_reference_yaw = false;
        nh_.getParam("/use_reference_yaw", use_reference_yaw);

        bool disable_imu = false;
        nh_.getParam("/disable_imu", disable_imu);
        
        if (use_reference_yaw) {
            double reference_yaw = 0;
            nh_.getParam("/reference_yaw", reference_yaw);
            goal_ = goal_ + reference_yaw;
            ROS_INFO("Reference Yaw used");
        }
        else if (use_local_yaw) {
            double local_yaw = 0;
            nh_.getParam("/local_yaw", local_yaw);
            goal_ = goal_ + local_yaw;
            ROS_INFO("Local Yaw Used");
        }
        else if (disable_imu) {
            goal_ = goal_;
            ROS_INFO("Line yaw USed");	
        }
        else {
            goal_ = goal_ + current_yaw_;
            ROS_INFO("Current Yaw Used");
        }
        
        yaw.setReference(goal_);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);

        goalReceived = false;
    }

    yaw.errorToPWM(msg->data);

    feedback_.current_yaw = msg->data;
    as_.publishFeedback(feedback_);

    nh_.setParam("/pwm_yaw", yaw.getPWM());
}
