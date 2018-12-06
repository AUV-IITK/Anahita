#include <angle_PID_server.h>

anglePIDAction::anglePIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), angle("ANGLE")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&anglePIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&anglePIDAction::preemptCB, this));
    goal_ = 0;
    
    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/mavros/imu/yaw", 1, &anglePIDAction::callBack, this);
    pub_ = nh_.advertise<std_msgs::Bool>("/kill/angularvelocity/z", 1);

    angle.setPID(2.5, 0, 0.5, 1);

    as_.start();
    ROS_INFO("angle_PID_server Initialised");
}

anglePIDAction::~anglePIDAction(void)
{
}

void anglePIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_angle;

    angle.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void anglePIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void anglePIDAction::callBack(const std_msgs::Float32::ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive()) {
        return;
    }
    
    angle.errorToPWM(msg->data);

    feedback_.current_angle = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
        std_msgs::Bool msg_;
        msg_.data = true;
        pub_.publish(msg_);
    }

    nh_.setParam("/pwm_sideward_front_turn", angle.getPWM());
    nh_.setParam("/pwm_sideward_back_turn", -1*angle.getPWM());
}
