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

    angle.setPID(-3.5, 0, -0.2, 1);

    as_.start();
    ROS_INFO("angle_PID_server Initialised");
}

anglePIDAction::~anglePIDAction(void)
{
}

void anglePIDAction::goalCB()
{
    ROS_INFO("Inside goal callback");
    goal_ = as_.acceptNewGoal()->target_angle;

    goalReceived = true;
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

    current_angle_ = msg->data;

    if (goalReceived) {
        //goal_ = goal_ + current_angle_;
        
         double reference_angle = 0;
         nh_.getParam("/reference_yaw", reference_angle);
         goal_ = goal_ + reference_angle;
        
        angle.setReference(goal_);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);

        goalReceived = false;
    }

    angle.errorToPWM(msg->data);

    feedback_.current_angle = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data <= goal_ + 1 && msg->data >= goal_ - 1) {
        // ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        // as_.setSucceeded(result_);
    }

    nh_.setParam("/pwm_yaw", angle.getPWM());
}
