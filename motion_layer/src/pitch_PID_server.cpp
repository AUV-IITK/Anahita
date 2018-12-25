#include <pitch_PID_server.h>

pitchPIDAction::pitchPIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), pitch("PITCH")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&pitchPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&pitchPIDAction::preemptCB, this));
    goal_ = 0;

    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/mavros/imu/pitch", 1, &pitchPIDAction::callBack, this);
    pub_ = nh_.advertise<std_msgs::Bool>("/kill/angularvelocity/y", 1);

    pitch.setPID(2.0, 0, 0.65, 1);

    as_.start();
    ROS_INFO("pitch_PID_server Initialised");
}

pitchPIDAction::~pitchPIDAction(void)
{
}

void pitchPIDAction::goalCB()
{
    ROS_INFO("Inside goal callback");
    goal_ = as_.acceptNewGoal()->target_pitch;

    pitch.setReference(goal_);
    ROS_INFO("Goal set as referecnce");

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void pitchPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void pitchPIDAction::callBack(const std_msgs::Float32::ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive()) {
        return;
    }
  ROS_INFO("INSIDE CALLBACK -----------_");
    pitch.errorToPWM(msg->data);

    feedback_.current_pitch = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data <= goal_ + 0.2 && msg->data >= goal_ - 0.2) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        // as_.setSucceeded(result_);
        std_msgs::Bool msg_;
        msg_.data = true;
        pub_.publish(msg_);
    }

    nh_.setParam("/pwm_pitch", pitch.getPWM());
}
