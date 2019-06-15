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
    sub_ = nh_.subscribe("/anahita/imu/pitch", 1, &pitchPIDAction::callBack, this);

    double p, i, d, band;

    nh_.getParam("/pitch/p", p);
    nh_.getParam("/pitch/i", i);
    nh_.getParam("/pitch/d", d);
    nh_.getParam("/pitch/band", band);

    pitch.setPID(p, i, d, band);

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

    pitch.errorToPWM(msg->data);

    feedback_.current_pitch = msg->data;
    as_.publishFeedback(feedback_);

    // if (msg->data <= goal_ + 0.2 && msg->data >= goal_ - 0.2) {
        // ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        // as_.setSucceeded(result_);
    // }

    nh_.setParam("/pwm_pitch", pitch.getPWM());
}
