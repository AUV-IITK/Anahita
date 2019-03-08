#include <heave_PID_server.h>

heavePIDAction::heavePIDAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name), heave("HEAVE")
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&heavePIDAction::goalCB, this));
   
    goal_ = 0;

    // subscribe to the data topic of interest
    sub_ = nh_.subscribe("/anahita/z_coordinate", 1, &heavePIDAction::callback, this);
    
    double p, i, d, band;

    nh_.getParam("/heave/p", p);
    nh_.getParam("/heave/i", i);
    nh_.getParam("/heave/d", d);
    nh_.getParam("/heave/band", band);

    heave.setPID(p, i, d, band);

    as_.start();
    ROS_INFO("heave_PID_server Initiated");
}

heavePIDAction::~heavePIDAction(void)
{
}

void heavePIDAction::goalCB()
{
    goal_ = as_.acceptNewGoal()->target_heave;
    goalReceived = true;
}

void heavePIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void heavePIDAction::callback(const std_msgs::Float32ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
        return;

    current_heave_ = msg->data;

    if (goalReceived) {
        bool enable_pressure = false;
        nh_.getParam("/enable_pressure", enable_pressure);

        bool use_reference_depth = false;
        nh_.getParam("/use_reference_depth", use_reference_depth);

        double reference_depth = 0;
        nh_.getParam("/reference_depth", reference_depth);

        if (enable_pressure) {
            ROS_INFO("Pressure sensor enabled");
            if (use_reference_depth) {
                goal_ = goal_ + reference_depth;
                ROS_INFO("Reference Depth used");
            }
            else {
                goal_ = goal_ + current_heave_;
                ROS_INFO("Current Depth used");
            }
        }

        heave.setReference(goal_);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);

        goalReceived = false;
    }
    
    heave.errorToPWM(msg->data);

    feedback_.current_heave = msg->data;
    as_.publishFeedback(feedback_);

    nh_.setParam("/pwm_heave", heave.getPWM());
}

