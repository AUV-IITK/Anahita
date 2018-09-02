#include <upward_PID_server.h>

upwardPIDAction::upwardPIDAction(std::string name, std::string type_) :
    as_(nh_, name, false),
    action_name_(name), z_coord("Z_COORD")
{
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&upwardPIDAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&upwardPIDAction::preemptCB, this));
    goal_ = 0;

    type = type_;

    //subscribe to the data topic of interest
    if (type == "VISION") {
        sub_ = nh_.subscribe("/buoy_task/buoy_coordinates", 1, &upwardPIDAction::visionCB, this);
        z_coord.setPID(7.5, 0, 2, 10);
    }
    else if (type == "SENSOR") {
        sub_ = nh_.subscribe("/vision/sensors/depth", 1, &upwardPIDAction::sensorCB, this);
        z_coord.setPID(2.4, 0, 0.5, 1);
    }

    as_.start();
}

upwardPIDAction::~upwardPIDAction(void)
{
}

void upwardPIDAction::goalCB()
{
    std::cout << "hey, what's up" << std::endl;
    goal_ = as_.acceptNewGoal()->target_depth;
    z_coord.setReference(goal_);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, got a target of %f", action_name_.c_str(), goal_);
}

void upwardPIDAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void upwardPIDAction::sensorCB(const std_msgs::Float32ConstPtr& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
        return;
    
    z_coord.errorToPWM(msg->data);

    feedback_.current_depth = msg->data;
    as_.publishFeedback(feedback_);

    if (msg->data == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }

    nh_.setParam("/pwm_upward_front", z_coord.getPWM());
    nh_.setParam("/pwm_upward_back", z_coord.getPWM());
}

void upwardPIDAction::visionCB(const geometry_msgs::PointStampedConstPtr &msg) {
    if (!as_.isActive())
        return;
    
    z_coord.errorToPWM(msg->point.z);

    feedback_.current_depth = msg->point.z;
    as_.publishFeedback(feedback_);

    if (msg->point.z == goal_) {
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }

    nh_.setParam("/pwm_upward_front", z_coord.getPWM());
    nh_.setParam("/pwm_upward_back", z_coord.getPWM());
}

void upwardPIDAction::setDataSource(std::string type_) {
    type = type_;
}

