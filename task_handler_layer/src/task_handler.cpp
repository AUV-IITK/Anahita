#include <task_handler.h>

taskHandler::taskHandler (double _timeout) {
    topic_map_["upward"] = "/anahita/z_coordinate";
    topic_map_["sideward"] = "/anahita/y_coordinate";
    topic_map_["forward"] = "/anahita/x_coordinate";
    topic_map_["angle"] = "/mavros/imu/yaw";
    topic_map_["pitch"] = "/mavros/imu/pitch";
    topic_map_["roll"] = "/mavros/imu/roll";

    task_map_["red_buoy"] = false;
    task_map_["green_buoy"] = false;
    task_map_["yellow_buoy"] = false;
    task_map_["buoy-gate"] = false;
    task_map_["gate"] = false;

    time_out_ =  _timeout;

    ros::Time::init();

    spin_thread_ = new boost::thread(boost::bind(&taskHandler::spinThread, this));
    vision_sub_ = nh_.subscribe("/detected", 1, &taskHandler::visionCB, this);
}

taskHandler::~taskHandler () {}

void taskHandler::callBack (const std_msgs::Float32Ptr &_msg) {
    data_ = _msg->data;
}

void taskHandler::spinThread () {
    // ros::spin();
}

bool taskHandler::isAchieved (double _target, double _band, std::string _topic) {

    double beginning = ros::Time::now().toSec();

    if (is_subscribed_) {
        sub_.shutdown();
    }
    sub_ = nh_.subscribe(topic_map_[_topic], 1, &taskHandler::callBack, this);
    is_subscribed_ = true;

    int count = 0;
    double then = ros::Time::now().toSec();

    while (ros::ok()) {
        if (std::abs(data_ - _target) <= _band) {
            if (!count) {
                then = ros::Time::now().toSec();
            }
            count++;
        }
        double now = ros::Time::now().toSec();
        double diff = now - then;
        double total_time = now - beginning;

        if (total_time >= time_out_) {
            return false;
        }
        if (diff >= 1.0) {
            if (count >= 10) {
                return true;
            }
            else {
                count = 0;
            }
        }
    }
}

bool taskHandler::isDetected (std::string _task, double _timeout) {
    vision_time_out_ = _timeout;
    double then = ros::Time::now().toSec();
    double now;
    double diff;
    while (!task_map_[_task] && ros::ok()) {
        now = ros::Time::now().toSec();
        diff = now - then;
        if (diff > vision_time_out_) {
            return false;
        }
    }
    task_map_[_task] = false;
}

void taskHandler::visionCB (const std_msgs::BoolPtr& _msg) {
    std::string current_task;
    nh_.getParam("/current_task", current_task);

    if (_msg->data) {
        if (current_task == "green_buoy") {
            task_map_["green_buoy"] = true;
        }
        else if (current_task == "yellow_buoy") {
            task_map_["yellow_buoy"] = true;
        }
        else if (current_task == "red_buoy") {
            task_map_["red_buoy"] = true;
        }
        else if (current_task == "gate") {
            task_map_["gate"] = true;
        }
        else if (current_task == "buoy-gate") {
            task_map_["buoy-gate"] = true;
        }
    } 
}

void taskHandler::setTimeout (double _time) {
    time_out_ = _time;
}

void taskHandler::setVisionTimeout (double _time) {
    vision_time_out_ = _time;
}
