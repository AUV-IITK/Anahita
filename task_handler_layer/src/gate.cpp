#include <gate.h>

gateTask::gateTask(): move_straight_(100), move_forward_(150) {
    sub_ = nh_.subscribe("/gate_task/gate_coordinates", 1, &gateTask::distanceCB, this);
    spin_thread = new boost::thread(boost::bind(&gateTask::spinThread, this));
    move_forward_.setDataSource("SENSOR", "VISION");
}

gateTask::~gateTask() {

}

void gateTask::setActive(bool status) {
    if (status == true) {
        move_forward_.setActive(true);
        while (distance >= 60) {
            continue;
        }
        move_forward_.setActive(false);
        move_straight_.setActive(true);
        ros::Duration(7).sleep();
        move_straight_.setActive(false);
    }
    else {
        move_forward_.setActive(false);
        move_straight_.setActive(false);
    }
}

void gateTask::spinThread() {
    ros::spin();
}

void gateTask::distanceCB(const geometry_msgs::PointStamped::ConstPtr &_msg) {
    distance = _msg->point.x;
}