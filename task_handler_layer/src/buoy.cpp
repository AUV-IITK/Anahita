#include <buoy.h>

buoy::buoy(): move_sideward_(100), single_buoy_() {}
buoy::~buoy() {}

void buoy::setActive(bool status) {
    if (status == true) {
        single_buoy_.setActive(true);
        move_sideward_.setActive(true);
        ros::Duration(10).sleep();
        move_sideward_.setActive(false);
        single_buoy_.setActive(true);
        move_sideward_.setActive(true);
        ros::Duration(10).sleep();
    }

    else if (status == false) {
        single_buoy_.setActive(false);
        move_sideward_.setActive(false);
    }
}