#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include <task_handler.h>
#include <navigation_handler.h>

#define FORWARD 1
#define BACKWARD -1

using namespace std;

void spinThread() {
    ROS_INFO("Spinning");
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::Time::init();

    taskHandler th(30); // time out 15 seconds

    navigationHandler nav_handle;

    boost::thread spin_thread(&spinThread);
    
    /////////////////////////////////////////////

    // Random code to test

    /////////////////////////////////////////////

    spin_thread.join();

    return 0;
}
