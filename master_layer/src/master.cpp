#include <ros/ros.h>
#include <buoy.h>
#include <line.h>
#include <gate.h>
#include <single_buoy.h>
#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_forward_server.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1000);

    // can make a node which can send a signal when an object is detected given the current task
    // can make a task handler class which can take care of when a task is completed or when an object is detected

    std_msgs::String current_task;
    current_task.data = "red_buoy";
    task_pub.publish(current_task);

    singleBuoy single_buoy;
    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////
    
    current_task.data = "yellow_buoy";
    task_pub.publish(current_task);

    moveSideward move_sideward(-100);
    move_sideward.setActive(true); // until the yellow buoy is detected
    ros::Duration(5).sleep(); // duration may vary depending upon the time observed in testing
    move_sideward.setActive(false);

    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////

    current_task.data = "green_buoy";
    task_pub.publish(current_task);

    move_sideward.setThrust(100);
    move_sideward.setActive(true); // until the green buoy is detected
    ros::Duration(10).sleep(); // should move approx. 10m straight sideways
    move_sideward.setActive(false);

    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////

    current_task.data = "buoy-gate";
    task_pub.publish(current_task);

    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");
    motion_layer::upwardPIDGoal upwardPIDgoal;

    ROS_INFO("Waiting for upwardPID server to start, Buoy-Gate transition.");
    upwardPIDClient.waitForServer();

    ROS_INFO("upwardPID server started, sending goal, Buoy-Gate transition.");
    upwardPIDgoal.target_depth = 40; // some number based on the data getting from the pressure sensor
    upwardPIDClient.sendGoal(upwardPIDgoal); 

    // to get vertically above the buoy

    // NEED to have a node or class to notify when something's done

    // when upward target is achieved 

    moveStraight move_straight(100);
    move_straight.setActive(true); // until the green buoy is detected under the bottom camera

    // Align to its center

    actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");
    motion_layer::sidewardPIDGoal sidewardPIDgoal;

    ROS_INFO("Waiting for sidewardPID server to start, Buoy-Gate transition.");
    sidewardPIDClient.waitForServer();

    ROS_INFO("sidewardPID server started, sending goal, Buoy-Gate transition.");
    sidewardPIDgoal.target_distance = 0;
    sidewardPIDClient.sendGoal(sidewardPIDgoal);

    actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient("forwardPID");
    motion_layer::forwardPIDGoal forwardPIDgoal;

    ROS_INFO("Waiting for forwardPID server to start, Buoy-Gate transition.");
    forwardPIDClient.waitForServer();

    ROS_INFO("forwardPID server started, sending goal, Buoy-Gate transition.");
    forwardPIDgoal.target_distance = 0;
    forwardPIDClient.sendGoal(forwardPIDgoal);

    // After the alignment

    move_sideward.setThrust(-100);
    move_sideward.setActive(true); // until gate is detected
    ros::Duration(5).sleep(); // time depends, better to have a node telling when the gate is detected

    ////////////////////////////////////////////////

    current_task.data = "gate";
    task_pub.publish(current_task);

    gateTask gate_task;
    gate_task.setActive(true); // blocking function, will terminalte after completion

    return 0;
}
