#include <ros/ros.h>
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
#include <task_handler.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1000);
    taskHandler th(15); // time out 15 seconds

    std_msgs::String current_task;
    current_task.data = "red_buoy";
    task_pub.publish(current_task);
    nh.setParam("/current_task", "red_buoy");

    singleBuoy single_buoy;
    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////
    
    current_task.data = "yellow_buoy";
    task_pub.publish(current_task);
    nh.setParam("/current_task", "yellow_buoy");

    moveSideward move_sideward(-100);
    move_sideward.setActive(true); // until the yellow buoy is detected (for vision node)
    
    // ros::Duration(5).sleep(); // duration may vary depending upon the time observed in testing
    
    th.isDetected("yellow_buoy", 15); // time out of 15 seconds
    move_sideward.setActive(false);

    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////

    current_task.data = "green_buoy";
    task_pub.publish(current_task);
    nh.setParam("/current_task", "green_buoy");

    move_sideward.setThrust(100);
    move_sideward.setActive(true); // until the green buoy is detected (for vision node)
    
    // ros::Duration(10).sleep(); // should move approx. 10m straight sideways
    
    th.isDetected("green_buoy", 15); // time out of 15 seconds
    move_sideward.setActive(false);

    single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////

    current_task.data = "buoy-gate";
    task_pub.publish(current_task);
    nh.setParam("/current_task", "buoy-gate");

    actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");
    motion_layer::upwardPIDGoal upwardPIDgoal;

    ROS_INFO("Waiting for upwardPID server to start, Buoy-Gate transition.");
    upwardPIDClient.waitForServer();

    ROS_INFO("upwardPID server started, sending goal, Buoy-Gate transition."); // task handler node
    upwardPIDgoal.target_depth = 40; // some number based on the data getting from the pressure sensor
    upwardPIDClient.sendGoal(upwardPIDgoal); 

    // to get vertically above the buoy
    // NEED to have a node or class to notify when something's done

    th.isAchieved(40, 10, "upward"); // target is 40, band is 10 and task is upward

    // when upward target is achieved 

    moveStraight move_straight(100); // for vision node
    move_straight.setActive(true); // until the green buoy is detected under the bottom camera

    th.isDetected("green_buoy", 10);

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
    forwardPIDClient.sendGoal(forwardPIDgoal); // task_handler node

    th.isAchieved(0, 10, "forward"); // blocks until forward task is done

    // After the alignment

    ////////////////////////////////////////////////

    current_task.data = "gate";
    task_pub.publish(current_task);
    nh.setParam("/current_task", "gate");

    move_sideward.setThrust(-100);
    move_sideward.setActive(true); // until gate is detected
    
    // ros::Duration(5).sleep(); // time depends, better to have a node telling when the gate is detected

    th.isDetected("gate", 10);

    gateTask gate_task;
    gate_task.setActive(true); // blocking function, will terminalte after completion

    return 0;
}
