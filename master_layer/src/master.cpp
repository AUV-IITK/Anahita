#include <ros/ros.h>

#include <gate.h>
#include <single_buoy.h>
#include <torpedo.h>
#include <marker_dropper.h>
#include <octagon.h>
#include <line.h>

#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_forward_server.h>
#include <move_downward_server.h>

#include <actionlib/client/simple_action_client.h>

#include <motion_layer/anglePIDAction.h>
#include <motion_layer/rollPIDAction.h>
#include <motion_layer/pitchPIDAction.h>
#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <motion_layer/upwardPIDAction.h>

#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>
#include <task_handler.h>
#include <boost/thread.hpp>

using namespace std;

void spinThread() {
    ROS_INFO("Spinning");
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1); 
    ros::Time::init();
    int pub_count = 0;
    std_msgs::String current_task;
    ros::Rate loop_rate(10);
    taskHandler th(15); // time out 15 seconds
    singleBuoy single_buoy;

    boost::thread spin_thread(&spinThread);

    ////////////////////////////////////////////

    //nh.setParam("/enable_pressure", 1);
    /*nh.setParam("/disable_imu", true);

     current_task.data = "line";
     while (ros::ok() && pub_count <= 5) {
         task_pub.publish(current_task);
         pub_count++;
         loop_rate.sleep();
     }
     pub_count = 0;
     nh.setParam("/current_task", "line");
     ROS_INFO("Current task: Line");

     bool temp = th.isDetected("line", 10);

     if (!temp) {
         ROS_INFO("Line not detected before the timeout");
         return 1;
     }

     ROS_INFO("Line Detected");

     lineTask task;

     temp = task.setActive(true);

     if (!temp) {
         ROS_INFO("Line Task Failed");
         return 1;
     }
     task.setActive(false);

     ROS_INFO("Completed the Line task");

     //nh.setParam("/enable_pressure", 1);
     nh.setParam("/disable_imu", false);*/
   
    /////////////////////////////////////////////

    ros::Duration(60).sleep();

    current_task.data = "red_buoy";
    while (ros::ok() && pub_count <= 5) {
        task_pub.publish(current_task);
        pub_count++;
        loop_rate.sleep();
    }
    pub_count = 0;
    nh.setParam("/current_task", "red_buoy");
    ROS_INFO("Current task: Red Buoy");
    
    single_buoy.setActive(true); // blocking function, will terminalte after completion 
    single_buoy.setActive(false);
    
    ROS_INFO("Completed the first buoy task, Now let's move on to the second");

    ///////////////////////////////////////////////

    /*current_task.data = "yellow_buoy";
    while (ros::ok() && pub_count <= 5) {
        task_pub.publish(current_task);
        pub_count++;
        loop_rate.sleep();
    }
    pub_count = 0;
    nh.setParam("/current_task", "yellow_buoy");
    ROS_INFO("Current task: Yellow Buoy");*/

    /*moveSideward move_sideward(-50);
    move_sideward.setActive(true);
    ros::Duration(10).sleep();
    //ROS_INFO("Finding Yellow Buoy....");
    //th.isDetected("yellow_buoy", 15); // time out of 15 seconds
    //ROS_INFO("Yellow Buoy Detected");
    move_sideward.setActive(false);

    /*single_buoy.setActive(true);
    single_buoy.setActive(false);

    ROS_INFO("Yellow Buoy done");
    
    //////////////////////////////////////////////

    current_task.data = "green_buoy";
    while (ros::ok() && pub_count <= 5) {
        task_pub.publish(current_task);
        pub_count++;
        loop_rate.sleep();
    }
    pub_count = 0;
    nh.setParam("/current_task", "green_buoy");
    ROS_INFO("Current task: Green Buoy");

    moveStraight move_straight(-50);
    move_straight.setActive(true);
    ros::Duration(3).sleep(); // configurable 
    move_straight.setActive(false);

    ROS_INFO("Finding Green Buoy...");
    move_sideward.setThrust(50);
    mo	ve_sideward.setActive(true); // until the green buoy is detected (for vision node)
    ros::Duration(5).sleep(); // configurable

    th.isDetected("green_buoy", 15); // time out of 15 seconds
    ROS_INFO("Green Buoy Detected");
            
    move_sideward.setActive(false);

    single_buoy.setActive(true); // blocking function, will terminalte after completion
    single_buoy.setActive(false);

    ROS_INFO("Green Buoy Done");*/
  
    ////////////////////////////////////////////////

    // current_task.data = "gate";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "gate");
    // ROS_INFO("Current task: Gate");

    // ROS_INFO("Gate Task, going down");
    // move_downward.setActive(true);
    // ros::Duration(4).sleep();
    // move_downward.setActive(false);
    // ROS_INFO("Gate Task, at the bottom");

    // move_sideward.setThrust(-100);
    // move_sideward.setActive(true); // until gate is detected
    
    // // ros::Duration(5).sleep(); // time depends, better to have a node telling when the gate is detected

    // th.isDetected("gate", 10);

    // move_sideward.setActive(false);

    // gateTask gate_task;
    // gate_task.setActive(true); // blocking function, will terminalte after completion
    // gate_task.setActive(false);

    // ///////////////////////////////////////////////////

    // // Gate-Torpedo Transition

    // current_task.data = "red_torpedo";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "red_torpedo");
    // ROS_INFO("Current task: Red Torpedo");

    // actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient("turnPID");
    // motion_layer::anglePIDGoal anglePIDGoal;

    // ROS_INFO("Waiting for anglePID server to start.");
    // anglePIDClient.waitForServer();

    // ROS_INFO("anglePID server started, sending goal.");

    // anglePIDGoal.target_angle = 60;
    // anglePIDClient.sendGoal(anglePIDGoal);

    // th.isAchieved(60, 2, "angle");

    // anglePIDClient.cancelGoal();

    // th.isDetected("red_torpedo", 5);

    // ///////////////////////////////////////////////////

    // Torpedo torpedo;

    // torpedo.setActive(true);
    // torpedo.setActive(false);

    // ////////////////////////////////////////////////////

    // current_task.data = "green_torpedo";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "green_torpedo");
    // ROS_INFO("Current task: Green Torpedo");

    // move_sideward.setThrust(100);
    // move_sideward.setActive(true);

    // th.isDetected("green_torpedo", 5);

    // move_sideward.setActive(false);

    // torpedo.setActive(true);
    // torpedo.setActive(false);

    // /////////////////////////////////////////////////////

    // // Torpedo-MarkerDropper Transition
    
    // // After finishing torpedo task turn 120 degree anticlockwise and then move straight
    // ROS_INFO("Master layer, anglePID server started, sending goal."); 

    // anglePIDGoal.target_angle = -120;
    // anglePIDClient.sendGoal(anglePIDGoal);

    // th.isAchieved(-120, 2, "angle"); // wait till the bot has rotated 120 degrees

    // anglePIDClient.cancelGoal();

    // moveStraight move_straight(100);
    // move_straight.setActive(true);
    // ros::Duration(6).sleep();
    // move_straight.setActive(false);

    // /////////////////////////////////////////////////////

    // // MarkerDropper
    // current_task.data = "marker_dropper_front";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "marker_dropper_front");
    // ROS_INFO("Current task: Marker Dropper Front");

    // th.isDetected("marker_dropper_front", 5);

    // actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");
    // motion_layer::sidewardPIDGoal sidewardPIDGoal;

    // ROS_INFO("Waiting for anglePID server to start.");
    // sidewardPIDClient.waitForServer();

    // ROS_INFO("anglePID server started, sending goal.");

    // sidewardPIDGoal.target_distance = 0;
    // sidewardPIDClient.sendGoal(sidewardPIDGoal);

    // move_straight.setActive(true);

    // // Until the bin is visible in the front camera

    // sidewardPIDClient.cancelGoal(); // dangerous

    // current_task.data = "marker_dropper_bottom";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "marker_dropper_bottom");
    // ROS_INFO("Current task: Marker Dropper Bottom");

    // move_straight.setThrust(50);

    // th.isDetected("marker_dropper_bottom", 6);

    // move_straight.setActive(false);

    // MarkerDropper md;
    // md.setActive(true);
    // md.setActive(false);

    // /////////////////////////////////////////////////////

    // // Octagon

    // ROS_INFO("Waiting for anglePID server to start.");
    // anglePIDClient.waitForServer();

    // ROS_INFO("anglePID server started, sending goal.");

    // anglePIDGoal.target_angle = 60;
    // anglePIDClient.sendGoal(anglePIDGoal);

    // th.isAchieved(60, 2, "angle");

    // anglePIDClient.cancelGoal();

    // current_task.data = "octagon";
    // while (ros::ok() && pub_count <= 5) {
    //     task_pub.publish(current_task);
    //     pub_count++;
    //     loop_rate.sleep();
    // }
    // pub_count = 0;
    // nh.setParam("/current_task", "octagon");
    // ROS_INFO("Current task: Octagon");

    // move_straight.setActive(true);

    // th.isDetected("octagon", 10);

    // move_straight.setActive(false);

    // Octagon octagon;
    // octagon.setActive(true);
    // octagon.setActive(false);
    
    /////////////////////////////////////////////////////
   
    nh.setParam("/kill_signal", true);

    spin_thread.join();

    return 0;
}
