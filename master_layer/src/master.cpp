#include <ros/ros.h>
#include <gate.h>
#include <single_buoy.h>
#include <straight_server.h>
#include <move_sideward_server.h>
#include <move_forward_server.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_layer/anglePIDAction.h>
#include <motion_layer/rollPIDAction.h>
#include <motion_layer/pitchPIDAction.h>
#include <motion_layer/forwardPIDAction.h>
#include <motion_layer/sidewardPIDAction.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/String.h>
#include <task_handler.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "master");
    ros::NodeHandle nh;
    ros::Publisher task_pub = nh.advertise<std_msgs::String>("/current_task", 1); 
    ros::Time::init();
    int pub_count = 0;
    std_msgs::String current_task;

    // taskHandler th(5); // time out 15 seconds

    // moveStraight object(50);
    // object.setActive(true);

    // TO MAINTAIN UPWARD
    // actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");
    // motion_layer::upwardPIDGoal upwardPIDgoal;
    // ROS_INFO("Waiting for upwardPID server to start, constant height");
    // upwardPIDClient.waitForServer();
    // ROS_INFO("upwardPID server started, sending goal, constant height"); // task handler node

    // upwardPIDgoal.target_depth = 1020; // some number based on the data getting from the pressure sensor
    // upwardPIDClient.sendGoal(upwardPIDgoal);
    // th.isAchieved(980, 2, "upward"); // target is 40, band is 10 and task is upward
    
    // ros::Duration(10).sleep();

    // upwardPIDgoal.target_depth = 980; // some number based on the data getting from the pressure sensor
    // upwardPIDClient.sendGoal(upwardPIDgoal); 
    // th.isAchieved(1020, 2, "upward"); // target is 40, band is 10 and task is upward
    
    // ros::Duration(10).sleep();
    
    // upwardPIDgoal.target_depth = 1020; // some number based on the data getting from the pressure sensor
    // upwardPIDClient.sendGoal(upwardPIDgoal); 
    // th.isAchieved(102, 2, "upward"); // target is 40, band is 10 and task is upward
    
    // ros::Duration(10).sleep();

    // object.setActive(false);
   
    // TO MAINTAIN PITCH
    // actionlib::SimpleActionClient<motion_layer::pitchPIDAction> pitchPIDClient("pitchPID");
    // motion_layer::pitchPIDGoal pitchPIDgoal;
    // ROS_INFO("Waiting for pitchPID server to start, constant pitch");
    // pitchPIDClient.waitForServer();
    // ROS_INFO("upwardPID server started, sending goal, constant pitch"); // task handler node
    // pitchPIDgoal.target_pitch = 14.33;
    // pitchPIDClient.sendGoal(pitchPIDgoal); 
    // th.isAchieved(14.33, 1.5, "pitch"); // target is 40, band is 10 and task is upward
 
    // TO MAINTAIN ROLL
    // actionlib::SimpleActionClient<motion_layer::rollPIDAction> rollPIDClient("rollPID");
    // motion_layer::rollPIDGoal rollPIDgoal;
    // ROS_INFO("Waiting for rollPID server to start, constant roll");
    // rollPIDClient.waitForServer();
    // ROS_INFO("rollPID server started, sending goal, constant roll"); // task handler node
    // rollPIDgoal.target_roll = 15.28; // some number based on the data getting from the pressure sensor
    // rollPIDClient.sendGoal(rollPIDgoal); 
    // th.isAchieved(15.28, 1.5, "roll"); // target is 40, band is 10 and task is upward

    // ros::Duration(500).sleep();
    
    ros::Rate loop_rate(10);
    while (ros::ok() && pub_count <= 5) {
        current_task.data = "red_buoy";
        task_pub.publish(current_task);
        pub_count++;
        loop_rate.sleep();
    }
    pub_count = 0;
    nh.setParam("/current_task", "red_buoy");
    ROS_INFO("Current task: Red Buoy");
    
    // ROS_INFO("Starting buoy task");
    // singleBuoy single_buoy;
    // single_buoy.setActive(true); // blocking function, will terminalte after completion 
    
    // ROS_INFO("Completed the first buoy task, Now let's move on to the second");
    // nh.setParam("/kill_signal", true);
    // ros::Duration(4).sleep();
    // nh.setParam("/kill_signal", false);

    ////////////////////////////////////////////////

    // moveSideward move_sideward1(50);
    // move_sideward1.setActive(true);
    // ros::Duration(4).sleep();
    // move_sideward1.setActive(false);
    // ROS_INFO("Sideawrd moving completed");
    // nh.setParam("/kill_signal", true);
    
    // ros::Duration(1).sleep();
    // nh.setParam("/kill_signal", false);    
    
    // ros::Duration(1).sleep();
    // ROS_INFO("started buoy task for yellow");
    // current_task.data = "yellow_buoy";
    // task_pub.publish(current_task);
    // nh.setParam("/current_task", "red_buoy");
    // ROS_INFO("Currently publishing the colour");
    // ros::Duration(5).sleep();
    // single_buoy.setActive(true);
  
    // nh.setParam("/kill_signal", true);
    
    // moveSideward move_sideward(100);
    // move_sideward.setActive(true); // until the yellow buoy is detected (for vision node)

    // ros::Duration(5).sleep(); // duration may vary depending upon the time observed in testing
    
    // move_sideward.setActive(false);

    // ROS_INFO("Move sideward completed");
    // singleBuoy single_buoy;
    // single_buoy.setActive(true);

    // ROS_INFO("Done buoy task");

    // th.isDetected("yellow_buoy", 15); // time out of 15 seconds
    // move_sideward.setActive(false);

    // single_buoy.setActive(true); // blocking function, will terminalte after completion

    ////////////////////////////////////////////////

    // current_task.data = "green_buoy";
    // task_pub.publish(current_task);
    // nh.setParam("/current_task", "green_buoy");

    // move_sideward.setThrust(100);
    // move_sideward.setActive(true); // until the green buoy is detected (for vision node)
    
    // ros::Duration(10).sleep(); // should move approx. 10m straight sideways
    
    // th.isDetected("green_buoy", 15); // time out of 15 seconds
    // move_sideward.setActive(false);

    // single_buoy.setActive(true); // blocking function, will terminalte after completion
  
    // moveStraight object(50);
    // object.setActive(true);
    // ros::Duration(20).sleep();
    // object.setActive(false);

    // ROS_INFO("Move forward completed");

    ////////////////////////////////////////////////

    // current_task.data = "buoy-gate";
    // task_pub.publish(current_task);
    // nh.setParam("/current_task", "buoy-gate");

    // Example for upward PID

    // actionlib::SimpleActionClient<motion_layer::upwardPIDAction> upwardPIDClient("upwardPID");
    // motion_layer::upwardPIDGoal upwardPIDgoal;

    // ROS_INFO("Waiting for upwardPID server to start, Buoy-Gate transition.");
    // upwardPIDClient.waitForServer();

    // ROS_INFO("upwardPID server started, sending goal, Buoy-Gate transition."); // task handler node
    // upwardPIDgoal.target_depth = 40; // some number based on the data getting from the pressure sensor
    // upwardPIDClient.sendGoal(upwardPIDgoal); 

    // to get vertically above the buoy
    // NEED to have a node or class to notify when something's done

    // th.isAchieved(40, 10, "upward"); // target is 40, band is 10 and task is upward

    // when upward target is achieved/
    // moveStraight moveStraight(50); // for vision node
    // moveStraight.setActive(true); // until the green buoy is detected under the bottom camera

    // th.isDetected("green_buoy", 10);

    //ros::Duration(10).sleep();
    //move_straight.setActive(false);
    // Align to its center
    // Example for sideward, forward, and angle PIDs

    // actionlib::SimpleActionClient<motion_layer::sidewardPIDAction> sidewardPIDClient("sidewardPID");
    // motion_layer::sidewardPIDGoal sidewardPIDgoal;

    // ROS_INFO("Waiting for sidewardPID server to start, Buoy-Gate transition.");
    // sidewardPIDClient.waitForServer();

    // ROS_INFO("sidewardPID server started, sending goal, Buoy-Gate transition.");
    // sidewardPIDgoal.target_distance = 0;
    // sidewardPIDClient.sendGoal(sidewardPIDgoal);

    // actionlib::SimpleActionClient<motion_layer::forwardPIDAction> forwardPIDClient("forwardPID");
    // motion_layer::forwardPIDGoal forwardPIDgoal;

    // ROS_INFO("Waiting for forwardPID server to start, Buoy-Gate transition.");
    // forwardPIDClient.waitForServer();

    // ROS_INFO("forwardPID server started, sending goal, Buoy-Gate transition.");
    // forwardPIDgoal.target_distance = 0;
    // forwardPIDClient.sendGoal(forwardPIDgoal); // task_handler node

    // actionlib::SimpleActionClient<motion_layer::anglePIDAction> anglePIDClient("turnPID");
    // motion_layer::anglePIDGoal anglePIDgoal;

    // ROS_INFO("Waiting for anglePID server to start.");
    // anglePIDClient.waitForServer();

    // ROS_INFO("anglePID server started, sending goal.");

    // anglePIDGoal.target_angle = angle_;
    // anglePIDClient.sendGoal(anglePIDGoal);

    // th.isAchieved(0, 10, "forward"); // blocks until forward task is done

    // After the alignment

    ////////////////////////////////////////////////

    // current_task.data = "gate";
    // task_pub.publish(current_task);
    // nh.setParam("/current_task", "gate");

    // move_sideward.setThrust(-100);
    // move_sideward.setActive(true); // until gate is detected
    
    // ros::Duration(5).sleep(); // time depends, better to have a node telling when the gate is detected

    // th.isDetected("gate", 10);
    
    // gateTask gate_task;
    // gate_task.setActive(true); // blocking function, will terminalte after completion
    // nh.setParam("/kill_signal", true);

    return 0;
}
