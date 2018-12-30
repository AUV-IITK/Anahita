#include <line.h>

lineTask::lineTask(): sidewardPIDClient("sidewardPID"), anglePIDClient("turnPID"), 
                    upwardPIDClient("upwardPID"), th(15) {
    // spin_thread = new boost::thread(boost::bind(&lineTask::spinThread, this));

    sub_ = nh_.subscribe("/mavros/imu/data", 1, &lineTask::angleCB, this);
}

lineTask::~lineTask() {

}

void lineTask::setActive(bool value) {
    if (value) {
        ROS_INFO("Waiting for sidewardPID server to start.");
        sidewardPIDClient.waitForServer();

        ROS_INFO("sidewardPID server started, sending goal.");
        sideward_PID_goal.target_distance = 0;
        sidewardPIDClient.sendGoal(sideward_PID_goal);

        ROS_INFO("Waiting for upwardPID server to start.");
        upwardPIDClient.waitForServer();

        ROS_INFO("upwardPID server started, sending goal.");
        upward_PID_goal.target_depth = 0;
        upwardPIDClient.sendGoal(upward_PID_goal);

        th.isAchieved(0, 5, "upward");

        ROS_INFO("Waiting for anglePID server to start.");
        anglePIDClient.waitForServer();

        while (ros::ok() && !angleReceived) { continue; }

        ROS_INFO("anglePID server started, sending goal.");
        angle_PID_goal.target_angle = angle_;
        anglePIDClient.sendGoal(angle_PID_goal);

        th.isAchieved(0, 5, "angle");

    }
    else {
        anglePIDClient.cancelGoal();
        sidewardPIDClient.cancelGoal();
        upwardPIDClient.cancelGoal();
        // spin_thread->join();
    }
}

// void lineTask::spinThread() {
//     ros::spin();
// }

void lineTask::angleCB(const std_msgs::Float32ConstPtr& _msg) {
    angle_ = _msg->data;
    angleReceived = true;
}