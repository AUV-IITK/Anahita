#include <depth_stabilise.h>

depthStabilise::depthStabilise () : heavePIDClient("heavePID") {}

depthStabilise::~depthStabilise () {}

void depthStabilise::activate (std::string type) {
    nh.setParam("/enable_pressure", true);
    if (type == "reference") {
        nh.setParam("/use_reference_depth", true);
    }
    spin_thread = new boost::thread(boost::bind(&depthStabilise::spinThread, this));
}

void depthStabilise::deActivate () {
    nh.setParam("/enable_pressure", false);
    nh.setParam("/use_reference_depth", false);

    heavePIDClient.cancelGoal();

    spin_thread->join();
    nh.setParam("/kill_signal", true);
}

void depthStabilise::spinThread() {

    ROS_INFO("Waiting for heavePID server to start, depth stabilise");
    heavePIDClient.waitForServer();

    ROS_INFO("heavePID server started, sending goal.");
    heave_PID_goal.target_heave = 0;
    heavePIDClient.sendGoal(heave_PID_goal);        
}
