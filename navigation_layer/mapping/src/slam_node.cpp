#include "slam_clsrv.h"
void observe_landmarks()
{
    while
}
namespace mapping{
int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_node"); 
    ros::NodeHandlePtr n(new ros::NodeHandle("~"));

    SlamNode node(n_);
    node.add_landmark();
    ros::spin();
}
}