#include "slam_clsrv.h"
void observe_landmarks()
{
    while
}
namespace mapping{
int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_node"); 
    ros::NodeHandlePtr n(new ros::NodeHandle);
    SlamFilter* filter(new SlamFilter);
    ros::Subscriber sub=n->subscribe("anahita/pose_gt/relay",10,);
    

    SlamClient client(n);
    SlamServer server(n, filter);
    ros::Subscriber sub=n->subscribe();//yet to see
    
}
}