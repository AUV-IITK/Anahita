#ifndef SLAM_SERVER_H
#define SLAM_SERVER_H

#include "slam_filter.h"
#include "ros/ros.h"
#include "mapping/slam_msg.h"
#include "mapping/slam_srv.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "util.h"

namespace mapping{
class SlamClient {
    private:
        ros::NodeHandlePtr n_;
    public:
        SlamClient(const ros::NodeHandlePtr &n_);
        int observe_landmark(std::string obj_id, double m_x,double m_y, double m_z, double u_x, double u_y, double u_z, int uncertainty);
        vec6 request_landmark(std::string obj_id);
        vec3 request_position();
};

class SlamServer { 
    private:
        ros::NodeHandlePtr n_;
        SlamFilter *filter_;

    public:
        SlamServer(const ros::NodeHandlePtr &n_, SlamFilter *filter);
        bool observe(mapping::slam_msg::Request &msg, mapping::slam_msg::Response &rep);
        bool slam_do(mapping::slam_srv::Request &req, mapping::slam_srv::Response &res);
        void Listen();
        void Landmark_observe();
};

class SlamNode
{
    private:
        ros::NodeHandlePtr n_;
        ros::Subscriber sub;
        SlamClient client;
        SlamServer server;
    public:
        SlamNode(const ros::NodeHandlePtr &n_);
        void Observe(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void add_landmark();
        
};
}
#endif