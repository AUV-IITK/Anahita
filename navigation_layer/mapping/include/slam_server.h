#ifndef SLAM_SERVER_H
#define SLAM_SERVER_H

#include "slam_filter.h"
#include "ros/ros.h"
#include "mapping/slam_msg.h"
#include "mapping/slam_srv.h"
#include "util.h"

namespace mapping{
class SlamClient {
    public:
        int observe_landmark(const ros::NodeHandlePtr &n_, std::string obj_id, double m_x,double m_y, double m_z, double u_x, double u_y, double u_z, int uncertainty);
        vec6 request_landmark(const ros::NodeHandlePtr &n_, std::string obj_id);
        vec3 request_position(const ros::NodeHandlePtr &n_);
};

class SlamServer {
    private:
        SlamFilter *filter_;

    public:
        SlamServer(SlamFilter *filter);
        bool observe(mapping::slam_msg::Request &msg, mapping::slam_msg::Response &rep);
        bool slam_do(mapping::slam_srv::Request &req, mapping::slam_srv::Response &res);
        void Listen(const ros::NodeHandlePtr &n_);
        void Landmark_observe(const ros::NodeHandlePtr n_);
};
}
#endif