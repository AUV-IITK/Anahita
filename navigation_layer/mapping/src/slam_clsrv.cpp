#include "slam_server.h"

namespace mapping{

//SLAM CLIENT DEFINTIONS
int SlamClient::observe_landmark(const ros::NodeHandlePtr &n_,std::string obj_id, double m_x, double m_y, double m_z, double u_x=0, 
            double u_y=0, double u_z=0, int uncertainty=0)
{
    mapping::slam_msg msg;
    msg.request.m_x=m_x;
    msg.request.m_y=m_y;
    msg.request.m_z=m_z;
    msg.request.id=obj_id;
    
    if(uncertainty==0)
    {
        msg.request.m_x=m_x;
        msg.request.m_y=m_y;
        msg.request.m_z=m_z;
    }
    else
    {
        msg.request.m_x=uncertainty;
        msg.request.m_y=uncertainty;
        msg.request.m_z=uncertainty;
    }
    ros::ServiceClient observe=n_->serviceClient<mapping::slam_msg>("observe_landmark"); //yet to see
    if(observe.call(msg))
    {
        if(msg.response.rep==1) return 1;
        else return 0;
    } 
    else 
        ROS_ERROR("REQUEST FAILED");
}

vec6 SlamClient::request_landmark(const ros::NodeHandlePtr &n_, std::string obj_id)
{
    ros::ServiceClient slclient=n_->serviceClient<mapping::slam_srv>("request_slam"); 
    mapping::slam_srv srv;
    srv.request.id=obj_id;
    if(slclient.call(srv))
    {
        vec6 result;
        result << (float)srv.response.m_x,(float)srv.response.m_y,(float)srv.response.m_z,
                       (float)srv.response.u_x,(float)srv.response.u_y,(float)srv.response.u_z;
        return result;
    }
    else
    {
        ROS_ERROR("REQUEST FAILED");
    }
}

vec3 SlamClient::request_position(const ros::NodeHandlePtr &n_)
{
    ros::ServiceClient slclient=n_->serviceClient<mapping::slam_srv>("request_slam"); 
    mapping::slam_srv srv;
    srv.request.id="";
    if(slclient.call(srv))
    {
        vec3 result;
        result << (float)srv.response.m_x,(float)srv.response.m_y,(float)srv.response.m_z;
        return result;
    }
    else
    {
        ROS_ERROR("REQUEST FAILED");
    }
}

//SLAM SERVER DEFINITIONS
SlamServer::SlamServer(SlamFilter *filter)
    : filter_{filter} {

}
bool SlamServer::slam_do(mapping::slam_srv::Request &req, mapping::slam_srv::Response &res)
{

    if (req.id == "") {
        vec3 pos {filter_->GetState()};
        res.m_x=pos(0,0);
        res.m_y=pos(1,0);
        res.m_z=pos(2,0);
        }
    else {
        vec6 state {filter_->GetState(req.id)};
        res.m_x=state(0,0);
        res.m_y=state(1,0);
        res.m_z=state(2,0);
        res.u_x=state(3,0);
        res.u_y=state(4,0);
        res.u_z=state(5,0);
        }
    return true;
    }

void SlamServer::Listen(const ros::NodeHandlePtr &n_) {
    while (true) 
    {
        ros::ServiceServer slsrv=n_->advertiseService("request_slam",&SlamServer::slam_do,this);
    }
    std::cout << "Finishing Up..." << std::endl;
}

bool SlamServer::observe(mapping::slam_msg::Request &msg, mapping::slam_msg::Response &rep)
{
    std::string id {msg.id.c_str()};
    vec3 relpos {float(msg.m_x), float(msg.m_y), float(msg.m_z)};
    mat3 cov {vec3(float(msg.u_x), float(msg.u_y), float(msg.u_z)).asDiagonal()};
    filter_->Landmark(id,relpos, cov);
    rep.rep=1;
    return true;
}
void SlamServer::Landmark_observe(const ros::NodeHandlePtr n_)
{
    ros::ServiceServer sub=n_->advertiseService("observe_landmark",&SlamServer::observe,this);
}

}