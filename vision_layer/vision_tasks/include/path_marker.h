#ifndef PATH_MARKER_H
#define PATH_MARKER_H

#include "base_class.h"
#include <master_layer/RequestMarkerAngle.h>

class PathMarker : public Base_class
{
public:
    PathMarker();
    virtual void loadParams () override;
    virtual void spinThreadBottom () override;
    bool markerAngle (master_layer::RequestMarkerAngle::Request &req,
                       master_layer::RequestMarkerAngle::Response &res); 

private:
    double MAJOR;
    double MINOR;
    ros::ServiceServer service;
};

#endif // PATH_MARKER_H
