#ifndef BUOY_H
#define BUOY_H

#include <ros/ros.h>
#include <single_buoy.h>
#include <move_sideward_server.h>

class buoy {
public:
    buoy();
    ~buoy();
    void setActive(bool);

private:
    singleBuoy single_buoy_;
    moveSideward move_sideward_;
};
#endif // BUOY_H
