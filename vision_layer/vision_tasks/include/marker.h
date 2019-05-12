#ifndef MARKER_H
#define MARKER_H

#include "base_class.h"

class Marker : public Base_class
{
public:
    using Base_class::frontTaskHandling;
    Marker();
    ~Marker();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;
    // void frontTaskHandling(bool status);

private:
    image_transport::Publisher front_roi_pub;
};

#endif // MARKER_H
