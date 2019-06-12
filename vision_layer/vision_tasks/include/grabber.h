#ifndef GRABBER_H
#define GRABBER_H

#include "base_class.h"

class Grabber: public Base_class
{
public:
    Grabber();
    ~Grabber();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;

private:
    image_transport::Publisher front_roi_pub;
};

#endif // GRABBER_H
