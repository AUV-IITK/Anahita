#ifndef CRUCIFIX_H
#define CRUCIFIX_H

#include "base_class.h"

class Crucifix : public Base_class
{
public:
    Crucifix();
    ~Crucifix();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;

private:
    image_transport::Publisher front_roi_pub;
};

#endif // CRUCIFIX_H
