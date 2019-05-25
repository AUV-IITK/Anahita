#ifndef GATE_TASK_H
#define GATE_TASK_H

#include "base_class.h"

class Gate : public Base_class
{
public:
    Gate();
   virtual void loadParams() override;
    virtual void spinThreadFront() override;
    virtual void spinThreadBottom() override;

private:
    image_transport::Publisher front_roi_pub;
};

#endif // GATE_TASK_H
