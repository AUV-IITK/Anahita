#ifndef GATE_TASK_H
#define GATE_TASK_H

#include "base_class.h"

class Gate : public Base_class
{
public:
    Gate();
    void loadParams() override;
    void spinThreadFront() override;

private:
    image_transport::Publisher front_roi_pub;
};

#endif // GATE_TASK_H
