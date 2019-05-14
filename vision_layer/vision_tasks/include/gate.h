#ifndef GATE_TASK_H
#define GATE_TASK_H

#include "base_class.h"

class Gate : public Base_class
{
public:
    Gate();
    ~Gate();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;
};

#endif // GATE_TASK_H
