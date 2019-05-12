#ifndef GATE_TASK_H
#define GATE_TASK_H

#include "base_class.h"

class Gate : public Base_class
{
public:
    using Base_class::frontTaskHandling;
    Gate();
    ~Gate();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;
    // void frontTaskHandling(bool status);
};

#endif // GATE_TASK_H
