#ifndef OCTAGON_TASK_H
#define OCTAGON_TASK_H

#include "base_class.h" 

class Octagon : public Base_class {	
public:
    Octagon();
	void spinThreadBottom() override;
    void spinThreadFront() override;
    void loadParams() override;
};
#endif // OCTAGON_TASK_H

