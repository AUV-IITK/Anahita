#ifndef TORPEDO_TASK_H
#define TORPEDO_TASK_H

#include "base_class.h"

class Torpedo : public Base_class
{
	public:
		Torpedo();
		~Torpedo();
		virtual void loadParams () override;
		virtual void spinThreadFront () override;
		virtual void spinThreadBottom () override;
};
#endif // TORPEDO_TASK_H

