#ifndef PATH_MARKER_H
#define PATH_MARKER_H

#include "base_class.h"

class PathMarker : public Base_class
{
public:
    PathMarker();
    ~PathMarker();
    virtual void loadParams () override;
    virtual void spinThreadFront () override;
    virtual void spinThreadBottom () override;
};

#endif // PATH_MARKER_H
