#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>
#include <tf/transform_datatypes.h>
#include <utils_general/Stopwatch.h>

class Constraint
{
public:
    typedef std::shared_ptr<Constraint> Ptr;

public:
    Constraint();
    virtual ~Constraint();

    virtual bool isSatisfied(const tf::Point& point) = 0;
protected:
    Stopwatch sw;
};

#endif // CONSTRAINT_H
