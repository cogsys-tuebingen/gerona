#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>
#include <tf/transform_datatypes.h>

class Constraint
{
public:
    typedef std::shared_ptr<Constraint> Ptr;

public:
    Constraint();
    virtual ~Constraint();

    virtual bool isSatisfied(const tf::Point& point) = 0;
};

#endif // CONSTRAINT_H
