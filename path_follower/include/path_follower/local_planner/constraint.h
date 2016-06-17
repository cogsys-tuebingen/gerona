#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <memory>
#include <path_follower/utils/path.h>
#include <utils_general/Stopwatch.h>

class Constraint
{
public:
    typedef std::shared_ptr<Constraint> Ptr;

public:
    Constraint();
    virtual ~Constraint();

    virtual bool isSatisfied(const LNode& point) = 0;
    long nsUsed();
protected:
    Stopwatch sw;
};

#endif // CONSTRAINT_H
