#ifndef DIS2PATH_CONSTRAINT_H
#define DIS2PATH_CONSTRAINT_H

#include <path_follower/local_planner/constraint.h>

class Dis2Path_Constraint : public Constraint
{
public:
    typedef std::shared_ptr<Dis2Path_Constraint> Ptr;

public:
    Dis2Path_Constraint();
    virtual ~Dis2Path_Constraint();

    virtual bool isSatisfied(const LNode& point) override;
private:
    double limit;
};

#endif // DIS2PATH_CONSTRAINT_H
