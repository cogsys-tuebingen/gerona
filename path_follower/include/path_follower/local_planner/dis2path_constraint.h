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
    void setParams(double new_limit);
    static void setDRate(double d_rate);
    static void setLimit(double dis2p);
    double getLimit();

    virtual bool isSatisfied(const LNode& point) override;
private:
    static double D_RATE, DIS2P_;
    double limit;
    int level;
};

#endif // DIS2PATH_CONSTRAINT_H
