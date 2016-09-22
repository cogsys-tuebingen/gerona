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
    static void setLimits(double dis2p, double dis2o);
    static void setVel(double vel);
    double getLimit();

    virtual bool isSatisfied(const LNode& point) override;
private:
    static double D_RATE, DIS2P_, DIS2O_, vel_;
    double limit;
    int level;
};

#endif // DIS2PATH_CONSTRAINT_H
