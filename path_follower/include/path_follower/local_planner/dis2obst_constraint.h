#ifndef DIS2OBST_CONSTRAINT_H
#define DIS2OBST_CONSTRAINT_H

#include <path_follower/local_planner/constraint.h>

class Dis2Obst_Constraint : public Constraint
{
public:
    typedef std::shared_ptr<Dis2Obst_Constraint> Ptr;

public:
    Dis2Obst_Constraint();
    virtual ~Dis2Obst_Constraint();
    static void setLimit(double dis2o);
    static void setVDis(double dis);
    double computeFrontier(double angle);

    virtual bool isSatisfied(const LNode& point) override;
private:
    static double DIS2O_, full_d;
};

#endif // DIS2OBST_CONSTRAINT_H
