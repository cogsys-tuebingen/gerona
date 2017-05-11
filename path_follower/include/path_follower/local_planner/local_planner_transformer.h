#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

/// PROJECT
#include <path_follower/local_planner/abstract_local_planner.h>

class LocalPlannerTransformer : public AbstractLocalPlanner
{
public:
    LocalPlannerTransformer();

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;

    virtual Path::Ptr updateLocalPath() override;

private:
    virtual void setParams(const LocalPlannerParameters& opt) override;
};

#endif // LOCAL_PLANNER_TRANSFORMER_H
