#ifndef LOCAL_PLANNER_NULL_H
#define LOCAL_PLANNER_NULL_H

/// PROJECT
#include <path_follower/local_planner/abstract_local_planner.h>

class LocalPlannerNull : public AbstractLocalPlanner
{
public:
    LocalPlannerNull();

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath() override;
    virtual bool isNull() const override;
    virtual void setParams(const LocalPlannerParameters& opt) override;
    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;
};

#endif // LOCAL_PLANNER_NULL_H
