#ifndef LOCAL_PLANNER_NULL_H
#define LOCAL_PLANNER_NULL_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

class LocalPlannerNull : public LocalPlanner
{
public:
    LocalPlannerNull();

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath(Path::Ptr& wlp) override;
    virtual bool isNull() const override;
    virtual void setParams(const LocalPlannerParameters& opt) override;
    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;
};

#endif // LOCAL_PLANNER_NULL_H
