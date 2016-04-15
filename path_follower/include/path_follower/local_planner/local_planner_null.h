#ifndef LOCAL_PLANNER_NULL_H
#define LOCAL_PLANNER_NULL_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

class LocalPlannerNull : public LocalPlanner
{
public:
    LocalPlannerNull(PathFollower& controller,
                     tf::Transformer &transformer);

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath() override;
    virtual bool isNull() const override;
};

#endif // LOCAL_PLANNER_NULL_H
