#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerTransformer : public LocalPlanner
{
public:
    LocalPlannerTransformer(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath() override;

private:
    ros::Time last_update_;
    ros::Duration update_interval_;
};

#endif // LOCAL_PLANNER_TRANSFORMER_H
