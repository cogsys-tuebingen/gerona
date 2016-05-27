#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>
#include <path_follower/local_planner/lnode.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerBFS : public LocalPlanner
{
public:
    LocalPlannerBFS(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer) override;

private:
    ros::Time last_update_;
    ros::Duration update_interval_;
};

#endif // LOCAL_PLANNER_BFS_H
