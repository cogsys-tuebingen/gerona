#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

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
    void getSuccessors(const Waypoint& current, int index, std::vector<int>& successors,
                       std::vector<Waypoint>& nodes, std::vector<int>& parents);
    bool isNearEnough(const Waypoint& current, const Waypoint& last);
    const double D_THETA = 5*M_PI/36;//Assume like the global planner 25° turn
    ros::Time last_update_;
    ros::Duration update_interval_;
};

#endif // LOCAL_PLANNER_BFS_H
