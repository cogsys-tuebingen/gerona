#ifndef LOCAL_PLANNER_ASTAR_H
#define LOCAL_PLANNER_ASTAR_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerAStar : public LocalPlanner
{
public:
    LocalPlannerAStar(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer) override;
    bool operator() (const int& lhs, const int&rhs) const;

private:
    typedef std::multiset<int,LocalPlannerAStar> prio_queue;
    ros::Time last_update_;
    ros::Duration update_interval_;
    std::vector<double> fScore;
};

#endif // LOCAL_PLANNER_ASTAR_H
