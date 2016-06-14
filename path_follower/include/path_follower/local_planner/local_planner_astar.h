#ifndef LOCAL_PLANNER_ASTAR_H
#define LOCAL_PLANNER_ASTAR_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

class LocalPlannerAStar : public LocalPlannerClassic
{
public:
    LocalPlannerAStar(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer) override;

private:
    typedef std::multiset<HNode*,CompareLNode> prio_queue;
};

#endif // LOCAL_PLANNER_ASTAR_H
