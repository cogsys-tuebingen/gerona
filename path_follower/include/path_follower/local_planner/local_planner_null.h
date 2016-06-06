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

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<bool>& fscorer) override;
    virtual bool isNull() const override;
};

#endif // LOCAL_PLANNER_NULL_H
