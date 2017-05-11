#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

/// PROJECT
#include <path_follower/local_planner/local_planner_implemented.h>

class LocalPlannerTransformer : public LocalPlannerImplemented
{
public:
    LocalPlannerTransformer();

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;

private:
    virtual void setParams(const LocalPlannerParameters& opt) override;
    virtual void printNodeUsage(std::size_t& nnodes) const override;
    virtual void printVelocity() override;
    virtual void printLevelReached() const override;
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     std::size_t& nnodes) override;
};

#endif // LOCAL_PLANNER_TRANSFORMER_H
