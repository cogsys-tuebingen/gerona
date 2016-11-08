#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

/// PROJECT
#include <path_follower/local_planner/local_planner_implemented.h>

class LocalPlannerTransformer : public LocalPlannerImplemented
{
public:
    LocalPlannerTransformer();

    virtual void setParams(int nnodes, int ic, double dis2p, double adis, double fdis, double s_angle,
                           int ia, double lmf, int max_level, double mu, double ef) override;
    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;
private:
    virtual void printNodeUsage(std::size_t& nnodes) const override;
    virtual void printVelocity() override;
    virtual void printLevelReached() const override;
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     std::size_t& nnodes) override;
};

#endif // LOCAL_PLANNER_TRANSFORMER_H
