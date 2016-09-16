#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

/// PROJECT
#include <path_follower/local_planner/local_planner_implemented.h>

class LocalPlannerTransformer : public LocalPlannerImplemented
{
public:
    LocalPlannerTransformer(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
    virtual void setParams(int nnodes, int ic, double dis2p, double dis2o, double s_angle, int ia, double lmf, int max_level) override;
    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;
private:
    virtual void printNodeUsage(std::size_t& nnodes) const override;
    virtual void printVelocity() override;
    virtual void printLevelReached() const override;
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     std::size_t& nnodes) override;
};

#endif // LOCAL_PLANNER_TRANSFORMER_H
