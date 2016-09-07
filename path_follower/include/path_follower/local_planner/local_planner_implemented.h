#ifndef LOCAL_PLANNER_IMPLEMENTED_H
#define LOCAL_PLANNER_IMPLEMENTED_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerImplemented : public LocalPlanner
{
public:
    LocalPlannerImplemented(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer,
                                      Path::Ptr& wlp) override;

    virtual void setGlobalPath(Path::Ptr path) override;
protected:
    bool transform2Odo(ros::Time& now);

    void setPath(Path::Ptr& local_path, Path::Ptr& wlp, SubPath& local_wps, ros::Time& now);

    void printSCTimeUsage(const std::vector<Constraint::Ptr>& constraints,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<bool>& fconstraints,
                          const std::vector<double>& wscorer);
private:
    virtual void printNodeUsage(std::size_t& nnodes) const = 0;
    virtual void printVelocity() = 0;
    virtual void printLevelReached() const = 0;
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     std::size_t& nnodes) = 0;
protected:
    ros::Time last_update_;
    ros::Duration update_interval_;

    SubPath waypoints, wlp_;

    bool tooClose;
};

#endif // LOCAL_PLANNER_IMPLEMENTED_H
