#ifndef LOCAL_PLANNER_IMPLEMENTED_H
#define LOCAL_PLANNER_IMPLEMENTED_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerImplemented : public LocalPlanner
{
public:
    LocalPlannerImplemented();

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer,
                                      Path::Ptr& wlp) override;

    virtual void setGlobalPath(Path::Ptr path) override;
private:
    bool transform2Odo(ros::Time& now);

    void setPath(Path::Ptr& local_path, Path::Ptr& wlp, SubPath& local_wps, ros::Time& now);

    void printSCTimeUsage(const std::vector<Constraint::Ptr>& constraints,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<bool>& fconstraints,
                          const std::vector<double>& wscorer);

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
    SubPath waypoints, wlp_;
    SubPath waypoints_map;

    bool tooClose;

    ros::Time last_update_;
};

#endif // LOCAL_PLANNER_IMPLEMENTED_H
