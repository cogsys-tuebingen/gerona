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
                                      const std::vector<double>& wscorer) override;

    virtual void setGlobalPath(Path::Ptr path) override;
protected:
    int transform2Odo(SubPath& waypoints, ros::Time& now);

    void setPath(Path::Ptr& local_path, SubPath& local_wps, ros::Time& now);

    void printSCTimeUsage(const std::vector<Constraint::Ptr>& constraints,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<bool>& fconstraints,
                          const std::vector<double>& wscorer);
private:
    virtual void printNodeUsage(int& nnodes) const = 0;
    virtual int algo(Eigen::Vector3d& pose, SubPath& waypoints, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     int& nnodes) = 0;
protected:
    ros::Time last_update_;
    ros::Duration update_interval_;
};

#endif // LOCAL_PLANNER_IMPLEMENTED_H
