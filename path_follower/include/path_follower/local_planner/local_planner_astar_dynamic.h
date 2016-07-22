#ifndef LOCAL_PLANNER_ASTAR_DYNAMIC_H
#define LOCAL_PLANNER_ASTAR_DYNAMIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

class LocalPlannerAStarDynamic : public LocalPlannerClassic
{
public:
    LocalPlannerAStarDynamic(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
private:
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     int& nnodes) override;

private:
    ros::NodeHandle pnh_;
    ros::ServiceClient map_service_client_;

    ros::Publisher local_map_pub_;

    double size_forward;
    double size_backward;
    double size_width;
};

#endif // LOCAL_PLANNER_ASTAR_DYNAMIC_H
