#ifndef LOCAL_PLANNER_ASTAR_DYNAMIC_H
#define LOCAL_PLANNER_ASTAR_DYNAMIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>

namespace lib_path
{
class CollisionGridMap2d;
}

class LocalPlannerAStarDynamic : public LocalPlannerClassic
{
public:
    LocalPlannerAStarDynamic(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual void reset() override;

private:
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     int& nnodes) override;

    SubPath calculateAvoidingPath(const Vector3d &odom_pose_v, const tf::Transform &odom_to_map);
    SubPath transformPath(const SubPath& path_map, const tf::Transform& trafo);
    Waypoint transformWaypoint(const Waypoint& wp, const tf::Transform& trafo);
    Eigen::Vector3d transformPose(const Eigen::Vector3d& wp, const tf::Transform& trafo);

    void updateMap();
    bool integrateObstacles();

    bool isPathObstructed(const SubPath &path, int start, std::size_t end, int radius);

private:
    ros::NodeHandle pnh_;
    ros::ServiceClient map_service_client_;

    ros::Publisher local_map_pub_;

    double size_forward;
    double size_backward;
    double size_width;


    nav_msgs::OccupancyGrid map;
    std::shared_ptr<lib_path::CollisionGridMap2d> map_info_static;

    std::shared_ptr<lib_path::CollisionGridMap2d> map_info;

    bool avoiding;
    SubPath avoiding_path_map_;
};

#endif // LOCAL_PLANNER_ASTAR_DYNAMIC_H
