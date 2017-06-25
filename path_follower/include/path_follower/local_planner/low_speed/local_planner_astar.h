#ifndef LOCAL_PLANNER_ASTAR_H
#define LOCAL_PLANNER_ASTAR_H

/// PROJECT
#include <path_follower/local_planner/abstract_local_planner.h>

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>

namespace lib_path
{
class CollisionGridMap2d;
}

namespace low_speed
{

class LocalPlannerAStar : public AbstractLocalPlanner
{
public:
    LocalPlannerAStar();

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;

    virtual Path::Ptr updateLocalPath() override;

private:
    virtual void setParams(const LocalPlannerParameters& opt) override;

    Path::Ptr doUpdateLocalPath();


public:
    virtual void setGlobalPath(Path::Ptr path) override;

    virtual void reset() override;

private:
    Path::Ptr calculateAvoidingPath(const std::string& frame);
    Path::Ptr calculateFinalAvoidingPath(const std::string& frame);
    SubPath transformPath(const SubPath& path_map, const tf::Transform& trafo);
    Waypoint transformWaypoint(const Waypoint& wp, const tf::Transform& trafo);

    void updateMap();
    bool integrateObstacles();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher local_map_pub_;

    double size_forward;
    double size_backward;
    double size_width;


    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid local_map;
    std::shared_ptr<lib_path::CollisionGridMap2d> map_info_static;

    std::shared_ptr<lib_path::CollisionGridMap2d> map_info;

    SubPath waypoints_odom;
};

}

#endif // LOCAL_PLANNER_ASTAR_H
