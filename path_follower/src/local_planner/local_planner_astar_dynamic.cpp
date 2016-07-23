/// HEADER
#include <path_follower/local_planner/local_planner_astar_dynamic.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <utils_path/common/CollisionGridMap2d.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>
#include <utils_path/generic/SteeringNeighborhood.hpp>
#include <utils_path/generic/SteeringNode.hpp>
#include <utils_path/generic/Heuristics.hpp>

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <ros/console.h>

namespace {

template <typename Algorithm>
struct NearPathTest
{
    NearPathTest(const LocalPlannerAStarDynamic& parent, const SubPath& map_path,
                 Algorithm& algo, const lib_path::Pose2d& start, const nav_msgs::OccupancyGrid& map, const lib_path::SimpleGridMap2d* map_info)
        : parent(parent), map_path(map_path),
          algo(algo), map(map), map_info(map_info),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          start_cell(start),

          candidates(0)
    {
        min_dist = 3.0;

        double dist_accum = 0.0;
        const Waypoint* last_wp = &map_path.front();
        for(const Waypoint& wp : map_path) {
            dist_accum += wp.distanceTo(*last_wp);

            if(dist_accum >= min_dist) {
                map_info->point2cellSubPixel(wp.x, wp.y, heuristic_goal.x,heuristic_goal.y);
                heuristic_goal.theta = wp.orientation;
                return;
            }

            last_wp = &wp;
        }

        const Waypoint& wp = map_path.back();
        map_info->point2cellSubPixel(wp.x, wp.y, heuristic_goal.x,heuristic_goal.y);
        heuristic_goal.theta = wp.orientation;
    }

    void reset()
    {
        candidates = 0;
    }

    bool terminate(const typename Algorithm::NodeT* node) const
    {
        if(node->depth < 3) {
            return false;
        }

        int x = node->x;
        int y = node->y;

        if(x < 0 || x >= w || y < 0 || y >= h) {
            return false;
        }

        double dist = std::hypot(x - start_cell.x, y - start_cell.y) * map_info->getResolution();

        if(min_dist > 0.0 && dist < min_dist) {
            return false;
        }

        double wx, wy;
        map_info->cell2pointSubPixel(x,y, wx,wy);
        //        wx = x;
        //        wy = y;


        const Waypoint* closest_wp = nullptr;
        double closest_dist = std::numeric_limits<double>::infinity();
        for(const Waypoint& wp : map_path) {
            double dist = std::hypot(wp.x - wx, wp.y - wy);
            if(dist < closest_dist) {
                closest_dist = dist;
                closest_wp = &wp;
            }
        }


        if(closest_dist > 2.0) {
            return false;
        }

        //        algo.addGoalCandidate(node, node->distance);
        algo.addGoalCandidate(node, closest_dist);
        ++candidates;

        return candidates > 5;
    }

    const lib_path::Pose2d* getHeuristicGoal() const
    {
        return &heuristic_goal;
    }

    const LocalPlannerAStarDynamic& parent;
    const SubPath& map_path;

    Algorithm& algo;
    const nav_msgs::OccupancyGrid& map;
    const lib_path::SimpleGridMap2d * map_info;

    double res;
    double ox;
    double oy;
    int w;
    int h;

    lib_path::Pose2d heuristic_goal;

    double min_dist;

    lib_path::Pose2d start_cell;

    mutable int candidates;
};


lib_path::Pose2d  convertToCell(const lib_path::SimpleGridMap2d * map_info, const tf::Pose& pose)
{
    tf::Vector3 pt = pose.getOrigin();

    lib_path::Pose2d res;
    unsigned tmpx, tmpy;
    map_info->point2cell(pt.getX(), pt.getY(), tmpx, tmpy);
    res.x = tmpx;
    res.y = tmpy;
    res.theta = tf::getYaw(pose.getRotation()) - map_info->getRotation();
    return res;
}

}

LocalPlannerAStarDynamic::LocalPlannerAStarDynamic(PathFollower &follower,
                                                   tf::Transformer& transformer,
                                                   const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),

      pnh_("~"),
      avoiding(false)
{
    pnh_.param("size/forward", size_forward, 0.2);
    pnh_.param("size/backward", size_backward, -0.55);
    pnh_.param("size/width", size_width, 0.45);

    std::string map_service = "/static_map";
    pnh_.param("map_service",map_service, map_service);

    map_service_client_ = pnh_.serviceClient<nav_msgs::GetMap> (map_service);

    local_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
}

bool LocalPlannerAStarDynamic::isPathObstructed(const SubPath &path, int start, size_t end, int radius)
{
    for(std::size_t i = std::max(0, start), n = std::min(end, path.size()); i < n; ++i) {
        const Waypoint& wp = path[i];

        unsigned wpx, wpy;
        map_info->point2cell(wp.x, wp.y, wpx, wpy);

        for(int dy = -radius; dy <= radius; ++dy) {
            for(int dx = -radius; dx <= radius; ++dx) {
                int nx = wpx + dx;
                int ny = wpy + dy;

                if(!map_info->isInMap(nx, ny) || map_info->isOccupied(nx, ny)) {
                    return true;
                }
            }
        }
    }

    return false;
}

size_t LocalPlannerAStarDynamic::findClosestWaypoint(const Eigen::Vector3d& pose, const SubPath &current_subpath_map)
{
    double closest_dist = std::numeric_limits<double>::infinity();
    std::size_t closest_index = 0;
    for(std::size_t i = 0; i < current_subpath_map.size(); ++i) {
        const Waypoint& wp = current_subpath_map[i];
        double dist = std::hypot(wp.x - pose(0), wp.y - pose(1));
        if(dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }

    return closest_index;
}


void LocalPlannerAStarDynamic::setGlobalPath(Path::Ptr path)
{
    avoiding = false;
    avoiding_path_map_.clear();

    LocalPlannerImplemented::setGlobalPath(path);
}

bool LocalPlannerAStarDynamic::algo(Eigen::Vector3d& odom_pose_v, SubPath& local_wps,
                                    const std::vector<Constraint::Ptr>& constraints,
                                    const std::vector<Scorer::Ptr>& scorer,
                                    const std::vector<bool>& fconstraints,
                                    const std::vector<double>& wscorer,
                                    int& nnodes){



    // calculate the corrective transformation to map from world coordinates to odom
    ros::Time time(0);
    if(!transformer_.waitForTransform("map", "odom", time, ros::Duration(0.1))) {
        ROS_WARN_NAMED("local_path", "cannot transform map to odom");
        return false;
    }

    tf::StampedTransform now_map_to_odom;
    transformer_.lookupTransform("map", "odom", time, now_map_to_odom);

    if(!map_info_static) {
        // HINT: for now we assume a static map
        // TODO: implement support for dynamic maps
//        updateMap();
    }
    updateMap();
    map_info = map_info_static;

    if(!integrateObstacles()) {
        return false;
    }

    Eigen::Vector3d map_pose = transformPose(odom_pose_v, now_map_to_odom);

    const SubPath &current_subpath_map = avoiding ? avoiding_path_map_ : waypoints_map;
    tf::Transform odom_to_map = now_map_to_odom.inverse();
    SubPath current_subpath_odom = transformPath(current_subpath_map, odom_to_map);

    // find the subpath that starts closest to the robot
    std::size_t closest_wp_map_index = findClosestWaypoint(map_pose, current_subpath_map);

    int cell_width = 2 * size_width * map_info->getResolution();
    int radius = std::ceil(cell_width / 2.) + 1;

    if(isPathObstructed(current_subpath_map, closest_wp_map_index - 20, closest_wp_map_index + 150, radius)) {
        ROS_WARN("new avoiding plan");
        avoiding_path_map_ = calculateAvoidingPath(odom_pose_v, odom_to_map);

        local_wps = transformPath(avoiding_path_map_, odom_to_map);

    } else {
        ROS_INFO("continue");
        Eigen::Vector3d pose = follower_.getRobotPose();
        std::size_t closest_local_wp_index = findClosestWaypoint(pose, current_subpath_odom);
        local_wps.assign(current_subpath_odom.begin() + closest_local_wp_index, current_subpath_odom.end());

        if(avoiding) {
            if(local_wps.size() < 15 || closest_local_wp_index > 30) {
                SubPath waypoints_now = transformPath(waypoints_map, odom_to_map);
                std::size_t closest_local_wp_now_index = findClosestWaypoint(pose, waypoints_now);
                const Waypoint& closest_wp = waypoints_now.at(closest_local_wp_now_index);
                double distance = std::hypot(closest_wp.x -pose(0), closest_wp.y - pose(1));

                ROS_INFO_STREAM("avoiding. distance: " << distance);


                if(distance < 0.1) {
                    ROS_INFO_STREAM("switch back to planned path");
                    avoiding = false;
                    std::size_t closest_local_wp_index = findClosestWaypoint(pose, waypoints_now);
                    local_wps.assign(waypoints_now.begin() + closest_local_wp_index, waypoints_now.end());

                } else {
                    ROS_INFO_STREAM("plan new avoidance path");
                    try {
                        avoiding_path_map_ = calculateAvoidingPath(odom_pose_v, odom_to_map);
                        local_wps = transformPath(avoiding_path_map_, odom_to_map);

                    } catch(const std::runtime_error& e) {
                        ROS_ERROR("planning failed, keep last plan");
                    }
                }
            }
        }
    }

    if(local_wps.empty()) {
        ROS_WARN("local path is empty");
        return false;
    }

    return true;
}

SubPath LocalPlannerAStarDynamic::transformPath(const SubPath &path_map, const tf::Transform& trafo)
{
    SubPath avoiding_path_odom;
    for (std::size_t i = 0, total = path_map.size(); i < total; ++i) {
        avoiding_path_odom.push_back(transformWaypoint(path_map[i], trafo));
    }
    smoothAndInterpolate(avoiding_path_odom);

    return avoiding_path_odom;
}

Waypoint LocalPlannerAStarDynamic::transformWaypoint(const Waypoint &wp, const tf::Transform& trafo)
{
    tf::Pose p(tf::createQuaternionFromYaw(wp.orientation), tf::Vector3(wp.x, wp.y, 0.0));
    tf::Pose tp = trafo * p;

    Waypoint wp_trans;
    wp_trans.x = tp.getOrigin().x();
    wp_trans.y = tp.getOrigin().y();
    wp_trans.orientation = tf::getYaw(tp.getRotation());

    return wp_trans;
}

Eigen::Vector3d LocalPlannerAStarDynamic::transformPose(const Eigen::Vector3d &pose, const tf::Transform& trafo)
{
    tf::Pose p(tf::createQuaternionFromYaw(pose(2)), tf::Vector3(pose(0), pose(1), 0.0));
    tf::Pose tp = trafo * p;

    Eigen::Vector3d pose_trans(tp.getOrigin().x(), tp.getOrigin().y(), tf::getYaw(tp.getRotation()));

    return pose_trans;
}

SubPath LocalPlannerAStarDynamic::calculateAvoidingPath(const Eigen::Vector3d& odom_pose_v, const tf::Transform& odom_to_map)
{
    tf::Pose odom_pose(tf::createQuaternionFromYaw(odom_pose_v(2)), tf::Vector3(odom_pose_v(0), odom_pose_v(1), 0.0));
    tf::Pose map_pose = odom_to_map.inverse() * odom_pose;

    // find path
    lib_path::Pose2d pose_cell = convertToCell(map_info.get(), map_pose);

    typedef
    lib_path::AStarSearch<lib_path::SteeringNeighborhood<60, 4, 15, lib_path::SteeringMoves::FORWARD_HALFSTEPS, false>, lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 >
            AStarPatsyForward;

    AStarPatsyForward algo_forward;
    algo_forward.setMap(map_info.get());

    algo_forward.setPathCandidateCallback([this](const typename AStarPatsyForward::PathT& path) {
        std::cout << "path candidate found: " << path.size() << std::endl;
        return false;
    });

    NearPathTest<AStarPatsyForward> goal_test_forward(*this, waypoints_map, algo_forward, pose_cell, map, map_info.get());

    ROS_INFO_STREAM("start: " << map_pose.getOrigin().getX() << ", " << map_pose.getOrigin().getY() << " -> node " << pose_cell.x << " / " << pose_cell.y << " / " << pose_cell.theta);
    auto path_start = algo_forward.findPath(pose_cell, goal_test_forward);
    if(path_start.empty()) {
        ROS_ERROR("found no local path");
        throw std::runtime_error("found no local path");
    }

    ROS_WARN_STREAM("found a local path of length " << path_start.size());

    SubPath avoiding_path_map;
    avoiding = true;

    for (std::size_t i = 0, total = path_start.size(); i < total; ++i) {
        auto pt = path_start.at(i);

        Waypoint wp_map;
        map_info->cell2pointSubPixel(pt.x, pt.y, wp_map.x, wp_map.y);
        wp_map.orientation = pt.theta;

        avoiding_path_map.push_back(wp_map);

        ROS_INFO_STREAM(pt.x << ", " << pt.y << " -> node " << wp_map.x << " / " << wp_map.y << " / " << pt.theta);
    }


    return avoiding_path_map;
}

void LocalPlannerAStarDynamic::updateMap()
{
    // get map
    nav_msgs::GetMap map_service;
    if(!map_service_client_.exists()) {
        map_service_client_.waitForExistence();
    }
    if(!map_service_client_.call(map_service)) {
        throw std::runtime_error("map service lookup failed");
    }

    map =  map_service.response.map;

    unsigned w = map.info.width;
    unsigned h = map.info.height;


    map_info_static.reset(new lib_path::CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width));

    std::vector<uint8_t> data(w*h);
    int i = 0;

    /// Map data
    /// -1: unknown -> 0
    /// 0:100 probabilities -> 1 - 100
    for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
        data[i++] = std::min(100, *it + 1);
    }

    map_info_static->setLowerThreshold(50);
    map_info_static->setUpperThreshold(70);
    map_info_static->setNoInformationValue(-1);

    map_info_static->set(data, w, h, 0.0);
    map_info_static->setOrigin(lib_path::Point2d(map.info.origin.position.x, map.info.origin.position.y));
}

bool LocalPlannerAStarDynamic::integrateObstacles()
{
    tf::StampedTransform trafo;
    ros::Time time;
    time.fromNSec(obstacle_cloud_->header.stamp * 1e3);
    if(!transformer_.waitForTransform("/map", obstacle_cloud_->header.frame_id, time, ros::Duration(0.1))) {
        ROS_ERROR_STREAM("cannot transform the obstacle cloud to the map frame at time " << time << " (now is " << ros::Time::now() << ")") ;
        return false;
    }
    transformer_.lookupTransform("/map", obstacle_cloud_->header.frame_id, time, trafo);

    int OBSTACLE = 100;

    map_info.reset(new lib_path::CollisionGridMap2d(*map_info_static));

    nav_msgs::OccupancyGrid local_map = map;
    local_map.header.frame_id = "/map";

    unsigned w = map.info.width;

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obstacle_cloud_->begin(); it != obstacle_cloud_->end(); ++it) {
        const pcl::PointXYZ& pt = *it;

        tf::Vector3 pt_cloud(pt.x, pt.y, pt.z);
        tf::Vector3 pt_map = trafo * pt_cloud;

        unsigned int x,y;
        if(map_info->point2cell(pt_map.x(), pt_map.y(), x, y)) {
            map_info->setValue(x,y, OBSTACLE);

            local_map.data.at(y * w + x) = 100;
        }
    }


    local_map_pub_.publish(local_map);

    return true;
}
