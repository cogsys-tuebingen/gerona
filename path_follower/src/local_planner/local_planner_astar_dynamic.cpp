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

using namespace lib_path;

namespace {


size_t findClosestWaypoint(const Eigen::Vector3d& pose, const SubPath &current_subpath_map)
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


template <typename Algorithm>
struct NearPathTest
{
    NearPathTest(const LocalPlannerAStarDynamic& parent, const SubPath& map_path,
                 Algorithm& algo, const Pose2d& start_cell, const nav_msgs::OccupancyGrid& map, const SimpleGridMap2d* map_info)
        : parent(parent), map_path(map_path),
          algo(algo), map(map), map_info(map_info),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          start_cell(start_cell),

          candidates(0)
    {
        min_dist = 3.5;
        desired_dist = 5.0;

        updateHeuristicGoal();
    }

    void updateHeuristicGoal()
    {
        double mx,my;
        map_info->cell2pointSubPixel(start_cell.x, start_cell.y, mx, my);
        Eigen::Vector3d start_pose(mx,my, start_cell.theta);

        std::size_t start_index = findClosestWaypoint(start_pose, map_path);

        double dist_accum = 0.0;
        const Waypoint* last_wp = &map_path.at(start_index);
        for(std::size_t i = start_index, n = map_path.size(); i < n; ++i) {
            const Waypoint& wp = map_path[i];
            dist_accum += wp.distanceTo(*last_wp);

            if(dist_accum >= desired_dist * 2) {
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

        if(dist < min_dist) {
            return false;
        }

        double wx, wy;
        map_info->cell2pointSubPixel(x,y, wx,wy);
        //        wx = x;
        //        wy = y;



        static const double distance_tolerance = 0.33;
        static const double angle_tolerance = M_PI / 8;

        double closest_dist = std::numeric_limits<double>::infinity();
        for(const Waypoint& wp : map_path) {
            double dtheta = MathHelper::AngleDelta(wp.orientation, node->theta);
            if(std::abs(dtheta) > angle_tolerance) {
                continue;
            }

            double dist = std::hypot(wp.x - wx, wp.y - wy);
            if(dist < closest_dist) {
                closest_dist = dist;
            }
        }

        if(dist > desired_dist) {
            if(closest_dist > distance_tolerance) {
                return false;
            }
        } else {
            if(closest_dist > 2 * distance_tolerance) {
                return false;
            }
        }

        //        algo.addGoalCandidate(node, node->distance);
        algo.addGoalCandidate(node, closest_dist);
        ++candidates;

        if(dist >= desired_dist) {
            if(closest_dist < distance_tolerance) {
                return true;
            }
        }

        return candidates > 2;
    }

    const Pose2d* getHeuristicGoal() const
    {
        return &heuristic_goal;
    }

    const LocalPlannerAStarDynamic& parent;
    const SubPath& map_path;

    Algorithm& algo;
    const nav_msgs::OccupancyGrid& map;
    const SimpleGridMap2d * map_info;

    double res;
    double ox;
    double oy;
    int w;
    int h;

    Pose2d heuristic_goal;

    double min_dist;
    double desired_dist;

    Pose2d start_cell;

    mutable int candidates;
};


Pose2d  convertToCell(const SimpleGridMap2d * map_info, const tf::Pose& pose)
{
    tf::Vector3 pt = pose.getOrigin();

    Pose2d res;
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
    pnh_.param("size/forward", size_forward, 0.15);
    pnh_.param("size/backward", size_backward, -0.8);
    pnh_.param("size/width", size_width, 0.60);

    std::string map_service = "/static_map";
    pnh_.param("map_service",map_service, map_service);

    map_service_client_ = pnh_.serviceClient<nav_msgs::GetMap> (map_service);

    local_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
}

bool LocalPlannerAStarDynamic::isPathObstructed(const SubPath &path, int start, size_t end, int radius)
{
    return false;
    int obstacle_count = 0;
    double res = map_info->getResolution();
    double r = radius * res;

    for(std::size_t i = std::max(0, start), n = std::min(end, path.size()); i < n; ++i) {
        const Waypoint& wp = path[i];

        unsigned wpx, wpy;
        map_info->point2cell(wp.x, wp.y, wpx, wpy);

        for(int dy = -radius; dy <= radius; ++dy) {
            for(int dx = -radius; dx <= radius; ++dx) {
                int nx = wpx + dx;
                int ny = wpy + dy;

                if(!map_info->isInMap(nx, ny) || map_info->isOccupied(nx, ny, wp.orientation)) {
                    geometry_msgs::Point c;
                    c.x = wp.x + dx * res;
                    c.y = wp.y + dy * res;
                    follower_.getVisualizer().drawCircle(obstacle_count++, c, r, "/map", "local_obstacles", 1.0, 0.0, 0.0, 1.0, 0.5);
                }
            }
        }
    }

    return obstacle_count > 0;
}

void LocalPlannerAStarDynamic::reset()
{
    avoiding = false;
    avoiding_path_map_.clear();

    tooClose = true;
}

void LocalPlannerAStarDynamic::setGlobalPath(Path::Ptr path)
{
    reset();
    LocalPlannerImplemented::setGlobalPath(path);
}

bool LocalPlannerAStarDynamic::algo(Eigen::Vector3d& odom_pose_v, SubPath& local_wps,
                                    const std::vector<Constraint::Ptr>& constraints,
                                    const std::vector<Scorer::Ptr>& scorer,
                                    const std::vector<bool>& fconstraints,
                                    const std::vector<double>& wscorer,
                                    int& nnodes){

    tooClose = false;

    // calculate the corrective transformation to map from world coordinates to odom
    ros::Time time(0);
    if(!transformer_.waitForTransform("map", "odom", time, ros::Duration(0.1))) {
        ROS_WARN_NAMED("local_path", "cannot transform map to odom");
        return false;
    }

    tf::StampedTransform map_to_odom;
    transformer_.lookupTransform("map", "odom", time, map_to_odom);

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

    Eigen::Vector3d map_pose = transformPose(odom_pose_v, map_to_odom);

    const SubPath &current_subpath_map = avoiding ? avoiding_path_map_ : waypoints_map;
    tf::Transform odom_to_map = map_to_odom.inverse();
    SubPath current_subpath_odom = transformPath(current_subpath_map, odom_to_map);

    // find the subpath that starts closest to the robot
    int closest_wp_map_index = findClosestWaypoint(map_pose, current_subpath_map);

    int cell_width = size_width / map_info->getResolution();
    int radius = std::ceil(cell_width / 2.);
    radius = 2;

    double look_back_distance = 1.0;
    double look_forward_distance = 5.0;

    double path_resolution = 0.1;


    typedef
    AStarDynamicSearch<SteeringNeighborhood<40, 4, 15, 60, 120, SteeringMoves::FORWARD, false>, NoExpansion, Pose2d, GridMap2d, 500 >
            AStarPatsyForward;
    typedef
    AStarDynamicSearch<SteeringNeighborhood<40, 4, 15, 60, 120, SteeringMoves::FORWARD, true>, NoExpansion, Pose2d, GridMap2d, 500 >
            AStarPatsyReverse;

    ROS_ASSERT(current_subpath_odom.size() >= 2);
    const Waypoint& first = current_subpath_odom.at(0);
    const Waypoint& second = current_subpath_odom.at(1);

    Eigen::Vector2d first_p = first;
    Eigen::Vector2d second_p = second;

    Eigen::Vector2d dir = second_p - first_p;
    double dir_angle = std::atan2(dir(1), dir(0));
    double delta = MathHelper::AngleDelta(dir_angle, first.orientation);
    bool forward = std::abs(delta) < M_PI / 2.0;

    ROS_INFO_STREAM(dir_angle << " / " << first.orientation << " / " << second.orientation << " / " << delta << " -> " << forward);

    int look_back = look_back_distance / path_resolution;
    int look_forward = look_forward_distance / path_resolution;;
    if(isPathObstructed(current_subpath_map, closest_wp_map_index - look_back, closest_wp_map_index + look_forward, radius)) {
        ROS_WARN("new avoiding plan");
        try {
            if(forward) {
                avoiding_path_map_ = calculateAvoidingPath<AStarPatsyForward>(forward, odom_pose_v, odom_to_map);
            } else {
                avoiding_path_map_ = calculateAvoidingPath<AStarPatsyReverse>(forward, odom_pose_v, odom_to_map);
            }
            local_wps = transformPath(avoiding_path_map_, odom_to_map);
            avoiding = true;

        } catch(const std::runtime_error& e) {
            ROS_ERROR_STREAM_THROTTLE(1, "planning failed: " << e.what());
            local_wps.clear();
            return true;
        }


    } else {
        ROS_INFO("continue");
        Eigen::Vector3d pose_odom = follower_.getRobotPose();
        std::size_t closest_current_subpath_index = findClosestWaypoint(pose_odom, current_subpath_odom);
        local_wps.assign(current_subpath_odom.begin() + closest_current_subpath_index, current_subpath_odom.end());

        if(avoiding) {
//            if(local_wps.size() < 15 || closest_current_subpath_index > 30) {
                SubPath waypoints_odom = transformPath(waypoints_map, odom_to_map);
                std::size_t closest_local_wp_odom_index = findClosestWaypoint(pose_odom, waypoints_odom);
                const Waypoint& closest_wp_odom = waypoints_odom.at(closest_local_wp_odom_index);
                double distance = std::hypot(closest_wp_odom.x -pose_odom(0), closest_wp_odom.y - pose_odom(1));

                ROS_INFO_STREAM("avoiding. distance: " << distance);

                geometry_msgs::Pose avoiding_closest_pose;
                avoiding_closest_pose.position.x = closest_wp_odom.x;
                avoiding_closest_pose.position.y = closest_wp_odom.y;
                avoiding_closest_pose.position.z = 0;
                avoiding_closest_pose.orientation = tf::createQuaternionMsgFromYaw(closest_wp_odom.orientation);
                follower_.getVisualizer().drawArrow("/odom", 0, avoiding_closest_pose, "avoiding_pose", 1.0, 0.0, 0.0, 0.0, 3.0);


                Eigen::Vector3d pose_map = transformPose(pose_odom, map_to_odom);
                std::size_t closest_local_wp_index = findClosestWaypoint(pose_map, waypoints_map);

                const Waypoint& first_wp = waypoints_map.at(std::max(0, (int) closest_local_wp_odom_index - look_back));
                avoiding_closest_pose.position.x = first_wp.x;
                avoiding_closest_pose.position.y = first_wp.y;
                avoiding_closest_pose.orientation = tf::createQuaternionMsgFromYaw(first_wp.orientation);
                follower_.getVisualizer().drawArrow("/odom", 1, avoiding_closest_pose, "avoiding_pose", 0.0, 1.0, 0.0, 0.0, 2.0);

                const Waypoint& last_wp = waypoints_map.at(std::min(waypoints_map.size() - 1, closest_local_wp_odom_index + look_forward));
                avoiding_closest_pose.position.x = last_wp.x;
                avoiding_closest_pose.position.y = last_wp.y;
                avoiding_closest_pose.orientation = tf::createQuaternionMsgFromYaw(last_wp.orientation);
                follower_.getVisualizer().drawArrow("/odom", 2, avoiding_closest_pose, "avoiding_pose", 0.0, 0.0, 1.0, 0.0, 2.0);

                if(distance < 0.33 && !isPathObstructed(waypoints_map, closest_local_wp_index - look_back, closest_local_wp_index + look_forward, radius)) {
                    ROS_INFO_STREAM("switch back to planned path");
                    avoiding = false;
                    local_wps.assign(waypoints_odom.begin() + closest_local_wp_index, waypoints_odom.end());

                } else if(closest_current_subpath_index + look_forward / 2 > current_subpath_odom.size()) {
                    ROS_INFO_STREAM("plan new avoidance path, distance to global path: " << distance);
                    try {
                        if(forward) {
                            avoiding_path_map_ = calculateAvoidingPath<AStarPatsyForward>(forward, odom_pose_v, odom_to_map);
                        } else {
                            avoiding_path_map_ = calculateAvoidingPath<AStarPatsyReverse>(forward, odom_pose_v, odom_to_map);
                        }
                        local_wps = transformPath(avoiding_path_map_, odom_to_map);

                    } catch(const std::runtime_error& e) {
                        ROS_ERROR("planning failed");
                        local_wps.clear();
                        return true;
                    }
                } else {
                    ROS_INFO_STREAM("keep current path " << closest_current_subpath_index + look_forward / 2 << " > " << current_subpath_odom.size());
                }
            }
//        }
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

template <typename Algorithm>
SubPath LocalPlannerAStarDynamic::calculateAvoidingPath(bool forward, const Eigen::Vector3d& odom_pose_v, const tf::Transform& odom_to_map)
{
    tf::Pose odom_pose(tf::createQuaternionFromYaw(odom_pose_v(2)), tf::Vector3(odom_pose_v(0), odom_pose_v(1), 0.0));
    tf::Pose map_pose = odom_to_map.inverse() * odom_pose;

    // find path
    Pose2d pose_cell = convertToCell(map_info.get(), map_pose);
    DirectionalNode<SteeringNode<HeuristicNode<Pose2d>>> start_config;
    DirectionalNode<SteeringNode<HeuristicNode<Pose2d>>>::init(start_config, start_config);
    start_config.x = pose_cell.x;
    start_config.y = pose_cell.y;
    start_config.theta = pose_cell.theta;

    if(!transformer_.canTransform("/base_link", "/trailer_link", ros::Time(0))) {
        ROS_ERROR_STREAM("cannot get latest transform between base_link and trailer_link") ;
        return {};
    }
    tf::StampedTransform base_link_to_trailer_link;
    transformer_.lookupTransform("/base_link", "/trailer_link", ros::Time(0), base_link_to_trailer_link);
    start_config.steering_angle = -tf::getYaw(base_link_to_trailer_link.getRotation()) / M_PI * 180.;

    ROS_WARN_STREAM("theta is " << start_config.theta );
    ROS_WARN_STREAM("starting with a steering angle of " << start_config.steering_angle );

    Algorithm algo;
    algo.setMap(map_info.get());
    algo.setTimeLimit(2.0);

    Stopwatch sw;
    sw.restart();

    algo.setPathCandidateCallback([this](const typename Algorithm::PathT& path) {
        std::cout << "path candidate found: " << path.size() << std::endl;
        return false;
    });

    NearPathTest<Algorithm> goal_test_forward(*this, waypoints_map, algo, start_config, map, map_info.get());


    Pose2d hgoal = *goal_test_forward.getHeuristicGoal();
    geometry_msgs::Pose arrow;
    map_info->cell2pointSubPixel(hgoal.x, hgoal.y, arrow.position.x, arrow.position.y);
    arrow.position.z = 0;
    arrow.orientation = tf::createQuaternionMsgFromYaw(hgoal.theta);

    follower_.getVisualizer().drawArrow("/map", 0, arrow, "heuristic_goal", 0.0, 0.0, 0.0, 0.0);

    ROS_INFO_STREAM("start: " << map_pose.getOrigin().getX() << ", " << map_pose.getOrigin().getY() << " -> node " << start_config.x << " / " << start_config.y << " / " << start_config.theta);
    ROS_INFO_STREAM("heuristic goal: " << arrow.position.x << " / " << arrow.position.y <<  " / " << hgoal.theta);
    ROS_INFO_STREAM(" -> arrow = " << arrow);
    ROS_INFO_STREAM("Search preparation took " << sw.usElapsed()/1000.0 << "ms");

    sw.restart();
    typename Algorithm::PathT path;
    try {
        path = algo.findPathWithStartConfiguration(start_config, goal_test_forward);
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM("path search failed: " << e.what());
        throw std::runtime_error("found no local path");
    }
    ROS_INFO_STREAM("Search took " << sw.usElapsed()/1000.0 << "ms for " << algo.getExpansions() << " expansions (" << algo.getMultiExpansions() << " multiply expanded and " << algo.getTouchedNodes() << " touched nodes).");

    if(path.empty()) {
        ROS_ERROR("found no local path");
        throw std::runtime_error("found no local path");
    }

    ROS_WARN_STREAM("found a local path of length " << path.size());

    SubPath avoiding_path_map;

    for (std::size_t i = 0, total = path.size(); i < total; ++i) {
        auto pt = path.at(i);

        Waypoint wp_map;
        map_info->cell2pointSubPixel(pt.x, pt.y, wp_map.x, wp_map.y);
        if(forward) {
            wp_map.orientation = pt.theta;
        } else {
            wp_map.orientation = MathHelper::AngleClamp(M_PI + pt.theta);
        }
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


    map_info_static.reset(new CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width));

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
    map_info_static->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));
}

bool LocalPlannerAStarDynamic::integrateObstacles()
{
    double max_range = 8.0;

    tf::StampedTransform trafo;
    ros::Time time;
    time.fromNSec(obstacle_cloud_->header.stamp * 1e3);
    if(!transformer_.waitForTransform("/map", obstacle_cloud_->header.frame_id, time, ros::Duration(0.1))) {
        ROS_ERROR_STREAM("cannot transform the obstacle cloud to the map frame at time " << time << " (now is " << ros::Time::now() << ")") ;
        return false;
    }
    transformer_.lookupTransform("/map", obstacle_cloud_->header.frame_id, time, trafo);

    int OBSTACLE = 100;

    map_info.reset(new CollisionGridMap2d(*map_info_static));

    nav_msgs::OccupancyGrid local_map = map;
    local_map.header.frame_id = "/map";

    unsigned w = map.info.width;

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obstacle_cloud_->begin(); it != obstacle_cloud_->end(); ++it) {
        const pcl::PointXYZ& pt = *it;

        tf::Vector3 pt_cloud(pt.x, pt.y, pt.z);

        double dist = std::hypot(pt.x, pt.y);
        if(dist > max_range) {
            continue;
        }

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
