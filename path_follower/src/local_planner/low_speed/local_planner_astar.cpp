/// HEADER
#include <path_follower/local_planner/low_speed/local_planner_astar.h>

/// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/factory/local_planner_factory.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/obstacle_cloud.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <cslibs_path_planning/common/CollisionGridMap2d.h>
#include <cslibs_path_planning/generic/Algorithms.hpp>
#include <cslibs_path_planning/common/SimpleGridMap2d.h>
#include <cslibs_path_planning/generic/DynamicSteeringNeighborhood.h>
#include <cslibs_path_planning/generic/SteeringNode.hpp>
#include <cslibs_path_planning/generic/Heuristics.hpp>

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>

REGISTER_LOCAL_PLANNER(low_speed::LocalPlannerAStar, AStar);

using namespace lib_path;
using namespace low_speed;


typedef
AStarDynamicSearch<DynamicSteeringNeighborhood, NoExpansion, Pose2d, GridMap2d, 500 >
PathPlanningAlgorithm;

LocalPlannerAStar::LocalPlannerAStar()
    : pnh_("~")
{
    if(nh_.hasParam("path_planner/size/forward")) {
        nh_.param("path_planner/size/forward", size_forward, 0.15);
    } else {
        pnh_.param("size/forward", size_forward, 0.15);
    }
    if(nh_.hasParam("path_planner/size/backward")) {
        nh_.param("path_planner/size/backward", size_backward, -0.8);
    } else {
        pnh_.param("size/backward", size_backward, -0.8);
    }
    if(nh_.hasParam("path_planner/size/width")) {
        nh_.param("path_planner/size/width", size_width, 0.75);
    } else {
        pnh_.param("size/width", size_width, 0.75);
    }

    ROS_INFO_STREAM("local planner dimensions (f/b/w) : " << size_forward << " / " << size_backward << " / " << size_width);

    local_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
}

Path::Ptr LocalPlannerAStar::updateLocalPath()
{
    ros::Time now = ros::Time::now();

    update_interval_ = ros::Duration(1.0);

    // only calculate a new local path, if enough time has passed.
    if(last_update_ + update_interval_ < now) {
        if(global_path_.n() == 0) {
            ROS_WARN("cannot calculate local path, global path is empty");
            return nullptr;
        }
        return doUpdateLocalPath();

    } else {
        return nullptr;
    }
}

void LocalPlannerAStar::setParams(const LocalPlannerParameters& opt){
}

void LocalPlannerAStar::setVelocity(geometry_msgs::Twist::_linear_type vector){
    (void) vector;
}

void LocalPlannerAStar::setVelocity(double velocity){
    (void) velocity;
}

Path::Ptr LocalPlannerAStar::doUpdateLocalPath()
{
    ros::Time now = ros::Time::now();

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();

    // transform the waypoints from world to odom
    Eigen::Vector3d pose = pose_tracker_->getRobotPose();
    int n = global_path_.n();
    double dist_to_last_pt = std::hypot(pose(0) - global_path_.p(n-1),
                                        pose(1) - global_path_.q(n-1));

    // find a path that starts at the robot's pose

    Path::Ptr local_wps = std::make_shared<Path>(odom_frame);
    //    local_wps.push_back(start_pose);

    //    double r = 1.0;
    //    Waypoint test(pose(0) + std::cos(pose(2)) * r, pose(1) + std::sin(pose(2)) * r, pose(2));
    //    local_wps.push_back(test);

    //    return setPath(odom_frame, local_wps, now);



    // calculate the corrective transformation to map from world coordinates to odom

    updateMap();
    map_info = map_info_static;

    if(!integrateObstacles()) {
        return {};
    }

    DynamicSteeringNeighborhood::goal_angle_threshold = pnh_.param("planner/goal_angle_threshold", 15.0) / 180. * M_PI;

    DynamicSteeringNeighborhood::allow_forward = true;
    DynamicSteeringNeighborhood::allow_backward = true;

    DynamicSteeringNeighborhood::MAX_STEER_ANGLE = pnh_.param("planner/ackermann_max_steer_angle", 45);
    DynamicSteeringNeighborhood::STEER_DELTA = pnh_.param("planner/ackermann_steer_delta", 15);
    DynamicSteeringNeighborhood::steer_steps = pnh_.param("planner/ackermann_steer_steps", 2);
    DynamicSteeringNeighborhood::LA = pnh_.param("planner/ackermann_la", 0.6);

    if(dist_to_last_pt > 2 * DynamicSteeringNeighborhood::LA) {
        DynamicSteeringNeighborhood::goal_dist_threshold = pnh_.param("planner/goal_dist_threshold", 0.25);
    } else {
        DynamicSteeringNeighborhood::goal_dist_threshold = pnh_.param("planner/goal_dist_threshold", 0.5);
    }

    bool final_approach = dist_to_last_pt < 4 * DynamicSteeringNeighborhood::LA;

    try {
        if(final_approach) {
            DynamicSteeringNeighborhood::reversed = true;
            local_wps = calculateFinalAvoidingPath(odom_frame);
        } else {
            DynamicSteeringNeighborhood::reversed = false;
            local_wps = calculateAvoidingPath(odom_frame);
        }

    } catch(const std::runtime_error& e) {
        local_map_pub_.publish(local_map);

        ROS_ERROR_STREAM_THROTTLE(1, "planning failed: " << e.what());
        return {};
    }

    local_map_pub_.publish(local_map);

    if(local_wps->empty()) {
        return {};
    }

    setPath(local_wps, now);

    return local_wps;
}

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


struct NearPathTest
{
    NearPathTest(const LocalPlannerAStar& parent, const SubPath& odom_path,
                 PathPlanningAlgorithm& algo, const Pose2d& start_cell, const nav_msgs::OccupancyGrid& map,
                 nav_msgs::OccupancyGrid& local_map, const SimpleGridMap2d* map_info)
        : parent(parent), odom_path(odom_path),
          algo(algo), map(map), local_map(local_map), map_info(map_info),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          start_cell(start_cell),

          candidates(0)
    {
        min_dist = DynamicSteeringNeighborhood::LA;
        desired_dist = 2.0 * DynamicSteeringNeighborhood::LA;

        updateHeuristicGoal();
    }

    void updateHeuristicGoal()
    {
        if(odom_path.empty()) {
            return;
        }

        double mx,my;
        map_info->cell2pointSubPixel(start_cell.x, start_cell.y, mx, my);
        Eigen::Vector3d start_pose(mx,my, start_cell.theta);

        std::size_t start_index = findClosestWaypoint(start_pose, odom_path);

        double dist_accum = 0.0;
        const Waypoint* last_wp = &odom_path.at(start_index);
        for(std::size_t i = start_index, n = odom_path.size(); i < n; ++i) {
            const Waypoint& wp = odom_path[i];
            dist_accum += wp.distanceTo(*last_wp);

            if(dist_accum >= desired_dist * 2) {
                map_info->point2cellSubPixel(wp.x, wp.y, heuristic_goal.x,heuristic_goal.y);
                heuristic_goal.theta = wp.orientation;
                return;
            }

            last_wp = &wp;
        }

        const Waypoint& wp = odom_path.back();
        map_info->point2cellSubPixel(wp.x, wp.y, heuristic_goal.x,heuristic_goal.y);
        heuristic_goal.theta = wp.orientation;
    }

    void reset()
    {
        candidates = 0;
    }

    bool terminate(const typename PathPlanningAlgorithm::NodeT* node) const
    {
//        if(!node->forward) {
//            ROS_WARN_STREAM("backwards, depth: " << node->depth << ", d:" << node->distance << ", h:" << node->h);
//        } else {
//            ROS_INFO_STREAM("forwards, depth: " << node->depth << ", d:" << node->distance << ", h:" << node->h);
//        }
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
        static const double angle_tolerance = M_PI / 4;

        double closest_dist = std::numeric_limits<double>::infinity();
        for(const Waypoint& wp : odom_path) {
            double dtheta = MathHelper::AngleDelta(wp.orientation, node->theta);
            if(std::abs(dtheta) > angle_tolerance) {
                continue;
            }

            double dist = std::hypot(wp.x - wx, wp.y - wy);
            if(dist < closest_dist) {
                closest_dist = dist;
            }
        }


        //        local_map.data.at(y * w + x) = closest_dist * 50.0;

        //        for(const Waypoint& wp : odom_path) {
        //            unsigned int x,y;
        //            if(map_info->point2cell(wp.x, wp.y, x, y)) {
        //                local_map.data.at(y * w + x) = 100;
        //            }
        //        }

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

        return candidates > 10;
    }

    const Pose2d* getHeuristicGoal() const
    {
        return &heuristic_goal;
    }

    const LocalPlannerAStar& parent;
    const SubPath odom_path;

    PathPlanningAlgorithm& algo;
    const nav_msgs::OccupancyGrid& map;
    nav_msgs::OccupancyGrid& local_map;
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


void LocalPlannerAStar::reset()
{
    last_update_ = ros::Time(0);
}

void LocalPlannerAStar::setGlobalPath(Path::Ptr path)
{
    reset();
    AbstractLocalPlanner::setGlobalPath(path);
}


SubPath LocalPlannerAStar::transformPath(const SubPath &path_map, const tf::Transform& trafo)
{
    SubPath avoiding_path_odom;
    for (std::size_t i = 0, total = path_map.size(); i < total; ++i) {
        avoiding_path_odom.push_back(transformWaypoint(path_map[i], trafo));
    }
    smoothAndInterpolate(avoiding_path_odom);

    return avoiding_path_odom;
}

Waypoint LocalPlannerAStar::transformWaypoint(const Waypoint &wp, const tf::Transform& trafo)
{
    tf::Pose p(tf::createQuaternionFromYaw(wp.orientation), tf::Vector3(wp.x, wp.y, 0.0));
    tf::Pose tp = trafo * p;

    Waypoint wp_trans;
    wp_trans.x = tp.getOrigin().x();
    wp_trans.y = tp.getOrigin().y();
    wp_trans.orientation = tf::getYaw(tp.getRotation());

    return wp_trans;
}

namespace {
Path::Ptr convertPath(typename PathPlanningAlgorithm::PathT path,

                      lib_path::CollisionGridMap2d* map_info,
                      const std::string &frame)
{
    Path::Ptr avoiding_path_map = std::make_shared<Path>(frame);

    if(path.size() > 0) {
        std::vector<SubPath> paths;

        // insert a first path
        paths.emplace_back();
        SubPath* current_subpath = &paths.back();
        current_subpath->forward = path.front().forward;

        Waypoint last_wp_map;
        for (auto pt : path) {
            Waypoint wp_map;
            map_info->cell2pointSubPixel(pt.x, pt.y, wp_map.x, wp_map.y);
            wp_map.orientation = pt.theta;

            if(pt.forward != current_subpath->forward) {

                // when the direction changes: add a new path segment
                if(!current_subpath->empty()) {
                    paths.emplace_back();
                    current_subpath = &paths.back();
                }
                current_subpath->forward = pt.forward;

                current_subpath->push_back(last_wp_map);
            }

            current_subpath->push_back(wp_map);

            last_wp_map = wp_map;
        }

        for(SubPath& subpath : paths) {
            AbstractLocalPlanner::smoothAndInterpolate(subpath);
        }

        avoiding_path_map->setPath(paths);
    }

    return avoiding_path_map;
}
}

Path::Ptr LocalPlannerAStar::calculateAvoidingPath(const std::string &frame)
{
    DirectionalNode<SteeringNode<HeuristicNode<Pose2d>>> start_config;
    DirectionalNode<SteeringNode<HeuristicNode<Pose2d>>>::init(start_config, start_config);

    // start at the robot pose
    Eigen::Vector3d odom_pose_v = pose_tracker_->getRobotPose();
    tf::Pose odom_pose(tf::createQuaternionFromYaw(odom_pose_v(2)),
                       tf::Vector3(odom_pose_v(0), odom_pose_v(1), 0.0));

    Pose2d pose_cell = convertToCell(map_info.get(), odom_pose);


    start_config.x = pose_cell.x;
    start_config.y = pose_cell.y;
    start_config.theta = pose_cell.theta;

    start_config.steering_angle = 0; // TODO

    PathPlanningAlgorithm algo;
    algo.setMap(map_info.get());
    algo.setTimeLimit(1.0);

    Stopwatch sw;
    sw.restart();

    algo.setPathCandidateCallback([](const typename PathPlanningAlgorithm::PathT& path) {
        return false;
    });

    SubPath map_path = global_path_;
    //    for(Waypoint& wp : map_path) {
    //        unsigned wpx, wpy;
    //        map_info->point2cell(wp.x, wp.y, wpx, wpy);
    //        wp.x = wpx;
    //        wp.y = wpy;
    //    }

    NearPathTest goal_test_forward(*this, map_path, algo, start_config, map, local_map, map_info.get());


    Pose2d hgoal = *goal_test_forward.getHeuristicGoal();
    geometry_msgs::Pose arrow;
    map_info->cell2pointSubPixel(hgoal.x, hgoal.y, arrow.position.x, arrow.position.y);
    arrow.position.z = 0;
    arrow.orientation = tf::createQuaternionMsgFromYaw(hgoal.theta);

    Visualizer::getInstance()->drawArrow("map", 0, arrow, "heuristic_goal", 0.0, 0.0, 0.0, 0.0);

    SearchOptions search_options;
    search_options.penalty_backward = 1.0;// 1.1;
    search_options.penalty_turn = 5.0;//5.0;
    search_options.oversearch_distance = 0.0;

    sw.restart();
    typename PathPlanningAlgorithm::PathT path;
    try {
        path = algo.findPathWithStartConfiguration(start_config, goal_test_forward, search_options);
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM("path search failed: " << e.what());
        throw std::runtime_error("found no local path");
    }
    ROS_INFO_STREAM("Search took " << sw.usElapsed()/1000.0 << "ms for " << algo.getExpansions() << " expansions (" << algo.getMultiExpansions() << " multiply expanded and " << algo.getTouchedNodes() << " touched nodes).");

    if(path.empty()) {
        ROS_ERROR("found no local path");
        throw std::runtime_error("found no local path");
    }

    return convertPath(path, map_info.get(), frame);
}


Path::Ptr LocalPlannerAStar::calculateFinalAvoidingPath(const std::string &frame)
{
    Pose2d start_config, goal_config;

    // start at the robot pose
    Eigen::Vector3d odom_pose_v = pose_tracker_->getRobotPose();
    tf::Pose odom_pose(tf::createQuaternionFromYaw(odom_pose_v(2)),
                       tf::Vector3(odom_pose_v(0), odom_pose_v(1), 0.0));

    Pose2d start_cell = convertToCell(map_info.get(), odom_pose);

    start_config.x = start_cell.x;
    start_config.y = start_cell.y;
    start_config.theta = start_cell.theta;

    // end at the last path pose
    int n = global_path_.n();
    ROS_ASSERT(n > 1);
    double theta = global_path_.theta_p(n-1);
    theta = std::atan2(global_path_.q(n-1) - global_path_.q(n-2),
                       global_path_.p(n-1) - global_path_.p(n-2));
    tf::Pose last_pose (tf::createQuaternionFromYaw(theta),
                        tf::Vector3(global_path_.p(n-1), global_path_.q(n-1), 0.0));

    Pose2d goal_cell = convertToCell(map_info.get(), last_pose);

    goal_config.x = goal_cell.x;
    goal_config.y = goal_cell.y;
    goal_config.theta = goal_cell.theta;

    // find the path
    PathPlanningAlgorithm algo;
    algo.setMap(map_info.get());
    algo.setTimeLimit(2.0);

    Stopwatch sw;
    sw.restart();

    SearchOptions search_options;
    search_options.penalty_backward = 1.0;
    search_options.penalty_turn = 2.0;
    search_options.oversearch_distance = 0.0;

    // debug publishing
    geometry_msgs::Pose goal_pose;
    map_info->cell2point(goal_config.x, goal_config.y, goal_pose.position.x, goal_pose.position.y);
    goal_pose.orientation = tf::createQuaternionMsgFromYaw(goal_config.theta);
    Visualizer::getInstance()->drawArrow(pose_tracker_->getFixedFrameId(), 0, goal_pose, "local_planner", 0,1,0, -1);

    typename PathPlanningAlgorithm::PathT path;
    try {
        path = algo.findPath(start_config, goal_config,
                             search_options);
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM("path search failed: " << e.what());
        throw std::runtime_error("found no local path");
    }
    ROS_INFO_STREAM("Search took " << sw.usElapsed()/1000.0 << "ms for " << algo.getExpansions() << " expansions (" << algo.getMultiExpansions() << " multiply expanded and " << algo.getTouchedNodes() << " touched nodes).");

    if(path.empty()) {
        ROS_ERROR("found no local path");
        throw std::runtime_error("found no local path");
    }

    return convertPath(path, map_info.get(), frame);
}

void LocalPlannerAStar::updateMap()
{
    map.info.width = 500;
    map.info.height = 500;
    map.info.resolution = 0.1;

    map.info.origin.position.x = -(map.info.width / 2.0 * map.info.resolution);
    map.info.origin.position.y = -(map.info.height / 2.0 * map.info.resolution);
    map.info.origin.orientation.w = 1;

    map_info_static.reset(new CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width));

    /// Map data
    /// -1: unknown -> 0
    /// 0:100 probabilities -> 1 - 100
    std::vector<uint8_t> data(map.info.width * map.info.height, 1);

    map.data.resize(map.info.width * map.info.height, 1);

    map_info_static->setLowerThreshold(50);
    map_info_static->setUpperThreshold(70);
    map_info_static->setNoInformationValue(-1);

    map_info_static->set(data, map.info.width, map.info.height, 0.0);
    map_info_static->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));
    map_info_static->setResolution(map.info.resolution);
}

bool LocalPlannerAStar::integrateObstacles()
{
    std::string fixed_frame = pose_tracker_->getFixedFrameId();
    std::string obstacle_frame = obstacle_cloud_->getFrameId();

    if(fixed_frame != obstacle_frame) {
        throw std::runtime_error("obstacles are not represented in the fixed frame!");
    }


    int OBSTACLE = 100;

    map_info.reset(new CollisionGridMap2d(*map_info_static));

    local_map = map;
    local_map.header.frame_id = fixed_frame;

    unsigned w = map.info.width;

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = obstacle_cloud_->cloud->begin(); it != obstacle_cloud_->cloud->end(); ++it) {
        const pcl::PointXYZ& pt = *it;

        unsigned int x,y;
        if(map_info->point2cell(pt.x, pt.y, x, y)) {
            map_info->setValue(x,y, OBSTACLE);

            for(int dy = -1; dy <= 1; ++dy) {
                for(int dx = -1; dx <= 1; ++dx) {
                    int xx = x+dx;
                    int yy = y+dy;
                    if(map_info->isInMap(xx, yy)) {
                        local_map.data.at(yy * w + xx) = 100;
                    }
                }
            }
        }
    }

    return true;
}
