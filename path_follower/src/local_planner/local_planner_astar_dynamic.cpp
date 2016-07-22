/// HEADER
#include <path_follower/local_planner/local_planner_astar_dynamic.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <utils_path/common/CollisionGridMap2d.h>
#include <utils_path/generic/Algorithms.hpp>
#include <utils_path/common/SimpleGridMap2d.h>

/// SYSTEM
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <ros/console.h>

namespace {


template <typename Algorithm>
struct NearPathTest
{
    NearPathTest(const LocalPlannerAStarDynamic& parent, Algorithm& algo, const Eigen::Vector3d& start, const nav_msgs::OccupancyGrid& map, const lib_path::SimpleGridMap2d* map_info)
        : parent(parent),
          algo(algo), map(map), map_info(map_info),
          res(map.info.resolution),
          ox(map.info.origin.position.x),
          oy(map.info.origin.position.y),
          w(map.info.width),
          h(map.info.height),

          start(start),

          candidates(0)
    {
    }

    void reset()
    {
        candidates = 0;
    }

    bool terminate(const typename Algorithm::NodeT* node) const
    {
        double wx, wy;
        map_info->cell2pointSubPixel(node->x,node->y, wx, wy);

        double dist = std::hypot(wx - start(0), wy - start(1));
        double min_dist = 3.0;// / map_info->getResolution();

        if(min_dist > 0.0 && dist < min_dist) {
            return false;
        }

        int x = (-ox + wx) / res;
        int y = (-oy + wy) / res;

        if(x < 0 || x >= w || y < 0 || y >= h) {
            return false;
        }

        algo.addGoalCandidate(node, dist);
        ++candidates;

        return candidates > 4;
    }

    const lib_path::Pose2d* getHeuristicGoal() const
    {
        return NULL;
    }

    const LocalPlannerAStarDynamic& parent;

    Algorithm& algo;
    const nav_msgs::OccupancyGrid& map;
    const lib_path::SimpleGridMap2d * map_info;

    double res;
    double ox;
    double oy;
    int w;
    int h;

    Eigen::Vector3d start;

    mutable int candidates;
};


lib_path::Pose2d  convertToMap(const lib_path::SimpleGridMap2d * map_info, const Eigen::Vector3d& pt)
{
    lib_path::Pose2d res;
    unsigned tmpx, tmpy;
    map_info->point2cell(pt(0), pt(1), tmpx, tmpy);
    res.x = tmpx;
    res.y = tmpy;
    res.theta = pt(2);// - map_info->getRotation();
    return res;
}

}

LocalPlannerAStarDynamic::LocalPlannerAStarDynamic(PathFollower &follower,
                                                   tf::Transformer& transformer,
                                                   const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),

      pnh_("~")
{
    pnh_.param("size/forward", size_forward, 0.4);
    pnh_.param("size/backward", size_backward, -0.6);
    pnh_.param("size/width", size_width, 0.5);

    std::string map_service = "/static_map";
    pnh_.param("map_service",map_service, map_service);

    map_service_client_ = pnh_.serviceClient<nav_msgs::GetMap> (map_service);

    local_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1, true);
}

bool LocalPlannerAStarDynamic::algo(Eigen::Vector3d& pose, SubPath& local_wps,
                                    const std::vector<Constraint::Ptr>& constraints,
                                    const std::vector<Scorer::Ptr>& scorer,
                                    const std::vector<bool>& fconstraints,
                                    const std::vector<double>& wscorer,
                                    int& nnodes){

    // get map
    nav_msgs::GetMap map_service;
    if(!map_service_client_.exists()) {
        map_service_client_.waitForExistence();
    }
    if(!map_service_client_.call(map_service)) {
        ROS_ERROR("map service lookup failed");
        return {};
    }

    const nav_msgs::OccupancyGrid& map =  map_service.response.map;

    unsigned w = map.info.width;
    unsigned h = map.info.height;


    std::shared_ptr<lib_path::SimpleGridMap2d> map_info;
    map_info.reset(new lib_path::CollisionGridMap2d(map.info.width, map.info.height, tf::getYaw(map.info.origin.orientation), map.info.resolution, size_forward, size_backward, size_width));

    std::vector<uint8_t> data(w*h);
    int i = 0;

    /// Map data
    /// -1: unknown -> 0
    /// 0:100 probabilities -> 1 - 100
    for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
        data[i++] = std::min(100, *it + 1);
    }

    map_info->setLowerThreshold(50);
    map_info->setUpperThreshold(70);
    map_info->setNoInformationValue(-1);

    map_info->set(data, w, h);
    map_info->setOrigin(lib_path::Point2d(map.info.origin.position.x, map.info.origin.position.y));

    // integrate cloud
    tf::StampedTransform trafo;
    ros::Time time;
    time.fromNSec(obstacle_cloud_->header.stamp);
    transformer_.lookupTransform("/map", obstacle_cloud_->header.frame_id, time, trafo);

    int OBSTACLE = 100;

    nav_msgs::OccupancyGrid local_map = map;
    local_map.header.frame_id = "/map";

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

    // find path
    lib_path::Pose2d pose_map = convertToMap(map_info.get(), pose);

    typedef lib_path::AStarSearch<lib_path::NonHolonomicNeighborhood<40, 160, lib_path::NonHolonomicNeighborhoodMoves::FORWARD, false>,
            lib_path::NoExpansion, lib_path::Pose2d, lib_path::GridMap2d, 500 > AStarPatsyForward;
    AStarPatsyForward algo_forward;
    algo_forward.setMap(map_info.get());

    NearPathTest<AStarPatsyForward> goal_test_forward(*this, algo_forward, pose, map, map_info.get());
    auto path_start = algo_forward.findPath(pose_map, goal_test_forward, 0);
    if(path_start.empty()) {
        ROS_ERROR("found no local path");
        return false;
    }

    ROS_WARN_STREAM("found a local path of length " << path_start.size());
    std::vector<HNode> nodes;
    nodes.resize(path_start.size());

    nnodes = nodes.size();

    for (std::size_t i = 1, total = path_start.size(); i < total; ++i) {
        auto pt = path_start.at(i);
        HNode* n = &nodes[i];

        double wx, wy;
        map_info->cell2pointSubPixel(pt.x, pt.y, wx, wy);

        ROS_INFO_STREAM("node " << wx << " / " <<wy << " / " << pt.theta);

        n->x = wx;
        n->x = wy;
        n->orientation = pt.theta;
        n->parent_ = nullptr;
    }

    for (std::size_t i = 1, total = path_start.size(); i < total; ++i) {
        HNode* n = &nodes[i];
        HNode* prev = &nodes[i-1];

        n->parent_ = prev;
        n->computeDiff();
    }
    HNode* obj = &nodes.back();

    processPath(obj, local_wps);

    ROS_WARN_STREAM("all done");
    return true;
}
