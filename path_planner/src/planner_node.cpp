/// HEADER
#include "planner_node.h"

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>

using namespace lib_path;

Planner::Planner()
    : nh("~"), map_info(NULL)
{
    std::string target_topic = "/move_base_simple/goal";
    nh.param("target_topic", target_topic, target_topic);

    goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (target_topic, 2, boost::bind(&Planner::updateGoalCallback, this, _1));

    nh.param("use_map_topic", use_map_topic_, false);
    use_map_service_ = !use_map_topic_;

    if(use_map_topic_) {
        std::string map_topic = "/map";
        nh.param("map_topic",map_topic, map_topic);
        map_sub = nh.subscribe<nav_msgs::OccupancyGrid>
                (map_topic, 1, boost::bind(&Planner::updateMapCallback, this, _1));

    } else {
        std::string map_service = "/dynamic_map";
        nh.param("map_service",map_service, map_service);
        map_service_client = nh.serviceClient<nav_msgs::GetMap>
                (map_service);
    }

    base_frame_ = "/base_link";
    nh.param("base_frame", base_frame_, base_frame_);

    path_publisher = nh.advertise<nav_msgs::Path> ("/path", 10);
    raw_path_publisher = nh.advertise<nav_msgs::Path> ("/path_raw", 10);
}

Planner::~Planner()
{

}

void Planner::updateMapCallback (const nav_msgs::OccupancyGridConstPtr &map) {
    updateMap(*map);
}
void Planner::updateMap (const nav_msgs::OccupancyGrid &map) {
    unsigned w = map.info.width;
    unsigned h = map.info.height;

    bool replace = map_info == NULL ||
            map_info->getWidth() != w ||
            map_info->getHeight() != h;

    if(replace){
        if(map_info != NULL) {
            delete map_info;
        }
        map_info = new lib_path::SimpleGridMap2d(map.info.width, map.info.height, map.info.resolution);
    }

    /// Map data
    /// -1: unknown -> 0
    /// 0:100 probabilities -> 1 - 101
    std::vector<uint8_t> data(w*h);
    int i = 0;
    for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
        data[i++] = *it;
    }

    map_info->set(data, w, h);
    map_info->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));
    map_info->setLowerThreshold(10);
    map_info->setUpperThreshold(70);
}

void Planner::updateGoalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
    ROS_INFO("got goal");
    if(use_map_service_) {
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            updateMap(map_service.response.map);
        }
    }

    if(map_info == NULL) {
        ROS_WARN("request for path planning, but no map there yet...");
        return;
    }

    ROS_INFO("starting search");
    lib_path::Pose2d from_world;
    lib_path::Pose2d to_world;

    tf::StampedTransform trafo;
    tfl.lookupTransform("/map", base_frame_, ros::Time(0), trafo);

    from_world.x = trafo.getOrigin().x();
    from_world.y = trafo.getOrigin().y();
    from_world.theta = tf::getYaw(trafo.getRotation());

    ROS_WARN_STREAM("theta=" << from_world.theta);

    to_world.x = goal->pose.position.x;
    to_world.y = goal->pose.position.y;
    to_world.theta = tf::getYaw(goal->pose.orientation);

    lib_path::Pose2d from_map;
    lib_path::Pose2d to_map;

    {
        unsigned fx, fy;
        map_info->point2cell(from_world.x, from_world.y, fx, fy);
        from_map.x = fx;
        from_map.y = fy;
        from_map.theta = from_world.theta;
    }

    ROS_WARN_STREAM("res=" << map_info->getResolution());
    {
        unsigned tx, ty;
        map_info->point2cell(to_world.x, to_world.y, tx, ty);
        to_map.x = tx;
        to_map.y = ty;
        to_map.theta = to_world.theta;
    }

    plan(*goal, from_world, to_world, from_map, to_map);
}
