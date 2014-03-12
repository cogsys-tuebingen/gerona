/// HEADER
#include "planner_node.h"

/// PROJECT
#include <utils_path/common/CollisionGridMap2d.h>

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

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

        std::cout << "using map topic " << map_topic << std::endl;

    } else {
        std::string map_service = "/dynamic_map";
        nh.param("map_service",map_service, map_service);
        map_service_client = nh.serviceClient<nav_msgs::GetMap> (map_service);

        std::cout << "using map service " << map_service << std::endl;
    }


    nh.param("use_cost_map", use_cost_map_, false);
    if(use_cost_map_) {
        std::string costmap_service = "/dynamic_map/cost";
        nh.param("cost_map_service",costmap_service, costmap_service);
        cost_map_service_client = nh.serviceClient<nav_msgs::GetMap> (costmap_service);

        std::cout << "using cost map service " << costmap_service << std::endl;
    }

    viz_pub = nh.advertise<visualization_msgs::Marker>("/marker", 0);

    base_frame_ = "/base_link";
    nh.param("base_frame", base_frame_, base_frame_);


    nh.param("size/forward", size_forward, 0.4);
    nh.param("size/backward", size_backward, -0.6);
    nh.param("size/width", size_width, 0.5);

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

        map_info = new lib_path::CollisionGridMap2d(map.info.width, map.info.height, map.info.resolution, size_forward, size_backward, size_width);
    }

    bool use_unknown;
    nh.param("use_unknown_cells", use_unknown, false);

    std::vector<uint8_t> data(w*h);

    int i = 0;
    if(use_unknown) {
        /// Map data
        /// -1: unknown -> 0
        /// 0:100 probabilities -> 1 - 101
        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            data[i++] = *it + 1;
        }

    } else {
        /// Map data
        /// -1: unknown -> -1
        /// 0:100 probabilities -> 0 - 100
        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            data[i++] = *it;
        }
    }

    map_info->set(data, w, h);
    map_info->setOrigin(Point2d(map.info.origin.position.x, map.info.origin.position.y));
    map_info->setLowerThreshold(10);
    map_info->setUpperThreshold(70);
}

void Planner::visualizeOutline(const geometry_msgs::Pose& at, int id, const std::string &frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "planning/outline";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 0.75;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;


    tf::Pose fl_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, size_width / 2.0, 0.0));
    tf::Pose fr_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, -size_width / 2.0, 0.0));
    tf::Pose bl_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, size_width / 2.0, 0.0));
    tf::Pose br_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, -size_width / 2.0, 0.0));

    tf::Transform transform;
    tf::poseMsgToTF(at, transform);

    tf::Pose fl_ = transform * fl_base;
    tf::Pose fr_ = transform * fr_base;
    tf::Pose bl_ = transform * bl_base;
    tf::Pose br_ = transform * br_base;

    geometry_msgs::Point fl, fr, bl, br;
    fl.x = fl_.getOrigin().x();
    fl.y = fl_.getOrigin().y();
    fr.x = fr_.getOrigin().x();
    fr.y = fr_.getOrigin().y();
    bl.x = bl_.getOrigin().x();
    bl.y = bl_.getOrigin().y();
    br.x = br_.getOrigin().x();
    br.y = br_.getOrigin().y();


    marker.points.push_back(fl);
    marker.points.push_back(fr);
    marker.points.push_back(fr);
    marker.points.push_back(br);
    marker.points.push_back(br);
    marker.points.push_back(bl);
    marker.points.push_back(bl);
    marker.points.push_back(fl);

    viz_pub.publish(marker);
}

void Planner::visualizePath(const nav_msgs::Path &path)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "planning/steps";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    tf::Pose fl_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, size_width / 2.0, 0.0));
    tf::Pose fr_base(tf::createIdentityQuaternion(), tf::Vector3(size_forward, -size_width / 2.0, 0.0));
    tf::Pose bl_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, size_width / 2.0, 0.0));
    tf::Pose br_base(tf::createIdentityQuaternion(), tf::Vector3(size_backward, -size_width / 2.0, 0.0));

    for(unsigned i = 0; i < path.poses.size(); ++i) {
        const geometry_msgs::Pose& pose = path.poses[i].pose;
        tf::Transform transform;
        tf::poseMsgToTF(pose, transform);

        tf::Pose fl_ = transform * fl_base;
        tf::Pose fr_ = transform * fr_base;
        tf::Pose bl_ = transform * bl_base;
        tf::Pose br_ = transform * br_base;

        geometry_msgs::Point fl, fr, bl, br;
        fl.x = fl_.getOrigin().x();
        fl.y = fl_.getOrigin().y();
        fr.x = fr_.getOrigin().x();
        fr.y = fr_.getOrigin().y();
        bl.x = bl_.getOrigin().x();
        bl.y = bl_.getOrigin().y();
        br.x = br_.getOrigin().x();
        br.y = br_.getOrigin().y();


        marker.points.push_back(fl);
        marker.points.push_back(fr);
        marker.points.push_back(fr);
        marker.points.push_back(br);
        marker.points.push_back(br);
        marker.points.push_back(bl);
        marker.points.push_back(bl);
        marker.points.push_back(fl);
    }

    viz_pub.publish(marker);
}

void Planner::updateGoalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
    ROS_INFO("got goal");

    //  visualizeOutline();

    if(use_map_service_) {
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            updateMap(map_service.response.map);
        }
    }

    if(use_cost_map_) {
        nav_msgs::GetMap map_service;
        if(cost_map_service_client.call(map_service)) {
            cost_map = map_service.response.map;
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

    geometry_msgs::Pose pose;
    tf::poseTFToMsg(trafo, pose);
    visualizeOutline(pose, 0, "/map");
    visualizeOutline(goal->pose, 1, "/map");

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


void Planner::subdividePath(nav_msgs::Path& result, geometry_msgs::PoseStamped low, geometry_msgs::PoseStamped up, double max_distance) {
    double dx = low.pose.position.x - up.pose.position.x;
    double dy = low.pose.position.y - up.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if(distance > max_distance) {
        // split half way between the lower and the upper node
        geometry_msgs::PoseStamped halfway(low);
        halfway.pose.position.x += (up.pose.position.x - low.pose.position.x) / 2.0;
        halfway.pose.position.y += (up.pose.position.y - low.pose.position.y) / 2.0;

        double up_y = tf::getYaw(up.pose.orientation);
        double low_y = tf::getYaw(low.pose.orientation);

        halfway.pose.orientation = tf::createQuaternionMsgFromYaw((up_y + low_y) / 2.0);

        // first recursive descent in lower part
        subdividePath(result, low, halfway, max_distance);
        // then add the half way point
        result.poses.push_back(halfway);
        // then descent in upper part
        subdividePath(result, halfway, up, max_distance);
    }
}

nav_msgs::Path Planner::interpolatePath(const nav_msgs::Path& path, double max_distance) {
    unsigned n = path.poses.size();

    nav_msgs::Path result;
    result.header = path.header;

    if(n < 2) {
        return result;
    }

    result.poses.push_back(path.poses[0]);

    for(unsigned i = 1; i < n; ++i){
        const geometry_msgs::PoseStamped* current = &path.poses[i];

        // split the segment, iff it is to large
        subdividePath(result, result.poses.back(), *current, max_distance);

        // add the end of the segment (is not done, when splitting)
        result.poses.push_back(*current);
    }
    return result;
}

nav_msgs::Path Planner::smoothPath(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance) {
    // find segments
    nav_msgs::Path result;
    result.header = path.header;

    int n = path.poses.size();
    if(n < 2) {
        return result;
    }

    nav_msgs::Path current_segment;

    const geometry_msgs::PoseStamped * last_point = &path.poses[0];
    current_segment.poses.push_back(*last_point);

    for(int i = 1; i < n; ++i){
        const geometry_msgs::PoseStamped* current_point = &path.poses[i];

        // append to current segment
        current_segment.poses.push_back(*current_point);

        bool is_the_last_node = i == n-1;
        bool segment_ends_with_this_node = false;

        if(is_the_last_node) {
            // this is the last node
            segment_ends_with_this_node = true;

        } else {
            const geometry_msgs::PoseStamped* next_point = &path.poses[i+1];

            // if angle between last direction and next direction to large -> segment ends
            double diff_last_x = current_point->pose.position.x - last_point->pose.position.x;
            double diff_last_y = current_point->pose.position.y - last_point->pose.position.y;
            double last_angle = std::atan2(diff_last_y, diff_last_x);

            double diff_next_x = next_point->pose.position.x - current_point->pose.position.x;
            double diff_next_y = next_point->pose.position.y - current_point->pose.position.y;
            double next_angle = std::atan2(diff_next_y, diff_next_x);

            if(std::abs(MathHelper::AngleClamp(last_angle - next_angle)) > M_PI / 2.0) {
                // new segment!
                // current node is the last one of the old segment
                segment_ends_with_this_node = true;
            }
        }

        if(segment_ends_with_this_node) {
            nav_msgs::Path smoothed_segment = smoothPathSegment(current_segment, weight_data, weight_smooth, tolerance);
            result.poses.insert(result.poses.end(), smoothed_segment.poses.begin(), smoothed_segment.poses.end());

            current_segment.poses.clear();

            if(!is_the_last_node) {
                // begin new segment

                // current node is also the first one of the new segment
                current_segment.poses.push_back(*current_point);
            }
        }

        last_point = current_point;
    }

    return result;
}

Pose2d Planner::convert(const geometry_msgs::PoseStamped& rhs)
{
    return Pose2d(rhs.pose.position.x, rhs.pose.position.y, tf::getYaw(rhs.pose.orientation));
}

void Planner::publish(const nav_msgs::Path &path)
{
    nav_msgs::Path pre_smooted_path = smoothPath(path, 0.9, 0.3);
    nav_msgs::Path interpolated_path = interpolatePath(pre_smooted_path, 0.1);
    nav_msgs::Path smooted_path = smoothPath(interpolated_path, 2.0, 0.3);

    /// path
    raw_path_publisher.publish(path);
    path_publisher.publish(smooted_path);

    visualizePath(smooted_path);
}

nav_msgs::Path Planner::smoothPathSegment(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance) {
    nav_msgs::Path new_path(path);
    new_path.header = path.header;

    unsigned n = path.poses.size();
    if(n < 2) {
        return new_path;
    }

    double last_change = -2 * tolerance;
    double change = 0;

    while(change > last_change + tolerance) {
        last_change = change;
        change = 0;

        for(unsigned i = 1; i < n-1; ++i){
            Pose2d path_i = convert(path.poses[i]);
            Pose2d new_path_i = convert(new_path.poses[i]);
            Pose2d new_path_ip1 = convert(new_path.poses[i+1]);
            Pose2d new_path_im1 = convert(new_path.poses[i-1]);

            Pose2d deltaData = weight_data * (path_i - new_path_i);
            new_path_i = new_path_i + deltaData;

            Pose2d deltaSmooth =  weight_smooth * (new_path_ip1 + new_path_im1 - 2* new_path_i);
            Pose2d nextSmooth = new_path_i + deltaSmooth;

            new_path.poses[i].pose.position.x = nextSmooth.x;
            new_path.poses[i].pose.position.y = nextSmooth.y;

            change += deltaData.distance_to_origin() + deltaSmooth.distance_to_origin();
        }
    }

    // update orientations
    double a = tf::getYaw(new_path.poses[0].pose.orientation);
    geometry_msgs::Pose current = new_path.poses[0].pose;
    geometry_msgs::Pose next = new_path.poses[1].pose;

    double dx = next.position.x - current.position.x;
    double dy = next.position.y - current.position.y;

    double dotprod = std::cos(a) * dx + std::sin(a) * dy;
    bool is_backward = dotprod < 0;//!new_path[0].forward;

    for(unsigned i = 1; i < n-1; ++i){
        Pose2d next = convert(new_path.poses[i+1]);
        Pose2d prev = convert(new_path.poses[i-1]);

        Pose2d delta = next - prev;
        double angle = std::atan2(delta.y, delta.x);

        if(is_backward) {
            angle = MathHelper::AngleClamp(angle + M_PI);
        }

        new_path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    }

    return new_path;
}
