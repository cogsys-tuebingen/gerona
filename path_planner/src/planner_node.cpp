/// HEADER
#include "planner_node.h"

/// PROJECT
#include <utils_path/common/CollisionGridMap2d.h>
#include <utils_path/common/Bresenham2d.h>
#include <utils_general/Stopwatch.h>

/// SYSTEM
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>

using namespace lib_path;

Planner::Planner()
    : nh("~"),
      server_(nh, "/plan_path", boost::bind(&Planner::execute, this, _1), false),
      map_info(NULL)
{
    std::string target_topic = "/goal";
    nh.param("target_topic", target_topic, target_topic);

    //    goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //            (target_topic, 2, boost::bind(&Planner::updateGoalCallback, this, _1));

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

    nh.param("preprocess", pre_process_, true);
    nh.param("postprocess", post_process_, true);

    nh.param("use_cost_map", use_cost_map_, false);
    if(use_cost_map_ && !pre_process_) {
        use_cost_map_service_ = true;
        std::string costmap_service = "/dynamic_map/cost";
        nh.param("cost_map_service",costmap_service, costmap_service);
        cost_map_service_client = nh.serviceClient<nav_msgs::GetMap> (costmap_service);

        std::cout << "using cost map service " << costmap_service << std::endl;

    } else {
        use_cost_map_service_ = false;
        if(pre_process_) {
            use_cost_map_ = true;
        }
    }

    nh.param("use_scan_front", use_scan_front_, true);
    nh.param("use_scan_back", use_scan_back_, true);

    if(use_scan_front_) {
        sub_front = nh.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 0, boost::bind(&Planner::laserCallback, this, _1, true));
    }
    if(use_scan_back_) {
        sub_back = nh.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 0, boost::bind(&Planner::laserCallback, this, _1, false));
    }


    nh.param("use_collision_gridmap", use_collision_gridmap_, false);

    viz_pub = nh.advertise<visualization_msgs::Marker>("/marker", 0);
    cost_pub = nh.advertise<nav_msgs::OccupancyGrid>("cost", 1, true);

    base_frame_ = "/base_link";
    nh.param("base_frame", base_frame_, base_frame_);


    nh.param("size/forward", size_forward, 0.4);
    nh.param("size/backward", size_backward, -0.6);
    nh.param("size/width", size_width, 0.5);

    path_publisher = nh.advertise<nav_msgs::Path> ("/path", 10);
    raw_path_publisher = nh.advertise<nav_msgs::Path> ("/path_raw", 10);

    server_.registerPreemptCallback(boost::bind(&Planner::preempt, this));
    server_.start();
}

Planner::~Planner()
{

}

void Planner::preempt()
{
    ROS_WARN("preempting!!");

    thread_mutex.lock();
    bool running = thread_running;
    thread_mutex.unlock();
    if(running) {
        ROS_WARN_STREAM("preempting path planner");
        thread_->interrupt();
        thread_->join();
    }

    server_.setPreempted();
}

void Planner::feedback(int status)
{
    if(server_.isActive()) {
        path_msgs::PlanPathFeedback f;
        f.status = status;
        server_.publishFeedback(f);
    }
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

        if(use_collision_gridmap_) {
            map_info = new lib_path::CollisionGridMap2d(map.info.width, map.info.height, map.info.resolution, size_forward, size_backward, size_width);
        } else {
            map_info = new lib_path::SimpleGridMap2d(map.info.width, map.info.height, map.info.resolution);
        }
    }

    bool use_unknown;
    nh.param("use_unknown_cells", use_unknown, true);

    std::vector<uint8_t> data(w*h);

    int i = 0;
    if(use_unknown) {
        /// Map data
        /// -1: unknown -> 0
        /// 0:100 probabilities -> 1 - 100
        for(std::vector<int8_t>::const_iterator it = map.data.begin(); it != map.data.end(); ++it) {
            data[i++] = std::min(100, *it + 1);
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


    cost_map.header = map.header;
    cost_map.info = map.info;
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
    ROS_INFO("planner: got goal");

    findPath(lookupPose(), *goal);
}

void Planner::execute(const path_msgs::PlanPathGoalConstPtr &goal)
{
    Stopwatch sw_global;
    ROS_INFO("planner: got request");

    Stopwatch sw;

    sw.reset();
    geometry_msgs::PoseStamped s;
    if(goal->use_start) {
        s = goal->start;
    } else {
        s = lookupPose();
    }
    ROS_INFO_STREAM("start pose lookup took " << sw.msElapsed() << "ms");

    sw.reset();
    nav_msgs::Path path = findPath(s, goal->goal);
    ROS_INFO_STREAM("findPath took " << sw.msElapsed() << "ms");

    if(path.poses.empty()) {
        feedback(path_msgs::PlanPathFeedback::STATUS_PLANNING_FAILED);

        path_msgs::PlanPathResult fail;
        server_.setAborted(fail, "no path found");

    } else {
        feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

        path_msgs::PlanPathResult success;
        success.path = path;
        server_.setSucceeded(success);
    }
    ROS_INFO_STREAM("execution took " << sw_global.msElapsed() << "ms");
}

nav_msgs::Path Planner::findPath(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
    Stopwatch sw;

    if(use_map_service_) {
        sw.reset();
        nav_msgs::GetMap map_service;
        if(map_service_client.call(map_service)) {
            updateMap(map_service.response.map);
        } else {
            ROS_ERROR("map service lookup failed");
            return nav_msgs::Path();
        }
        ROS_INFO_STREAM("map service lookup took " << sw.msElapsed() << "ms");
    }

    if(map_info == NULL) {
        ROS_ERROR("request for path planning, but no map there yet...");
        return nav_msgs::Path();
    }

    if(use_scan_front_ && !scan_front.ranges.empty()) {
        integrateLaserScan(scan_front);
    }
    if(use_scan_back_ && !scan_back.ranges.empty()) {
        integrateLaserScan(scan_back);
    }

    if(use_cost_map_service_) {
        sw.reset();
        nav_msgs::GetMap map_service;
        if(cost_map_service_client.call(map_service)) {
            cost_map = map_service.response.map;
        }
        ROS_INFO_STREAM("cost map service lookup took " << sw.msElapsed() << "ms");
    }

    if(pre_process_) {
        sw.reset();
        preprocess(start, goal);
        ROS_INFO_STREAM("preprocessing took " << sw.msElapsed() << "ms");
    }

    sw.reset();
    nav_msgs::Path path_raw = doPlan(start, goal);
    ROS_INFO_STREAM("planning took " << sw.msElapsed() << "ms");

    nav_msgs::Path path;
    if(post_process_) {
        sw.reset();
        path = postprocess(path_raw);
        ROS_INFO_STREAM("postprocessing took " << sw.msElapsed() << "ms");
    } else {
        path = path_raw;
    }

    sw.reset();
    publish(path, path_raw);
    ROS_INFO_STREAM("publish took " << sw.msElapsed() << "ms");


    return path;
}


void Planner::preprocess(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
    // growth
    cv::Mat map(map_info->getHeight(), map_info->getWidth(), CV_8UC1, map_info->getData());
    cv::Mat working;
    map.copyTo(working);

    int erosion_size = 4;
    int iterations = 2;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );
    cv::dilate(working, working, element, cv::Point(-1,-1), iterations);


    cv::Mat mask(working.rows, working.cols, CV_8UC1, cv::Scalar::all(255));

    unsigned sx, sy;
    map_info->point2cell(start.pose.position.x, start.pose.position.y, sx, sy);
    unsigned gx, gy;
    map_info->point2cell(goal.pose.position.x, goal.pose.position.y, gx, gy);

    int r = erosion_size * iterations * 0.5;
    cv::circle(mask, cv::Point(sx,sy), r, cv::Scalar::all(0), CV_FILLED);
    cv::circle(mask, cv::Point(gx,gy), r, cv::Scalar::all(0), CV_FILLED);


    working.copyTo(map, mask);


    // cost
    int h = map_info->getHeight();
    int w = map_info->getWidth();
    cost_map.data.resize(h*w);
    cv::Mat costmap(h, w, CV_8UC1, cost_map.data.data());
    map.copyTo(costmap);

//    cv::Mat unknown_mask;
//    cv::inRange(map, 0, 0, unknown_mask);
//    costmap.setTo(0, unknown_mask);
//    cv::threshold(costmap, costmap, 50, 255, cv::THRESH_BINARY);

    costmap = 100 - costmap;

    cv::Mat distance;
    cv::distanceTransform(costmap, distance, CV_DIST_L2, CV_DIST_MASK_PRECISE);

    double scale_  = 100.0;
    double max_distance_meters_ = 2.5;
    double factor = (scale_ * map_info->getResolution() / max_distance_meters_);
    distance.convertTo(costmap, CV_8UC1, factor);

//    cv::threshold(costmap, costmap, 98, 98, CV_THRESH_TRUNC);
    costmap = cv::max(0, 98 - costmap);
//    costmap.setTo(50, unknown_mask);

    cv::imwrite("costmap.png", costmap);

    cost_pub.publish(cost_map);
}

nav_msgs::Path Planner::postprocess(const nav_msgs::Path& path)
{
    Stopwatch sw;

    sw.restart();
    nav_msgs::Path simplified_path = simplifyPath(path);
    ROS_INFO_STREAM("simplifying took " << sw.msElapsed() << "ms");

    //    nav_msgs::Path pre_smooted_path = smoothPath(simplified_path, 0.9, 0.3);

    sw.restart();
    nav_msgs::Path interpolated_path = interpolatePath(simplified_path, 0.5);
    ROS_INFO_STREAM("interpolation took " << sw.msElapsed() << "ms");


    sw.restart();
    nav_msgs::Path smooted_path = smoothPath(interpolated_path, 0.6, 0.3);
    ROS_INFO_STREAM("smoothing took " << sw.msElapsed() << "ms");


    sw.restart();
    nav_msgs::Path final_interpolated_path = interpolatePath(smooted_path, 0.1);
    ROS_INFO_STREAM("final interpolation took " << sw.msElapsed() << "ms");


    sw.restart();
    nav_msgs::Path final_smoothed_path = smoothPath(final_interpolated_path, 2.0, 0.4);
    ROS_INFO_STREAM("final smoothing took " << sw.msElapsed() << "ms");

    return final_smoothed_path;
}


geometry_msgs::PoseStamped Planner::lookupPose()
{
    geometry_msgs::PoseStamped own_pose;
    own_pose.header.frame_id = "/map";
    own_pose.header.stamp = ros::Time::now();
    tf::StampedTransform trafo = lookupTransform(own_pose.header.frame_id, base_frame_, own_pose.header.stamp);
    tf::poseTFToMsg(trafo, own_pose.pose);

    return own_pose;
}

tf::StampedTransform Planner::lookupTransform(const std::string& from, const std::string& to, const ros::Time& stamp)
{
    tf::StampedTransform trafo;
    if(tfl.waitForTransform(from, to, stamp, ros::Duration(1.0))) {
        tfl.lookupTransform(from, to, stamp, trafo);
    } else {
        ROS_WARN("cannot lookup own pose, using last estimate");
        tfl.lookupTransform(from, to, ros::Time(0), trafo);
    }
    return trafo;
}


nav_msgs::Path Planner::doPlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
    feedback(path_msgs::PlanPathFeedback::STATUS_PLANNING);

    ROS_INFO("starting search");
    lib_path::Pose2d from_world;
    lib_path::Pose2d to_world;

    from_world.x = start.pose.position.x;
    from_world.y = start.pose.position.y;
    from_world.theta = tf::getYaw(start.pose.orientation);

    ROS_WARN_STREAM("theta=" << from_world.theta);

    to_world.x = goal.pose.position.x;
    to_world.y = goal.pose.position.y;
    to_world.theta = tf::getYaw(goal.pose.orientation);

    visualizeOutline(start.pose, 0, "/map");
    visualizeOutline(goal.pose, 1, "/map");

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


    feedback(path_msgs::PlanPathFeedback::STATUS_POST_PROCESSING);

    //nav_msgs::Path path = plan(goal, from_world, to_world, from_map, to_map);

    boost::thread worker(boost::bind(&Planner::planThreaded, this, goal, from_world, to_world, from_map, to_map));
    ros::Rate spin(10);
    ros::Time start_time = ros::Time::now();
    ros::Duration max_search_time(40);
    while(ros::ok()) {
        ROS_INFO_STREAM_THROTTLE(2, "still planning");
        bool timeout = start_time + max_search_time < ros::Time::now();
        if(timeout){
            ROS_ERROR("search timed out");
        }
        if(server_.isPreemptRequested() || timeout) {
            ROS_INFO_STREAM("preempted path planner");
            return nav_msgs::Path();

        } else {
            bool running;
            thread_mutex.lock();
            running = thread_running;
            thread_ = &worker;
            thread_mutex.unlock();

            if(!running) {
                worker.join();
                break;
            }
        }
        spin.sleep();
    }

    ROS_INFO_STREAM("sub-planner done or aborted");

    thread_mutex.lock();
    nav_msgs::Path path = thread_result;
    thread_mutex.unlock();

    return path;
}

void Planner::planThreaded(const geometry_msgs::PoseStamped &goal, const Pose2d &from_world, const Pose2d &to_world, const Pose2d &from_map, const Pose2d &to_map)
{
    thread_mutex.lock();
    thread_running = true;
    thread_mutex.unlock();

    nav_msgs::Path path = plan(goal, from_world, to_world, from_map, to_map);

    thread_mutex.lock();
    thread_result = path;
    thread_running = false;
    thread_mutex.unlock();
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

namespace {
bool isFree(SimpleGridMap2d* map_ptr, const nav_msgs::OccupancyGrid& costmap,
            const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
    lib_path::Bresenham2d bresenham;

    unsigned int fx, fy, tx, ty;
    map_ptr->point2cell(from.x, from.y, fx, fy);
    map_ptr->point2cell(to.x, to.y, tx, ty);
    bresenham.setGrid(map_ptr, fx, fy, tx, ty);

    std::size_t w = costmap.info.width;

    unsigned x,y;
    while(bresenham.next()) {
        bresenham.coordinates(x,y);
        if(!map_ptr->isFree(x,y)) {
            return false;
        }
        std::size_t idx = y * w + x;
        if(costmap.data[idx] > 5) {
            return false;
        }
    }

    return true;
}
}

nav_msgs::Path Planner::simplifyPath(const nav_msgs::Path &path)
{
    nav_msgs::Path result = path;

    for(std::size_t i = 1; i < result.poses.size() - 1;) {
        // check if i can be removed
        if(isFree(map_info, cost_map, result.poses[i-1].pose.position, result.poses[i+1].pose.position)) {
            result.poses.erase(result.poses.begin() + i);
        } else {
            ++i;
        }
    }

    return result;
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

    for(int i = 0; i < n; ++i){
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

void Planner::laserCallback(const sensor_msgs::LaserScanConstPtr &scan, bool front)
{
    if(front) {
        scan_front = *scan;
    } else {
        scan_back = *scan;
    }
}

void Planner::integrateLaserScan(const sensor_msgs::LaserScan &scan)
{
    tf::StampedTransform trafo = lookupTransform("/map", scan.header.frame_id, scan.header.stamp);

    double angle = scan.angle_min;
    for(std::size_t i = 0, total = scan.ranges.size(); i < total; ++i) {
        const float& range = scan.ranges[i];
        tf::Vector3 pt_laser(std::cos(angle) * range, std::sin(angle) * range, 0);
        tf::Vector3 pt_map = trafo * pt_laser;

        unsigned int x,y;
        if(map_info->point2cell(pt_map.x(), pt_map.y(), x, y)) {
            map_info->setValue(x,y, 100);
        }

        angle += scan.angle_increment;
    }
}

void Planner::publish(const nav_msgs::Path &path, const nav_msgs::Path &path_raw)
{
    /// path
    raw_path_publisher.publish(path_raw);
    path_publisher.publish(path);

    visualizePath(path);
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
            new_path_i = new_path_i + deltaSmooth;

            new_path.poses[i].pose.position.x = new_path_i.x;
            new_path.poses[i].pose.position.y = new_path_i.y;

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
