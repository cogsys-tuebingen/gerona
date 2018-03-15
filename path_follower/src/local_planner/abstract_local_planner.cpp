/// HEADER
#include <path_follower/local_planner/abstract_local_planner.h>
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/utils/pose_tracker.h>

#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/parameters/local_planner_parameters.h>

#include <pcl_ros/point_cloud.h>

AbstractLocalPlanner::AbstractLocalPlanner()
    : controller_(nullptr),
      pose_tracker_(nullptr),
      transformer_(nullptr),
      opt_(nullptr),

      last_update_(0)
{

}

void AbstractLocalPlanner::init(RobotController* controller, PoseTracker* pose_tracker)
{
    controller_ = controller;
    pose_tracker_ = pose_tracker;

    opt_ = LocalPlannerParameters::getInstance();

    update_interval_ = ros::Duration (opt_->update_interval());

    transformer_ = &pose_tracker_->getTransformListener();

    setParams(*opt_);
}

AbstractLocalPlanner::~AbstractLocalPlanner()
{

}

void AbstractLocalPlanner::reset()
{

}

void AbstractLocalPlanner::setGlobalPath(Path::Ptr path)
{
    global_path_.interpolatePath(path,false);

    ros::Time now = ros::Time::now();

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();

    tf::StampedTransform initial_map_to_odom_;    
    if(transformer_->waitForTransform(world_frame, odom_frame, now, ros::Duration(1.0))) {
        transformer_->lookupTransform(world_frame, odom_frame, now, initial_map_to_odom_);
        return;
    }
    if(transformer_->waitForTransform(world_frame, odom_frame, ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN_NAMED("global_path", "cannot transform map to odom, using latest");
        transformer_->lookupTransform(world_frame, odom_frame, ros::Time(0), initial_map_to_odom_);
        return;
    }

    ROS_ERROR_NAMED("global_path", "cannot transform %s to %s", world_frame.c_str(), odom_frame.c_str());
}

bool AbstractLocalPlanner::isNull() const
{
    return false;
}

void AbstractLocalPlanner::setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg)
{
    last_obstacle_cloud_ = obstacle_cloud_;
    obstacle_cloud_ = msg;
}

void AbstractLocalPlanner::setElevationMap(const std::shared_ptr<ElevationMap const> &msg)
{
    last_elevation_map_ = elevation_map_;
    elevation_map_ = msg;
}



void AbstractLocalPlanner::addConstraint(Constraint::Ptr constraint)
{
    constraints.push_back(constraint);
}

void AbstractLocalPlanner::setVelocity(geometry_msgs::Twist velocity)
{
    setVelocity(velocity.linear);
}



void AbstractLocalPlanner::addScorer(Scorer::Ptr scorer, double weight)
{
    scorer->setWeight(weight);
    scorers.push_back(scorer);
}


void AbstractLocalPlanner::setPath(const Path::Ptr& local_path, const ros::Time& now)
{
    controller_->reset();

    controller_->setGlobalPath(global_path_.getOriginalPath());

    if(local_path->empty()) {
        controller_->stopMotion();

    } else {
        controller_->setPath(local_path);
    }

    last_update_ = now;
}

Path::Ptr AbstractLocalPlanner::setPath(const std::string& frame_id, const SubPath& local_wps, const ros::Time& now)
{
    Path::Ptr local_path = std::make_shared<Path>(frame_id);
    local_path->setPath({local_wps});

    setPath(local_path, now);

    return local_path;
}

std::vector<SubPath> AbstractLocalPlanner::getAllLocalPaths() const
{
    return {};
}

//borrowed from path_planner/planner_node.cpp
SubPath AbstractLocalPlanner::interpolatePath(const SubPath& path, double max_distance){
    unsigned n = path.size();
    if(n < 2) {
        return path;
    }

    SubPath result;
    result.forward = path.forward;
    result.push_back(path[0]);

    for(unsigned i = 1; i < n; ++i){
        const Waypoint* current = &path[i];
        // split the segment, iff it is to large
        subdividePath(result, result.back(), *current, max_distance);
        // add the end of the segment (is not done, when splitting)
        result.push_back(*current);
    }
    return result;
}

//borrowed from path_planner/planner_node.cpp : TODO: unify
void AbstractLocalPlanner::smoothAndInterpolate(SubPath& local_wps){
    //interpolate
    local_wps = interpolatePath(local_wps, 0.5);
    //smoothing
    local_wps = smoothPath(local_wps, 0.6, 0.15);
    //final interpolate
    local_wps = interpolatePath(local_wps, 0.1);
    //final smoothing
    local_wps = smoothPath(local_wps, 2.0, 0.4);
}
//borrowed from path_planner/planner_node.cpp
SubPath AbstractLocalPlanner::smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance){
    int n = path.size();
    if(n < 2) {
        return path;
    }

    SubPath result;
    result.forward = path.forward;
    // find segments
    // smooth segments and merge results
    SubPath smoothed_segment = smoothPathSegment(path, weight_data, weight_smooth, tolerance);
    result.wps.insert(result.end(), smoothed_segment.begin(), smoothed_segment.end());

    return result;
}


//borrowed from path_planner/planner_node.cpp
void AbstractLocalPlanner::subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance){
    double distance = low.distanceTo(up);
    if(distance > max_distance) {
        // split half way between the lower and the upper node
        Waypoint halfway(low.x,low.y,low.orientation);

        halfway.x += (up.x - low.x) / 2.0;
        halfway.y += (up.y - low.y) / 2.0;

        halfway.orientation = (up.orientation + low.orientation) / 2.0;
        // first recursive descent in lower part
        subdividePath(result, low, halfway, max_distance);
        // then add the half way point
        result.push_back(halfway);
        // then descent in upper part
        subdividePath(result, halfway, up, max_distance);
    }
}


//borrowed from path_planner/planner_node.cpp : TODO: unify
SubPath AbstractLocalPlanner::smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance)
{
    unsigned n = path.size();
    if(n < 2) {
        return path;
    }
    SubPath new_path(path);

    double last_change = -2 * tolerance;
    double change = 0;

    int offset = 2;

    while(change > last_change + tolerance) {
        last_change = change;
        change = 0;
        Waypoint origin(0,0,0);

        for(unsigned i = offset; i < n-offset; ++i){
            Waypoint path_i = path[i];
            Waypoint new_path_i = new_path[i];
            Waypoint new_path_ip1 = new_path[i+1];
            Waypoint new_path_im1 = new_path[i-1];

            Waypoint deltaData(weight_data*(path_i.x - new_path_i.x),
                               weight_data*(path_i.y - new_path_i.y),0.0);

            new_path_i.x = new_path_i.x + deltaData.x;
            new_path_i.y = new_path_i.y + deltaData.y;

            Waypoint deltaSmooth(weight_smooth * (new_path_ip1.x + new_path_im1.x - 2* new_path_i.x),
                                 weight_smooth * (new_path_ip1.y + new_path_im1.y - 2* new_path_i.y),
                                 0.0);
            new_path_i.x = new_path_i.x + deltaSmooth.x;
            new_path_i.y = new_path_i.y + deltaSmooth.y;

            new_path[i].x = new_path_i.x;
            new_path[i].y = new_path_i.y;


            change += deltaData.distanceTo(origin) + deltaSmooth.distanceTo(origin);
        }
    }

    // update orientations
    double a = new_path[0].orientation;
    Waypoint current = new_path[0];
    Waypoint next = new_path[1];

    double dx = next.x - current.x;
    double dy = next.y - current.y;

    Eigen::Vector2d looking_dir_normalized(std::cos(a), std::sin(a));
    Eigen::Vector2d delta(dx, dy);
    const double theta_diff = std::acos(delta.dot(looking_dir_normalized) / delta.norm());

    // decide whether to drive forward or backward
    bool is_backward = (theta_diff > M_PI_2 || theta_diff < -M_PI_2) ;

    for(unsigned i = 1; i < n-1; ++i){
        Waypoint next = new_path[i+1];
        Waypoint prev = new_path[i-1];

        Waypoint delta(next.x - prev.x, next.y - prev.y, 0.0);
        double angle = std::atan2(delta.y, delta.x);

        if(is_backward) {
            angle = MathHelper::AngleClamp(angle + M_PI);
        }

        new_path[i].orientation = angle;
    }

    return new_path;
}
