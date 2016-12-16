/// HEADER
#include <path_follower/local_planner/local_planner.h>
#include <pcl_ros/point_cloud.h>
#include <path_follower/utils/obstacle_cloud.h>

#include <path_follower/utils/pose_tracker.h>

LocalPlanner::LocalPlanner()
    : controller_(nullptr),
      pose_tracker_(nullptr),
      transformer_(nullptr)
{

}

void LocalPlanner::init(RobotController* controller, PoseTracker* pose_tracker, const ros::Duration& update_interval)
{
    controller_ = controller;
    pose_tracker_ = pose_tracker;
    update_interval_ = update_interval;

    transformer_ = &pose_tracker_->getTransformListener();
}

LocalPlanner::~LocalPlanner()
{

}

void LocalPlanner::reset()
{

}

void LocalPlanner::setGlobalPath(Path::Ptr path)
{
    global_path_.interpolatePath(path,false);

    ros::Time now = ros::Time::now();

    tf::StampedTransform initial_map_to_odom_;
    if(transformer_->waitForTransform("map", "odom", now, ros::Duration(1.0))) {
        transformer_->lookupTransform("map", "odom", now, initial_map_to_odom_);
        return;
    }
    if(transformer_->waitForTransform("map", "odom", ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN_NAMED("global_path", "cannot transform map to odom, using latest");
        transformer_->lookupTransform("map", "odom", ros::Time(0), initial_map_to_odom_);
        return;
    }

    ROS_ERROR_NAMED("global_path", "cannot transform map to odom");
}

bool LocalPlanner::isNull() const
{
    return false;
}

void LocalPlanner::setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg)
{
    last_obstacle_cloud_ = obstacle_cloud_;
    obstacle_cloud_ = msg;
}


void LocalPlanner::addConstraint(Constraint::Ptr constraint)
{
    constraints.push_back(constraint);
}

void LocalPlanner::addScorer(Scorer::Ptr scorer, double weight)
{
    scorer->setWeight(weight);
    scorers.push_back(scorer);
}
