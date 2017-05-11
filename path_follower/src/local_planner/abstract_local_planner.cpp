/// HEADER
#include <path_follower/local_planner/abstract_local_planner.h>
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/utils/pose_tracker.h>

#include <pcl_ros/point_cloud.h>

AbstractLocalPlanner::AbstractLocalPlanner()
    : controller_(nullptr),
      pose_tracker_(nullptr),
      transformer_(nullptr),
      opt_(nullptr),

      last_update_(0)
{

}

void AbstractLocalPlanner::init(RobotController* controller, PoseTracker* pose_tracker,
                        const LocalPlannerParameters& opt)
{
    controller_ = controller;
    pose_tracker_ = pose_tracker;
    opt_ = &opt;

    update_interval_ = ros::Duration (opt_->update_interval());

    transformer_ = &pose_tracker_->getTransformListener();

    setParams(opt);
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

bool AbstractLocalPlanner::isNull() const
{
    return false;
}

void AbstractLocalPlanner::setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg)
{
    last_obstacle_cloud_ = obstacle_cloud_;
    obstacle_cloud_ = msg;
}


void AbstractLocalPlanner::addConstraint(Constraint::Ptr constraint)
{
    constraints.push_back(constraint);
}

void AbstractLocalPlanner::addScorer(Scorer::Ptr scorer, double weight)
{
    scorer->setWeight(weight);
    scorers.push_back(scorer);
}


Path::Ptr AbstractLocalPlanner::setPath(const std::string& frame_id, SubPath& local_wps, ros::Time& now)
{
    Path::Ptr local_path = std::make_shared<Path>(frame_id);
    local_path->setPath({local_wps});

    controller_->reset();

    controller_->setGlobalPath(global_path_.getOriginalPath());

    if(local_wps.empty()) {
        local_path->setPath({});
        controller_->stopMotion();

    } else {
        local_path->setPath({local_wps});
        controller_->setPath(local_path);
    }

    last_update_ = now;

    return local_path;
}

std::vector<SubPath> AbstractLocalPlanner::getAllLocalPaths() const
{
    return {};
}
