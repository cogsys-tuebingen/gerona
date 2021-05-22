/// HEADER
#include <path_follower/local_planner/high_speed_local_planner.h>

/// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/pose_tracker.h>

HighSpeedLocalPlanner::HighSpeedLocalPlanner()
    : waypoints(), wlp_(), close_to_goal(false), last_update_(0)
{

}

void HighSpeedLocalPlanner::setGlobalPath(Path::Ptr path)
{
    AbstractLocalPlanner::setGlobalPath(path);
    close_to_goal = false;
}

bool HighSpeedLocalPlanner::transform2Odo(ros::Time& now)
{
    tf::StampedTransform now_map_to_odom;

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();

    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(world_frame, odom_frame, ros::Time(0), now_map_to_odom);
    }catch(const tf::TransformException& ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(world_frame, odom_frame, now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform(world_frame, odom_frame, now, now_map_to_odom);
    }

    tf::Transform transform_correction = now_map_to_odom.inverse();

    // transform the waypoints from world to odom
    for(Waypoint& wp : waypoints) {
        tf::Point pt(wp.x, wp.y, 0);
        pt = transform_correction * pt;
        wp.x = pt.x();
        wp.y = pt.y();

        tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
        rot = transform_correction * rot;
        wp.orientation = tf::getYaw(rot);

    }

    return true;
}

void HighSpeedLocalPlanner::printSCTimeUsage()
{
    for(std::size_t i = 0; i < constraints.size(); ++i){
        ROS_INFO_STREAM("Constraint #" << (i+1) << " took " << constraints.at(i)->nsUsed()/1000.0 << " us");
    }
    for(std::size_t i = 0; i < scorers.size(); ++i){
        ROS_INFO_STREAM("Scorer #" << (i+1) << " took " << scorers.at(i)->nsUsed()/1000.0 << " us");
    }
}

Path::Ptr HighSpeedLocalPlanner::updateLocalPath()
{
    ros::Time now = ros::Time::now();
    Stopwatch gsw;
    gsw.restart();
    if(last_update_ + update_interval_ < now && !close_to_goal) {

        // only look at the first sub path for now
        waypoints_map = (SubPath) global_path_;
        waypoints = (SubPath) global_path_;
        wlp_.wps.clear();

        std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();

        Path::Ptr local_path(new Path(odom_frame));

        if(!transform2Odo(now)){
            ROS_WARN_THROTTLE(1, "cannot calculate local path, transform to odom not known");
            return local_path;
        }

        if(!obstacle_cloud_) {
            ROS_WARN_THROTTLE(1, "computing local path without obstacle cloud");
        }


        Eigen::Vector3d pose = pose_tracker_->getRobotPose();

        std::size_t nnodes = 0;

        SubPath local_wps;
        local_wps.forward = true;

        if(!algo(pose, local_wps, nnodes)){
            return local_path;
        }

        return setPath(odom_frame, local_wps, now);

    } else {
        return nullptr;
    }
}
