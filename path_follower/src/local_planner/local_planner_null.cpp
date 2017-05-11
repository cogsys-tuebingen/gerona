/// HEADER
#include <path_follower/local_planner/local_planner_null.h>

/// PROJECT

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/factory/local_planner_factory.h>

REGISTER_LOCAL_PLANNER(LocalPlannerNull, Null);


LocalPlannerNull::LocalPlannerNull()
{

}

void LocalPlannerNull::setGlobalPath(Path::Ptr path)
{
    /*
     * this local planner does nothing, so the global path is
     * equal to the local path!
     */
    LocalPlanner::setGlobalPath(path);

    controller_->setGlobalPath(path);
    controller_->setPath(path);
}

Path::Ptr LocalPlannerNull::updateLocalPath(Path::Ptr& wlp)
{
    (void) wlp;
    // do nothing
    return nullptr;
}

bool LocalPlannerNull::isNull() const
{
    return true;
}

void LocalPlannerNull::setParams(const LocalPlannerParameters &opt){
}

void LocalPlannerNull::setVelocity(geometry_msgs::Twist::_linear_type vector){
    (void) vector;
}

void LocalPlannerNull::setVelocity(double velocity){
    (void) velocity;
}
