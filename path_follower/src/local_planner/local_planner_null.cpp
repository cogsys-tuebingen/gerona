/// HEADER
#include <path_follower/local_planner/local_planner_null.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerNull::LocalPlannerNull(PathFollower &follower,
                                   tf::Transformer &transformer)
    : LocalPlanner(follower, transformer)
{

}

void LocalPlannerNull::setGlobalPath(Path::Ptr path)
{
    /*
     * this local planner does nothing, so the global path is
     * equal to the local path!
     */
    LocalPlanner::setGlobalPath(path);

    follower_.getController()->setPath(global_path_);
}

Path::Ptr LocalPlannerNull::updateLocalPath()
{
    // do nothing
    return nullptr;
}

bool LocalPlannerNull::isNull() const
{
    return true;
}
