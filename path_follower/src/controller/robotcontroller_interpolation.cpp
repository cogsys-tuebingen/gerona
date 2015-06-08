/// HEADER
#include <path_follower/controller/robotcontroller_interpolation.h>

/// PROJECT
#include <path_follower/pathfollower.h>

/// THIRD PARTY
#include "../alglib/interpolation.h"

RobotController_Interpolation::RobotController_Interpolation(PathFollower *path_driver)
    : RobotController(path_driver),
      nh_("~"),
      interpolated_(false)
{
    interp_path_pub_ = path_driver->getNodeHandle().advertise<nav_msgs::Path>("interp_path", 10);
}

void RobotController_Interpolation::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    if(interpolated_) {
        return;
    }

    try {
        path_interpol.interpolatePath(path_);
        publishInterpolatedPath();


    } catch(const alglib::ap_error& error) {
        throw std::runtime_error(error.msg);
    }

    initialize();

}

void RobotController_Interpolation::initialize()
{
    interpolated_ = true;
}

void RobotController_Interpolation::reset()
{
    interpolated_ = false;
}
void RobotController_Interpolation::interpolatePath()
{


}

void RobotController_Interpolation::publishInterpolatedPath()
{
    interp_path_pub_.publish((nav_msgs::Path) path_interpol);
}


bool RobotController_Interpolation::reachedGoal(const Eigen::Vector3d& pose) const
{

    const unsigned int end = path_interpol.length() - 1;
    return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
            <= getParameters().goal_tolerance();
}
