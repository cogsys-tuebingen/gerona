/// HEADER
#include <path_follower/controller/robotcontroller_interpolation.h>

/// PROJECT
#include <path_follower/pathfollower.h>

/// THIRD PARTY
#include <interpolation.h>

RobotController_Interpolation::RobotController_Interpolation()
    : RobotController(),
      interpolated_(false)
{
    interp_path_pub_ = pnh_.advertise<nav_msgs::Path>("interp_path", 10);
}

void RobotController_Interpolation::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    if(interpolated_) {
        return;
    }

    std::cerr << "interpolating path in frame " << path->getFrameId() << std::endl;

    path_interpol.interpolatePath(path);
    publishInterpolatedPath();

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

void RobotController_Interpolation::publishInterpolatedPath()
{
    interp_path_pub_.publish((nav_msgs::Path) path_interpol);
}


bool RobotController_Interpolation::reachedGoal(const Eigen::Vector3d& pose) const
{

    const unsigned int end = path_interpol.n() - 1;
    return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
            <= getParameters().goal_tolerance();
}
