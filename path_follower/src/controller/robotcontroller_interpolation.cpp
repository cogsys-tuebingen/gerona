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
//    if(N_ <= 2) {
//        return;
//    }

//    for(uint i = 0; i < N_; ++i) {
//        geometry_msgs::PoseStamped poza;
//        poza.pose.position.x = p_[i];
//        poza.pose.position.y = q_[i];
//        interp_path_.poses.push_back(poza);
//    }

//    interp_path_.header.frame_id = "map";
//    interp_path_pub_.publish(interp_path_);
}

