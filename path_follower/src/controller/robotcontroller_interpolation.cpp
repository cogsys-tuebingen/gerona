/// HEADER
#include <path_follower/controller/robotcontroller_interpolation.h>

/// PROJECT
#include <path_follower/pathfollower.h>

/// THIRD PARTY
#include "../alglib/interpolation.h"

RobotController_Interpolation::RobotController_Interpolation(PathFollower *path_driver)
    : RobotController(path_driver),
      nh_("~"),
      interpolated_(false),
      N_(0),
      s_prim_(0)
{
    interp_path_pub_ = path_driver->getNodeHandle().advertise<nav_msgs::Path>("interp_path", 10);
}

void RobotController_Interpolation::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    if(interpolated_) {
        return;
    }

    p_.clear();
    q_.clear();
    p_prim_.clear();
    q_prim_.clear();
    p_sek_.clear();
    q_sek_.clear();
    s_.clear();
    interp_path_.poses.clear();
    curvature_.clear();

    try {
        interpolatePath();
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
    std::deque<Waypoint> waypoints;
    waypoints.insert(waypoints.end(), path_->getCurrentSubPath().begin(), path_->getCurrentSubPath().end());

    // (messy) hack!!!!!
    // remove waypoints that are closer than 0.1 meters to the starting point
    Waypoint start = waypoints.front();
    while(!waypoints.empty()) {
        std::deque<Waypoint>::iterator it = waypoints.begin();
        const Waypoint& wp = *it;

        double dx = wp.x - start.x;
        double dy = wp.y - start.y;
        double distance = hypot(dx, dy);
        if(distance < 0.1) {
            waypoints.pop_front();
        } else {
            break;
        }
    }

    //copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
    //as an intermediate step, calculate the arclength of the curve, and do the reparameterization with respect to arclength

    N_ = waypoints.size();

    if(N_ < 2) {
        return;
    }

    double X_arr[N_], Y_arr[N_], l_arr[N_], l_arr_unif[N_];
    double L = 0;

    for(std::size_t i = 0; i < N_; ++i) {
        const Waypoint& waypoint = waypoints[i];

        X_arr[i] = waypoint.x;
        Y_arr[i] = waypoint.y;

    }

    l_arr[0] = 0;

    for(std::size_t i = 1; i < N_; i++){

        L += hypot(X_arr[i] - X_arr[i-1], Y_arr[i] - Y_arr[i-1]);
        l_arr[i] = L;

    }
    ROS_INFO("Length of the path: %lf m", L);


    double f = std::max(0.0001, L / (double) (N_-1));

    for(std::size_t i = 1; i < N_; i++){

        l_arr_unif[i] = i * f;

    }

    //initialization before the interpolation
    alglib::real_1d_array X_alg, Y_alg, l_alg, l_alg_unif;
    alglib::real_1d_array x_s, y_s, x_s_prim, y_s_prim, x_s_sek, y_s_sek;

    X_alg.setcontent(N_, X_arr);
    Y_alg.setcontent(N_, Y_arr);
    l_alg.setcontent(N_, l_arr);
    l_alg_unif.setcontent(N_, l_arr_unif);


    //interpolate the path and find the derivatives
    alglib::spline1dconvdiff2cubic(l_alg, X_alg, l_alg_unif, x_s, x_s_prim, x_s_sek);
    alglib::spline1dconvdiff2cubic(l_alg, Y_alg, l_alg_unif, y_s, y_s_prim, y_s_sek);

    //define path components, its derivatives, and curvilinear abscissa, then calculate the path curvature
    for(uint i = 0; i < N_; ++i) {

        p_.push_back(x_s[i]);
        q_.push_back(y_s[i]);

        p_prim_.push_back(x_s_prim[i]);
        q_prim_.push_back(y_s_prim[i]);

        p_sek_.push_back(x_s_sek[i]);
        q_sek_.push_back(y_s_sek[i]);

        s_.push_back(l_alg_unif[i]);

        curvature_.push_back((x_s_prim[i]*y_s_sek[i] - x_s_sek[i]*y_s_prim[i])/
                            (sqrt(pow((x_s_prim[i]*x_s_prim[i] + y_s_prim[i]*y_s_prim[i]), 3))));

    }

}

void RobotController_Interpolation::publishInterpolatedPath()
{
    if(N_ <= 2) {
        return;
    }

    for(uint i = 0; i < N_; ++i) {
        geometry_msgs::PoseStamped poza;
        poza.pose.position.x = p_[i];
        poza.pose.position.y = q_[i];
        interp_path_.poses.push_back(poza);
    }

    interp_path_.header.frame_id = "map";
    interp_path_pub_.publish(interp_path_);
}

