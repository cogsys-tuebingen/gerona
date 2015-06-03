#ifndef ROBOTCONTROLLER_INTERPOLATION_H
#define ROBOTCONTROLLER_INTERPOLATION_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>

/// SYSTEM
#include <nav_msgs/Path.h>

/**
 * @brief The RobotController_Interpolation class is a base class for
 *        all controllers that use the interpolated path instead of
 *        the raw path.
 */
class RobotController_Interpolation : public RobotController
{
protected:
    RobotController_Interpolation(PathFollower *path_driver);

protected:
    virtual void setPath(Path::Ptr path);
    virtual void reset();

    virtual void initialize();

private:
    void interpolatePath();
    void publishInterpolatedPath();

protected:
    ros::NodeHandle nh_;

    ros::Publisher interp_path_pub_;

    // is there an interpolated path?
    bool interpolated_;

    //number of path elements
    uint N_;

    //interpolated path
    nav_msgs::Path interp_path_;
    //x component of the interpolated path
    std::vector<double> p_;
    //y componenet of the interpolated path
    std::vector<double> q_;
    //first derivation of the x component w.r.t. path
    std::vector<double> p_prim_;
    //first derivation of the y component w.r.t. path
    std::vector<double> q_prim_;
    //second derivation of the x component w.r.t. path
    std::vector<double> p_sek_;
    //second derivation of the y component w.r.t. path
    std::vector<double> q_sek_;
    //curvature in path coordinates
    std::vector<double> curvature_;

    //path variable
    std::vector<double> s_;
    //path variable derivative
    double s_prim_;

};

#endif // ROBOTCONTROLLER_INTERPOLATION_H
