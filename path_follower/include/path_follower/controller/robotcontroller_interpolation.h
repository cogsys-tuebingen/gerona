#ifndef ROBOTCONTROLLER_INTERPOLATION_H
#define ROBOTCONTROLLER_INTERPOLATION_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/path_interpolated.h>

/// SYSTEM
#include <nav_msgs/Path.h>
#include <Eigen/Core>

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

    bool reachedGoal(const Eigen::Vector3d& pose) const;

public:
    virtual void setCurrentPose(const Eigen::Vector3d&) override {}

protected:
    struct InterpolationParameters : public Parameters {
        P<double> goal_tolerance;

        InterpolationParameters() :
            goal_tolerance(this, "~goal_tolerance", 0.3, "minimum distance at which the robot stops")
        {}
    };

    virtual const InterpolationParameters& getParameters() const = 0;

protected:
    void publishInterpolatedPath();

protected:
    ros::NodeHandle nh_;

    ros::Publisher interp_path_pub_;


    PathInterpolated path_interpol;

    // is there an interpolated path?
    bool interpolated_;

    //interpolated path
    nav_msgs::Path interp_path_;
};

#endif // ROBOTCONTROLLER_INTERPOLATION_H
