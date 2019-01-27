#ifndef ROBOTCONTROLLER_ORTHEXP_H
#define ROBOTCONTROLLER_ORTHEXP_H

/// THIRD PARTY
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <cslibs_navigation_utilities/MathHelper.h>

// SYSTEM
#include <boost/algorithm/clamp.hpp>

/**
 * @brief The RobotController_OrthogonalExponential class is the base class for all orthexp algorithms
 */
class RobotController_OrthogonalExponential: public RobotController
{
public:
    /**
     * @brief RobotController_2Steer_Stanley
     */
    RobotController_OrthogonalExponential();

    //! Immediately stop any motion.
    virtual void stopMotion();
    /**
     * @brief start
     */
    virtual void start();


protected:
    /**
     * @brief computeMoveCommand computes the command velocity for the robot
     *
     * The command velocity is computed for each controller differently. This is the core of
     * every controller. For more details, please visit: https://github.com/cogsys-tuebingen/gerona/wiki/controllers
     * On this wiki page, you will find references for each controller, where more mathematical and experimental details
     * can be found.
     *
     * @param cmd
     */
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    /**
     * @brief publishMoveCommand publishes the computed move command
     *
     * The command velocity is set depending on the kinematics of the robot. E.g. for
     * differential drives the command input is (v, w), where v is linear, and w angular velocity,
     * and for an Ackermann drive, the command input is (v, phi), where v is linear velocity, and
     * phi is the steering angle. For an omnidirectional vehicle, it is possible to directly set
     * the linear velocity and the direction angle, while the rotation is set independently.
     *
     * @param cmd
     */
    virtual void publishMoveCommand(const MoveCommand &cmd) const;
    /**
     * @brief lookAtCommand defines whether the robot should look at a certain point while driving
     * @param cmd
     */
    void lookAtCommand(const std_msgs::StringConstPtr& cmd);
    /**
     * @brief lookAt defines the point at which the robot should look at while driving
     * @param look_at
     */
    void lookAt(const geometry_msgs::PointStampedConstPtr& look_at);
    void laserBack(const sensor_msgs::LaserScanConstPtr& scan_back);
    void laserFront(const sensor_msgs::LaserScanConstPtr& scan_front);
    /**
     * @brief computeControl computes the command velocity specific for every orthogonal-exponential controller
     *
     * Orthogonal-exponential controller is in principle the same for every wheeled robot, but the command
     * output is computed differently for different kinematic types.
     *
     */
    virtual void computeControl();

private:
    void initialize();

    void findMinDistance();

    void keepHeading();
    void lookInDrivingDirection();
    void rotate();

protected:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> max_ang_velocity;

        ControllerParameters(const std::string& name = "orthexp"):
            RobotController::ControllerParameters(name),
            max_ang_velocity(this, "max_angular_velocity", 0.5, " Maximum angular velocity.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_OrthogonalExponential *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_OrthogonalExponential *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f)
        {}

        operator MoveCommand()
        {
            MoveCommand mcmd(true);
            mcmd.setDirection(direction_angle);
            mcmd.setVelocity(speed);
            mcmd.setRotationalVelocity(rotation);
            return mcmd;
        }

        bool isValid()
        {
            if ( std::isnan(speed) || std::isinf(speed)
                 || std::isnan(direction_angle) || std::isinf(direction_angle)
                 || std::isnan(rotation) || std::isinf(rotation) )
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          std::isnan(speed), std::isinf(speed),
                          std::isnan(direction_angle), std::isinf(direction_angle),
                          std::isnan(rotation), std::isinf(rotation));
                // fix this instantly, to avoid further problems.
                speed = 0.0;
                direction_angle = 0.0;
                rotation = 0.0;

                return false;
            } else {
                return true;
            }
        }
    };

    Command cmd_;

    ros::NodeHandle nh_;

    ros::Subscriber look_at_sub_;
    ros::Subscriber look_at_cmd_sub_;

    ros::Subscriber laser_sub_front_;
    ros::Subscriber laser_sub_back_;

    std::vector<float> ranges_front_;
    std::vector<float> ranges_back_;

    enum ViewDirection {
        KeepHeading,
        LookAtPoint,
        LookInDrivingDirection,
        Rotate
    };

    ViewDirection view_direction_;
    geometry_msgs::Point look_at_;

    double vn_;
    double theta_des_;
    double Ts_;

    double alpha_e_;
};
#endif // ROBOTCONTROLLER_ORTHEXP_H

