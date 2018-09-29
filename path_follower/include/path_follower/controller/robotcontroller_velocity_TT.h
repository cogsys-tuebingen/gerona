#ifndef ROBOTCONTROLLER_VELOCITY_TT_H
#define ROBOTCONTROLLER_VELOCITY_TT_H

/// THIRD PARTY
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_Velocity_TT: public RobotController
{
public:
    /**
     * @brief RobotController_Kinematic_HBZ
     */
    RobotController_Velocity_TT();

    /**
     * @brief stopMotion stops the robot
     */
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
     * @brief initialize
     */
    virtual void initialize();
    /**
     * @brief WheelVelocities computes the speed of the left and right wheels
     *
     * Currently not used, but could be helpful.
     *
     */
    //void WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array);
    /**
     * @brief computeSpeed computes the actual command velocity, by using different speed control techniques
     */
    //virtual double computeSpeed();

private:
    void findMinDistance();

protected:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> dist_thresh;
        P<double> krep;



        ControllerParameters(const std::string& name = "ekm"):

            RobotController::ControllerParameters(name),
            dist_thresh(this, "dist_thresh", 1, "dist_thresh description."),
            krep(this, "krep", 0.1, "krep description")

        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_Velocity_TT *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Velocity_TT *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f), rotation(0.0f)
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


    void reset();
    void setPath(Path::Ptr path);

    cv::Vec2f CalcForceRep();
    cv::Vec2f CalcForceRep(const pcl::PointCloud<pcl::PointXYZ>& cloud);

private:

    std::queue<geometry_msgs::PoseStamped> targetPoses_;



};

#endif // ROBOTCONTROLLER_KINEMATIC_HBZ_H
