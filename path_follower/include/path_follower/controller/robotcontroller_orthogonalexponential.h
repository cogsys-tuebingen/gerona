#ifndef ROBOTCONTROLLER_ORTHEXP_H
#define ROBOTCONTROLLER_ORTHEXP_H

/// THIRD PARTY
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_OrthogonalExponential: public RobotController
{
public:
    RobotController_OrthogonalExponential();

    //! Immediately stop any motion.
    virtual void stopMotion();

    virtual void start();


protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

    virtual bool isOmnidirectional() const
    {
        return true;
    }

    void lookAtCommand(const std_msgs::StringConstPtr& cmd);
    void lookAt(const geometry_msgs::PointStampedConstPtr& look_at);
    void laserBack(const sensor_msgs::LaserScanConstPtr& scan_back);
    void laserFront(const sensor_msgs::LaserScanConstPtr& scan_front);

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
        P<double> max_angular_velocity;

        ControllerParameters(const std::string& name = "orthexp"):
            RobotController::ControllerParameters(name),
            max_angular_velocity(this, "max_angular_velocity", 2.0, "")
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

