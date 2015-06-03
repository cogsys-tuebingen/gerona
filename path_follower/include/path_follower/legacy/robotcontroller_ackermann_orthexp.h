#ifndef ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
#define ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H

/// THIRD PARTY
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/pathfollower.h>

class RobotController_Ackermann_OrthogonalExponential : public RobotController_Interpolation
{
public:
    RobotController_Ackermann_OrthogonalExponential(PathFollower *path_driver);

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

private:
    void initialize();

    void findMinDistance();

    void keepHeading();
    void lookInDrivingDirection();
    void rotate();

private:
    struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters
    {
        P<double> k;
        P<double> kp;
        P<double> kd;
        P<double> max_angular_velocity;
        P<double> look_ahead_dist;
        P<double> k_o;
        P<double> k_g;
        P<double> k_w;
        P<double> k_curv;

        ControllerParameters():
            k(this, "~k", 1.5, ""),
            kp(this, "~kp", 0.4, ""),
            kd(this, "~kd", 0.2, ""),
            max_angular_velocity(this, "~max_angular_velocity", 2.0, ""),
            look_ahead_dist(this, "~look_ahead_dist", 0.5, ""),
            k_o(this, "~k_o", 0.3, ""),
            k_g(this, "~k_g", 0.4, ""),
            k_w(this, "~k_w", 0.5, ""),
            k_curv(this, "~k_curv", 0.05, "")
        {}
    } opt_;

    const RobotController_Interpolation::InterpolationParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_Ackermann_OrthogonalExponential *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;


        // initialize all values to zero
        Command(RobotController_Ackermann_OrthogonalExponential *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f)
        {}

        operator MoveCommand()
        {
            MoveCommand mcmd;
            mcmd.setDirection(direction_angle);
            mcmd.setVelocity(speed);
            return mcmd;
        }

        bool isValid()
        {
            if ( isnan(speed) || isinf(speed)
                 || isnan(direction_angle) || isinf(direction_angle))
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d",
                          isnan(speed), isinf(speed),
                          isnan(direction_angle), isinf(direction_angle));
                // fix this instantly, to avoid further problems.
                speed = 0.0;
                direction_angle = 0.0;

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
    double e_theta_curr_;

    double curv_sum_;
    double distance_to_goal_;
    double distance_to_obstacle_;
};

#endif // ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
