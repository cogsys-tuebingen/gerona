#ifndef ROBOTCONTROLLER_OFC_H
#define ROBOTCONTROLLER_OFC_H

/// THIRD PARTY
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_OFC: public RobotController
{
public:
    RobotController_OFC();

    //! Immediately stop any motion.
    virtual void stopMotion();

protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;
    virtual void initialize();

protected:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> kp_lin;
        P<double> ki_lin;
        P<double> kd_lin;
        P<double> kp_ang;
        P<double> ki_ang;
        P<double> kd_ang;
        P<double> goal_x;
        P<double> goal_y;
        P<double> max_angular_velocity;

        ControllerParameters():
            RobotController::ControllerParameters("ofc"),

            kp_lin(this, "kp_lin", 1.0, "P component for the linear velocity control."),
            ki_lin(this, "ki_lin", 0.1, "I component for the linear velocity control."),
            kd_lin(this, "kd_lin", 0.1, "D component for the linear velocity control."),
            kp_ang(this, "kp_ang", 1.0, "P component for the angular velocity control."),
            ki_ang(this, "ki_ang", 0.1, "I component for the angular velocity control."),
            kd_ang(this, "kd_ang", 0.1, "D component for the angular velocity control."),
            goal_x(this, "goal_x", 2.0, "x component of the goal position."),
            goal_y(this, "goal_y", 0.0, "y component of the goal position."),
            max_angular_velocity(this, "max_angular_velocity", 0.5, "Maximum angular velocity.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_OFC *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_OFC *parent):
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

    //nominal velocity
    double vn_;
    //sampling time
    double Ts_;

    //previous time stamp for PID
    ros::Time T_prev_;
    //previous time stamp for the target's velocity
    ros::Time previous_t_;
    //previous error value
    double e_prev_lin_;
    double e_prev_ang_;
    //integral sum for the PID controller
    double e_sum_lin_;
    double e_sum_ang_;

    //the moment when the person moved for the last time
    ros::Time last_movement_;

    //previous target position vector
    tf::Vector3 target_vec_prev_;

    //a deque of 10 last measured velocity values
    std::deque<double> meas_velocities_;
    //counter used to update the deque
    int counter_;
    //mean velocity
    double mean_vel_;

    //publish the desired goal position
    ros::Publisher goal_pub;
    //publish the target (person) position
    ros::Publisher target_pub;
    //publish the error vector
    ros::Publisher err_vec_pub;

};

#endif // ROBOTCONTROLLER_OFC_H
