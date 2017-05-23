#ifndef ROBOTCONTROLLER_PBR_H
#define ROBOTCONTROLLER_PBR_H

/// THIRD PARTY
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller.h>

#include <path_follower/utils/extended_kalman_filter.h>

class RobotController_PBR: public RobotController
{
public:
    /**
     * @brief RobotController_PBR
     */
    RobotController_PBR();
    /**
     * @brief stopMotion stops the robot
     */
    virtual void stopMotion();
    /**
     * @brief start
     */
    virtual void start();
    /**
     * @brief setCurrentPose corrects the EKF state
     * @param current robot pose measurement
     */
    virtual void setCurrentPose(const Eigen::Vector3d&);

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
     * @brief WheelVelocities preditcs the EKF state
     * @param array representing the velocities of each wheel
     */
    void WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array);

private:
    void findMinDistance();

private:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> k1;
        P<double> k2;
        P<double> max_angular_velocity;

        ControllerParameters():
            RobotController::ControllerParameters("PBR"),

            k1(this, "k1", 1.0, "Factor for tuning the angular velocity command."),
            k2(this, "k2", 1.0, "Factor for tuning the angular velocity command."),
            max_angular_velocity(this, "max_angular_velocity", 0.8, "Maximum angular velocity.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_PBR *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_PBR *parent):
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

    EKF ekf_;

    ros::Subscriber look_at_sub_;
    ros::Subscriber look_at_cmd_sub_;

    ros::Subscriber laser_sub_front_;
    ros::Subscriber laser_sub_back_;

    ros::Subscriber wheel_vel_sub_;

    ros::Publisher ICR_pub_;
    ros::Publisher marker_pub_;

    std::vector<float> ranges_front_;
    std::vector<float> ranges_back_;

    void reset();
    void setPath(Path::Ptr path);


    //nominal velocity
    double vn_;
    //sampling time
    double Ts_;

    //index of the orthogonal projection to the path
    uint ind_;

    //velocity of the left tread
    double Vl_;
    //velocity of the right tread
    double Vr_;

    //pose estimated by the EKF
    Eigen::Vector3d pose_ekf_;
    //ICR coordinates estimated by the EKF
    Eigen::Vector3d ICR_ekf_;
    //last time step in which the prediction was made
    ros::Time last_time_;

    //cumulative curvature sum w.r.t. path
    double curv_sum_;
    //cumulative distance to goal sum w.r.t. path
    double distance_to_goal_;
    //distance to the nearest obstacle
    double distance_to_obstacle_;

    //points estimated by the EKF
    visualization_msgs::Marker ekf_path_marker_;

    //points of the augmented path
    visualization_msgs::Marker path_aug_marker_;

    //x and y components of the augmented path
    std::vector<double> x_aug_;
    std::vector<double> y_aug_;
};

#endif // ROBOTCONTROLLER_PBR_H

