#ifndef ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H
#define ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H

/// THIRD PARTY
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include "robotcontroller.h"
#include "multiplepidwrapper.h"
#include "visualizer.h"
#include "obstacledetectoromnidrive.h"
#include "pathfollower.h"

class RobotController_Omnidrive_OrthogonalExponential : public RobotController
{
public:
    RobotController_Omnidrive_OrthogonalExponential(ros::Publisher &cmd_publisher, PathFollower *path_driver);

    virtual void publishCommand();

    //! Immediately stop any motion.
    virtual void stopMotion();

    virtual void initOnLine();

    virtual ObstacleDetector* getObstacleDetector()
    {
        return &obstacle_detector_;
    }


protected:
    virtual void setPath(PathWithPosition path);

    virtual void behaveOnLine();

    virtual void behaveAvoidObstacle();

    /**
     * @return True, when turning point is reached, otherwise false.
     */
    virtual bool behaveApproachTurningPoint();

    virtual void reset();

    virtual bool isOmnidirectional() const
    {
        return true;
    }

    void lookAtCommand(const std_msgs::StringConstPtr& cmd);
    void lookAt(const geometry_msgs::PointStampedConstPtr& look_at);

private:
    void initialize();
    void clearBuffers();
    void interpolatePath();
    void publishInterpolatedPath();

    void keepHeading();
    void lookInDrivingDirection();
    void rotate();

private:
    struct Command
    {
        RobotController_Omnidrive_OrthogonalExponential *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Omnidrive_OrthogonalExponential *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f), rotation(0.0f)
        {}

        operator geometry_msgs::Twist()
        {
            // direction_angle is relative to direction of movement;
            // control angle, however, is relative to orientation of the robot.
            //Eigen::Vector2d mov_dir = parent_->predictDirectionOfMovement(); //FIXME: auch falsch, mov_dir ist relativ zur welt?!s
            //float angle = atan2(mov_dir(1), mov_dir(0)) + direction_angle;
            float angle = direction_angle;

            geometry_msgs::Twist msg;
            msg.linear.x  = speed * cos(angle);
            msg.linear.y  = speed * sin(angle);
            msg.angular.z = rotation;
            return msg;
        }

        bool isValid()
        {
            if ( isnan(speed) || isinf(speed)
                 || isnan(direction_angle) || isinf(direction_angle)
                 || isnan(rotation) || isinf(rotation) )
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          isnan(speed), isinf(speed),
                          isnan(direction_angle), isinf(direction_angle),
                          isnan(rotation), isinf(rotation));
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

    Visualizer *visualizer_;

    Command cmd_;
    ObstacleDetectorOmnidrive obstacle_detector_;

    ros::NodeHandle nh_;
    ros::Publisher interp_path_pub_;
    ros::Publisher points_pub_;

    ros::Subscriber look_at_sub_;
    ros::Subscriber look_at_cmd_sub_;


    nav_msgs::Path interp_path;
    std::vector<double> p;
    std::vector<double> q;
    std::vector<double> p_prim;
    std::vector<double> q_prim;


    enum ViewDirection {
        KeepHeading,
        LookAtPoint,
        LookInDrivingDirection,
        Rotate
    };

    ViewDirection view_direction_;
    geometry_msgs::Point look_at_;

    bool initialized;

    double vn;
    double theta_des;
    uint N;
    double Ts;
    double e_theta_curr;


    //control parameters
    double param_k;
    double param_kp;
    double param_kd;

    double brake_distance;
    double max_angular_velocity;
    double rotation_threshold_min;
    double rotation_threshold_max;

    visualization_msgs::Marker robot_path_marker;
};

#endif // ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H
