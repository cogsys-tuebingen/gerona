#ifndef ROBOTCONTROLLER_OMNIDRIVE_VV_H
#define ROBOTCONTROLLER_OMNIDRIVE_VV_H

/// THIRD PARTY
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/multiplepidwrapper.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/pathfollower.h>

class RobotController_Omnidrive_VirtualVehicle : public RobotController
{
public:
    RobotController_Omnidrive_VirtualVehicle(ros::Publisher &cmd_publisher, PathFollower *path_driver);

    virtual void publishCommand();

    //! Immediatley stop any motion.
    virtual void stopMotion();

    virtual void initOnLine();

    virtual ObstacleDetector* getObstacleDetector()
    {
        return &obstacle_detector_;
    }

    virtual bool isOmnidirectional() const
    {
        return true;
    }


protected:
    virtual void setPath(PathWithPosition path);

    virtual void behaveOnLine();

    /**
     * @return True, when turning point is reached, otherwise false.
     */
    virtual bool behaveApproachTurningPoint();

    virtual void reset();

private:
    struct Command
    {
        RobotController_Omnidrive_VirtualVehicle *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Omnidrive_VirtualVehicle *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f), rotation(0.0f)
        {}

        operator geometry_msgs::Twist()
        {
            // direction_angle is relative to direction of movement;
            // control angle, however, is relative to orientation of the robot.
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
    ros::Publisher vv_path_pub_;
    ros::Publisher points_pub_;


    nav_msgs::Path interp_path;
    nav_msgs::Path vv_path;
    std::vector<double> p;
    std::vector<double> q;
    std::vector<double> p_prim;
    std::vector<double> q_prim;
    std::vector<double> xr_calc;
    std::vector<double> yr_calc;

    bool initialized;

    uint N;
    double vn;
    uint counter;
    double d;
    double k;
    double epsilon;
    double alpha;
    double c;
    double ro;
    double gama;
    double Ts;
    double psi_d_prev;
    visualization_msgs::Marker robot_path;
};

#endif // ROBOTCONTROLLER_OMNIDRIVE_VV_H
