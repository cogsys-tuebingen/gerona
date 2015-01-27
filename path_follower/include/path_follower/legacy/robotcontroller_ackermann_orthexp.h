#ifndef ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
#define ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H

/// THIRD PARTY
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/multiplepidwrapper.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/pathfollower.h>

class RobotController_Ackermann_OrthogonalExponential : public RobotController
{
public:
    RobotController_Ackermann_OrthogonalExponential(ros::Publisher &cmd_publisher, PathFollower *path_driver);

    virtual void publishCommand();

    //! Immediately stop any motion.
    virtual void stopMotion();

    virtual void start();
    virtual ControlStatus execute();

    virtual ObstacleDetector* getObstacleDetector()
    {
        return &obstacle_detector_;
    }


protected:
    virtual void setPath(Path::Ptr path);

    virtual void reset();

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
    void clearBuffers();
    void interpolatePath();
    void publishInterpolatedPath();

    void findMinDistance();

    void keepHeading();
    void lookInDrivingDirection();
    void rotate();

private:
    struct Command
    {
        RobotController_Ackermann_OrthogonalExponential *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Ackermann_OrthogonalExponential *parent):
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
    ros::Publisher points_pub_;

    ros::Subscriber look_at_sub_;
    ros::Subscriber look_at_cmd_sub_;

    ros::Subscriber laser_sub_front_;
    ros::Subscriber laser_sub_back_;

    std::vector<float> ranges_front_;
    std::vector<float> ranges_back_;

    nav_msgs::Path interp_path;
    std::vector<double> p;
    std::vector<double> q;
    std::vector<double> p_prim;
    std::vector<double> q_prim;
    std::vector<double> curvature;


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


    double curv_sum;
    double look_ahead_dist;
    double distance_to_goal;
    double distance_to_obstacle_;


    //control parameters
    double param_k;
    double param_kp;
    double param_kd;
    double param_k_curv;
    double param_k_g;
    double param_k_o;
    double param_k_w;


    double max_angular_velocity;

    visualization_msgs::Marker robot_path_marker;
};

#endif // ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
