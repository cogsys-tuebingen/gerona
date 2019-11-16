#ifndef PATHFOLLOWERPARAMETERS_H
#define PATHFOLLOWERPARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>
#include <ros/node_handle.h>

struct PathFollowerParameters : public Parameters
{
    static const PathFollowerParameters* getInstance()
    {
        static PathFollowerParameters instance;
        return &instance;
    }

private:
    ros::NodeHandle nh;

public:
    P<std::string> controller;
    P<std::string> collision_avoider;
    P<std::string> world_frame;
    P<std::string> robot_frame;
    P<std::string> odom_frame;
    P<double> wp_tolerance;
    P<double> goal_tolerance;
    P<double> steer_slow_threshold;
    P<float> min_velocity;
    P<float> max_velocity;
    P<bool> abort_if_obstacle_ahead;

private:
    PathFollowerParameters():

        controller(this, "controller_type", "ackermann_purepursuit", "Defines which controller is used."),
        collision_avoider(this, "collision_avoider_type", "", "Defines, which collision avoider is used."),

        world_frame(this, "world_frame",
                    nh.param("gerona/world_frame", std::string("map")),
                    "Name of the world frame."),
        robot_frame(this, "robot_frame",
                    nh.param("gerona/robot_frame", std::string("base_link")),
                    "Name of the robot frame."),
        odom_frame(this, "odom_frame",
                   nh.param("gerona/odom_frame", std::string("odom")),
                   "Name of the odometry frame."),

        wp_tolerance(this, "waypoint_tolerance",  0.20 , ""),
        goal_tolerance(this, "goal_tolerance",  0.15 , ""),
        steer_slow_threshold(this, "steer_slow_threshold",  0.25 ,
                             "Robot slows down, when steering angle exceeds this threshold."
                             " May not be supported by all robot controllers."),
        min_velocity(this, "min_velocity",  0.4 ,
                     "Minimum speed of the robot (needed, as the outdoor buggys can't handle"
                     " velocities below about 0.3)."),
        max_velocity(this, "max_velocity",  2.0 ,
                     "Maximum velocity (to prevent the high level control from running amok)."),

        abort_if_obstacle_ahead(this, "abort_if_obstacle_ahead",  false,
                                "If set to true, path execution is aborted, if an obstacle is"
                                " detected on front of the robot. If false, the robot will"
                                " stop, but not abort (the obstacle might move away).")

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
        if(max_velocity() < min_velocity()) {
            ROS_ERROR("min velocity larger than max velocity!");
            max_velocity.set(min_velocity());
        }
    }
};

#endif // PATHFOLLOWERPARAMETERS_H
