#ifndef PATHFOLLOWERPARAMETERS_H
#define PATHFOLLOWERPARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>

struct PathFollowerParameters : public Parameters
{
    P<std::string> controller;
    P<double> wp_tolerance;
    P<double> goal_tolerance;
    P<double> steer_slow_threshold;
    P<double> max_distance_to_path;
    P<std::string> world_frame;
    P<std::string> robot_frame;
    P<bool> use_vfh;
    P<bool> use_path_lookout;
    P<float> min_velocity;
    P<float> max_velocity;
    P<float> collision_box_width;
    P<float> collision_box_min_length;
    P<float> collision_box_crit_length;
    P<float> collision_box_max_length;
    P<float> collision_box_velocity_factor;
    P<float> collision_box_velocity_saturation;
    P<bool> abort_if_obstacle_ahead;

    PathFollowerParameters():
        controller(this, "~controller", "ackermann_pid", "Defines, which controller is used."),
        wp_tolerance(this,  "~waypoint_tolerance",  0.20 , ""),
        goal_tolerance(this,  "~goal_tolerance",  0.15 , ""),
        steer_slow_threshold(this,  "~steer_slow_threshold",  0.25 , ""),
        max_distance_to_path(this,  "~max_distance_to_path",  0.3 , "Maximum distance the robot is allowed to depart from the path. If this threshold is exceeded, the path follower will abort."),

        world_frame(this, "~world_frame",  "/map", "Name of the world frame."),
        robot_frame(this, "~robot_frame",  "/base_link", "Name of the robot frame."),

        use_vfh(this, "~use_vfh",  false, "If set to true, vector field histogram is used for collision avoidance."),
        use_path_lookout(this, "~use_path_lookout",  true, "If set to true, path lookout is done (check if there are obstacles somewhere on the path ahead of the robot)."),

        min_velocity(this,  "~min_velocity",  0.4 , "Minimum speed of the robot (needed, as the outdoor buggys can't handle velocities below about 0.3)."),
        max_velocity(this,  "~max_velocity",  2.0 , "Maximum velocity (to prevent the high level control from running amok)."),

        collision_box_width(this,  "~collision_box_width",  0.5, "Width of the collision box for obstacle avoidance."),
        collision_box_min_length(this,  "~collision_box_min_length",  0.5, "Minimum length of the collision box for obstacle avoidance (grows with increasing velocity)."),
        collision_box_crit_length(this,  "~collision_box_crit_length",  0.3, ""),
        collision_box_max_length(this,  "~collision_box_max_length",  1.0, "Maximum length of the collision box for obstacle avoidance."),
        collision_box_velocity_factor(this,  "~collision_box_velocity_factor",  1.0, "This factor determines, how much the length of the box is increased, depending on the velocity."),
        collision_box_velocity_saturation(this,  "~collision_box_velocity_saturation",  max_velocity(), "The velocity for which the maximum length should be used."),
        abort_if_obstacle_ahead(this, "~abort_if_obstacle_ahead",  false, "If set to true, path execution is aborted, if an obstacle is detected on front of the robot. If false, the robot will stop, but not abort (the obstacle might move away).")
    {
        if(max_velocity() < min_velocity()) {
            ROS_ERROR("min velocity larger than max velocity!");
            max_velocity.set(min_velocity());
        }
        if(collision_box_max_length() < collision_box_min_length()) {
            ROS_ERROR("min length larger than max length!");
            collision_box_min_length.set(collision_box_max_length());
        }
        if(collision_box_min_length() < collision_box_crit_length()) {
            ROS_ERROR("min length smaller than crit length!");
            collision_box_crit_length.set(collision_box_min_length());
        }
    }
};

#endif // PATHFOLLOWERPARAMETERS_H
