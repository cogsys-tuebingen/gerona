#ifndef PATHFOLLOWERPARAMETERS_H
#define PATHFOLLOWERPARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>

struct PathFollowerParameters : public Parameters
{
    P<std::string> controller;
    P<std::string> world_frame;
    P<std::string> robot_frame;
    P<double> wp_tolerance;
    P<double> goal_tolerance;
    P<double> steer_slow_threshold;
    P<float> min_velocity;
    P<float> max_velocity;
    P<bool> abort_if_obstacle_ahead;

    // obstacle avoider
    P<bool> obstacle_avoider_use_collision_box;
    //P<bool> obstacle_avoider_use_vfh;  // not yet implemented

    // supervisors
    P<bool> supervisor_use_path_lookout;
    P<bool> supervisor_use_waypoint_timeout;
    P<bool> supervisor_use_distance_to_path;

    P<double> supervisor_distance_to_path_max_dist;
    P<float> supervisor_waypoint_timeout_time;


    PathFollowerParameters():
        controller(this, "~controller", "ackermann_purepursuit", "Defines, which controller is used."),
        world_frame(this, "~world_frame",  "/map", "Name of the world frame."),
        robot_frame(this, "~robot_frame",  "/base_link", "Name of the robot frame."),
        wp_tolerance(this,  "~waypoint_tolerance",  0.20 , ""),
        goal_tolerance(this,  "~goal_tolerance",  0.15 , ""),
        steer_slow_threshold(this,  "~steer_slow_threshold",  0.25 ,
                             "Robot slows down, when steering angle exceeds this threshold."
                             " May not be supported by all robot controllers."),
        min_velocity(this,  "~min_velocity",  0.4 ,
                     "Minimum speed of the robot (needed, as the outdoor buggys can't handle"
                     " velocities below about 0.3)."),
        max_velocity(this,  "~max_velocity",  2.0 ,
                     "Maximum velocity (to prevent the high level control from running amok)."),

        abort_if_obstacle_ahead(this, "~abort_if_obstacle_ahead",  false,
                                "If set to true, path execution is aborted, if an obstacle is"
                                " detected on front of the robot. If false, the robot will"
                                " stop, but not abort (the obstacle might move away)."),

        obstacle_avoider_use_collision_box(this, "~obstacle_avoider/use_collision_box", true,
                                           "Use the collision box obstacle avoider ('ObstacleDetector')"),
        //obstacle_avoider_vfh(this, "~obstacle_avoider/use_vfh",  false,
        //                     "If set to true, vector field histogram is used for collision avoidance."),

		  supervisor_use_path_lookout(this, "~supervisor/use_path_lookout",  false,
                                    "Set to `true` to activate supervisor _path lookout_ (check"
                                    " if there are obstacles somewhere on the path ahead of the"
                                    " robot)."),
		  supervisor_use_waypoint_timeout(this, "~supervisor/use_waypoint_timeout", false,
                                        "Set to `true` to activate supervisor _waypoint timeout_."),
		  supervisor_use_distance_to_path(this, "~supervisor/use_distance_to_path", false,
                                        "Set to `true` to activate supervisor _distance to path_."),

        supervisor_distance_to_path_max_dist(this,  "~supervisor/distance_to_path/max_dist",  0.3,
                                             "Maximum distance the robot is allowed to depart from the path."
                                             " If this threshold is exceeded, the distance_to_path superv will abort."),

        supervisor_waypoint_timeout_time(this, "~supervisor/waypoint_timeout/time", 10.0,
                                         "If the robot needs more than this time (in seconds), supervisor"
                                         " WaypointTimeout will stop the path execution.")
    {
        if(max_velocity() < min_velocity()) {
            ROS_ERROR("min velocity larger than max velocity!");
            max_velocity.set(min_velocity());
        }
    }
};

#endif // PATHFOLLOWERPARAMETERS_H
