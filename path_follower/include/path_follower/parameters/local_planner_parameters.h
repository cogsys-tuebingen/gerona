#ifndef LOCAL_PLANNER_PARAMETERS_H
#define LOCAL_PLANNER_PARAMETERS_H

#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>
#include <path_follower/parameters/path_follower_parameters.h>

struct LocalPlannerParameters : public Parameters
{
    static const LocalPlannerParameters* getInstance()
    {
        static LocalPlannerParameters instance(PathFollowerParameters::getInstance());
        return &instance;
    }

    //Parameters for the Local Planner
    P<std::string> local_planner;
    P<bool> use_distance_to_path_constraint, use_distance_to_obstacle_constraint;
    P<double> score_weight_distance_to_path, score_weight_delta_distance_to_path, score_weight_curvature, score_weight_delta_curvature, score_weight_level, score_weight_distance_to_obstacles;
    P<int> max_num_nodes,max_depth,curve_segment_subdivisions,intermediate_angles;
    P<double> update_interval,distance_to_path_constraint, safety_distance_surrounding, safety_distance_forward, max_steering_angle, step_scale, mu, ef;
    P<bool> use_velocity;
    P<double> min_velocity, min_distance_to_goal;

private:
    LocalPlannerParameters(const Parameters* parent):
        Parameters("local_planner", parent),

        local_planner(this, "algorithm", "NULL", "Algorithm to be used by the Local Planner."),

        //Constraints
        use_distance_to_path_constraint(this,
                                        "use_distance_to_path_constraint",
                                        true,
                                        "Determines whether the first constraint is used or not. (Distance to global path)"),
        use_distance_to_obstacle_constraint(this,
                                            "use_distance_to_obstacle_constraint",
                                            true,
                                            "Determines whether the second constraint is used or not. (Distance to nearest obstacle)"),

        //Scorers
        score_weight_distance_to_path(this,
                                      "score_weight_distance_to_path",
                                      3.5,
                                      "Determines whether the first scorer is used or not. (Distance to global path (P))"),
        score_weight_delta_distance_to_path(this,
                                            "score_weight_delta_distance_to_path",
                                            1.5,
                                            "Determines whether the second  scorer is used or not. (Distance to global path (D))"),
        score_weight_curvature(this,
                               "score_weight_curvature",
                               4.0,
                               "Determines whether the third scorer is used or not. (Curvature of the point (P))"),
        score_weight_delta_curvature(this,
                                     "score_weight_delta_curvature",
                                     3.0,
                                     "Determines whether the fouth scorer is used or not. (Curvature of the point (D))"),
        score_weight_level(this,
                           "score_weight_level",
                           1.5,
                           "Determines whether the fifth scorer is used or not. (Tree level reached)"),
        score_weight_distance_to_obstacles(this,
                                           "score_weight_distance_to_obstacles",
                                           5.0,
                                           "Determines whether the sixth scorer is used or not. (Distance to nearest obstacle)"),

        //Other local planner parameters
        max_num_nodes(this, "max_num_nodes", 400,
                      "Determines the maximum number of nodes used by the local planner"),
        max_depth(this, "max_depth", 10,
                  "Determines the maximum depth of the tree used by the local planner"),
        curve_segment_subdivisions(this, "curve_segment_subdivisions", 7,
                                   "Determines the number of subdivisions of curve segments in the final path"),
        intermediate_angles(this, "intermediate_angles", 0,
                            "Determines the number of intermediate angles between 0 and +-s_angle for the expansion of a node"),
        update_interval(this, "update_interval", 0.125,
                        "Determines the update interval in seconds of the local planner"),
        distance_to_path_constraint(this, "distance_to_path_constraint", 2.5,
                                    "Determines how far from the path should the local planner perform"),
        safety_distance_surrounding(this, "safety_distance_surrounding", 0.65,
                                    "Determines the security distance around the robot"),
        safety_distance_forward(this, "safety_distance_forward", 0.55,
                                "Determines the extra security distance in front of the robot"),
        max_steering_angle(this, "max_steering_angle", 35.0,
                           "Determines the steering angle (in degrees) for the local planner"),
        step_scale(this, "step_scale", 1.20,
                   "Scales the size of individual steps during the search"),
        mu(this, "mu", 0.1,
           "Determines the coefficient of friction"),
        ef(this, "ef", 2.0,
           "Exponential factor the dis2obst scorer (std::exp(<ef>/distance(point, obstacle)) - 1.0"),
        use_velocity(this, "use_velocity", true,
                     "Determines if the current velocity is used by the local planner"),
        min_velocity(this, "min_velocity", 0.5,
                     "Minimum velocity for planning"),
        min_distance_to_goal(this, "min_distance_to_goal", 0.2,
                     "If goal is within this distance stop")


      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
    }
};

#endif // LOCAL_PLANNER_PARAMETERS_H
