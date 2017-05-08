# Options for path planning algorithms

# max_search_duration: [optional] Number of seconds before the search terminates [defaults to infinity]
float32 max_search_duration

# has_search_dir: Determines whether a map-based search should use <search_dir> as a heuristic goal [defaults to false]
bool has_search_dir
# search_dir: [optional] The heuristic goal for a map-based search
geometry_msgs/Point search_dir

# grow_obstacle: [optional] Determines whether to grow obstacles [defaults to false]
bool grow_obstacles
# obstacle_growth_radius: [optional] The radius to grow obstacles by
float32 obstacle_growth_radius


### IMPLEMENTATION SPECIFIC OPTIONS
## Note that not all path planners support these options!

# goal_dist_threshold: Locations closer than this value are considered to be the goal
float32 goal_dist_threshold

# goal_angle_threshold_degree: Orientations closer than this value are considered to be the goal
float32 goal_angle_threshold_degree

# reversed: If true, the planner is requested to plan from the goal pose to the start
bool reversed


# allow_forward: Determines whether to allow forward segments
bool allow_forward

# allow_forward: Determines whether to allow backward segments
bool allow_backward

#note: if both directions are false (or not specified), both are considered true


## Ackermann specifica options
float32 ackermann_la
int32 ackermann_steer_steps
float32 ackermann_max_steer_angle_degree
float32 ackermann_steer_delta_degree

