# Options for path planning algorithms
# Note that not all path planners support these options!

bool has_search_dir
geometry_msgs/Point search_dir

float32 max_search_duration

bool grow_obstacles
float32 obstacle_growth_radius

float32 goal_dist_threshold
float32 goal_angle_threshold_degree

bool reversed

bool allow_forward
bool allow_backward

float32 ackermann_la
int32 ackermann_steer_steps
float32 ackermann_max_steer_angle_degree
float32 ackermann_steer_delta_degree

