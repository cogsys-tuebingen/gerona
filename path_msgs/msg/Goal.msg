# A generic path planning goal
std_msgs/String planning_algorithm
std_msgs/String planning_channel

std_msgs/String following_algorithm

uint8 type

uint8 GOAL_TYPE_POSE = 0
geometry_msgs/PoseStamped pose


uint8 GOAL_TYPE_MAP = 1
nav_msgs/OccupancyGrid map
float32 min_dist

bool has_search_dir
geometry_msgs/Point search_dir

float32 max_search_duration

bool grow_obstacles
float32 obstacle_growth_radius
