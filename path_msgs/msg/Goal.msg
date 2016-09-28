# A generic path planning goal
std_msgs/String algorithm
std_msgs/String channel

uint8 type

uint8 GOAL_TYPE_POSE = 0
geometry_msgs/PoseStamped pose


uint8 GOAL_TYPE_MAP = 1
nav_msgs/OccupancyGrid map
float32 min_dist

bool has_search_dir
geometry_msgs/Point search_dir


bool grow_obstacles
float32 obstacle_growth_radius
