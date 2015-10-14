# A generic path planning goal
std_msgs/String algorithm

uint8 type

uint8 GOAL_TYPE_POSE = 0
geometry_msgs/PoseStamped pose

uint8 GOAL_TYPE_MAP = 1
nav_msgs/OccupancyGrid map
float32 min_dist
