# A generic path planning goal

# planning_algorithm: [optional] Which path planner to use, an empty string uses the default planner.
std_msgs/String planning_algorithm

# planning_channel: [optional] The topic on which to request a new path, can be empty.
std_msgs/String planning_channel

# type: The type of goal this message specifies:
#  GOAL_TYPE_POSE: Find a path to the pose specified in <pose>
#  GOAL_TYPE_MAP: Find a path that ends in a cell of <map> which has a value of at least <map_search_min_value>
uint8 GOAL_TYPE_POSE = 0
uint8 GOAL_TYPE_MAP = 1
uint8 type



### POSE SEARCH
## only if <type> == GOAL_TYPE_POSE

geometry_msgs/PoseStamped pose



### MAP SEARCH
## only if <type> == GOAL_TYPE_MAP

# map: A map coding for the goal locations using the threshold <map_search_min_value>
nav_msgs/OccupancyGrid map

# map_search_min_value: Minimum cell value to be considered a goal [defaults to 32]
int32 map_search_min_value

# map_search_min_value: Minimum number of candidate cells to find before terminating [defaults to 64]
int32 map_search_min_candidates

# min_dist: The minimum required path length [defaults to 0.0]
float32 min_dist

