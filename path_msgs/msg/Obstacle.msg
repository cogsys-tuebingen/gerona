# An obstacle is defined by it's enclosing circle
# position is the center of the circle

geometry_msgs/Point position
float32 radius

# Result of the obstacle weighting in path lookout. If this is > 1, the robot will stop.
float32 weight
