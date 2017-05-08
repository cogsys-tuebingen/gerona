## Options for path following algorithms

# init_mode: Determines how the path follower reacts to a new path:
#  INIT_MODE_STOP will cause the robot to brake
#  INIT_MODE_CONTINUE will not send a brake command
uint8 INIT_MODE_STOP = 0
uint8 INIT_MODE_CONTINUE = 1
uint8 init_mode

# velocity: Driving speed of the robot (interpreted as recommended value)
float32 velocity
