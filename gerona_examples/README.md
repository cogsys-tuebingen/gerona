# GeRoNa examples

This package provides example launch files for GeRoNa.

## Stage


We provide two simple stage examples:
Both of them spawn a stage world with a kobuki and start GeRoNa plus RViz.
Use the navigation goal from RViz to test GeRoNa interactively.

### Examples

1. stage_example_odometry.launch
    Uses only the simulated odometry. No mapping is performed, which causes problems with obstacles behind the robot

2. stage_example_amcl.launch
    Uses a map and AMCL for full navigation

### Installation of requirements (Ubuntu)

Use rosdep to get all dependencies, or install them manually with
    sudo apt-get install ros-kinetic-turtlebot-stage
