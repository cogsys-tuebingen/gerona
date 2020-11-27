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

Remove the CATKIN_IGNORE file in gerona_examples

Use rosdep to get all dependencies, or install them manually with

    sudo apt-get install ros-kinetic-turtlebot-stage

To use rosdep, run the following

    rosdep install --from-paths -i -r -y <path-to-src>


## Gazebo

We provide two gazebo examples that use skid steering robots.
To run the example, install the following dependencies:

Use rosdep to get all dependencies, or install them manually with
    sudo apt-get install ros-kinetic-summit-xl-sim

Clone the latest version of the following repositories into your workspace:

    git clone https://github.com/RobotnikAutomation/robotnik_msgs.git
    git clone https://github.com/RobotnikAutomation/robotnik_sensors.git
    git clone https://github.com/RobotnikAutomation/summit_xl_sim.git
    git clone https://github.com/RobotnikAutomation/summit_xl_common.git
    git clone https://github.com/rst-tu-dortmund/costmap_prohibition_layer
    git clone https://github.com/ros-perception/slam_gmapping.git
    git clone https://github.com/ros-perception/openslam_gmapping.git
    git clone https://github.com/Gastd/p3at_tutorial

> The first startup of gazebo might take a while as gazebo downloads missing models.

### Examples

1. gazebo_example_summit.launch

Summit XL simulation, shows how to remap topics, tf frames and groups.

> There is a problem with the summit simulation in older gazebo versions: friciton pyramid makes skid steering inhomogeneous.
> Depending on the absolute orientation of the robot, the skidding behaves differently.
> To test if your gazebo version has this problem, simply publish a static command to the robot:

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
          x: 0.3
          y: 0.0
          z: 0.0
        angular:
          x: 0.0
          y: 0.0
          z: 0.50" -r 10

> If the robot is not moving in a circle, a useful behavior of GeRoNa cannot be expected.

2. gazebo_example_pioneer.launch

Pioneer simulation and full GeRoNa stack demonstration.

3. gazebo_example_pioneer_pioneer_follower_only.launch

Pioneer simulation and demonstration where only the path follower is used.
A secondary node (`follow_path_example.py`) is used to convert `nav_msgs/Path` from [`global_planner`](http://wiki.ros.org/global_planner) into `path_msgs/DirectionalPath`s.

4. gazebo_example_summit_mbc.launch

Demonstrates the use of local planning on a elevation map only based only on the RGB-D sensor.
In order to run the example the "summit_xl_rgbd_down.urdf.xacro" file from "gerona_examples/robots" needs to be copied to
"summit_xl_description/robots" folder in the summit_xl_common package.

