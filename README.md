# GeRoNa: Generic Robot Navigation

This project contains a set of [ROS](ros.org) packages which provide a navigation suite, including path planning on a given map, controlling of the robot to follow this path and obstacle avoidance while driving.

The packages are build with the goal to get a modular, easily extensible framework, where new modules (e.g. kinematic model of a new robot, new obstacle avoidance algorithms, etc) can be added with minimal effort.

The following video demonstrates various supported robot kinematics:

[![GeRoNa: Generic Robot Navigation Video](https://img.youtube.com/vi/Ppdi7dQ7Vzw/0.jpg)](https://www.youtube.com/watch?v=Ppdi7dQ7Vzw)

Installation
------------

First, install the dependencies. For example, from your workspace root directory:

    cd src
    git clone gitlab@gitlab.cs.uni-tuebingen.de:libs/cslibs_path_planning 
    git clone gitlab@gitlab.cs.uni-tuebingen.de:libs/cslibs_utils
    cd ..

    rosdep install --from-paths -i -y src
    
Then you can either use `catkin_make`, `catkin_make_isolated` or [catkin build](https://github.com/catkin/catkin_tools) to build the software.

Quick Start
-----------

_This is only a really quick "quick start" guide. For more detailed explanations, see below._

### Send goals from some ROS node
To start the complete navigation project as well as nodes for SLAM and obstacle detection (using 2d laser), simply run

    roslaunch navigation_launch navigation.launch

Now everything is set up and your node can connect to the _navigate_to_goal_ [action server](http://wiki.ros.org/actionlib) and send goals.

### Set goals manually using Rviz
If you want to set goal poses manually using Rviz (e.g. for testing or demonstration purposes), run

    roslaunch navigation_launch rviz_controlled.launch

### Select robot controller/model
By default a controller for car-like robots is used (_ackermann_purepursuit_). To change this, simply set the environment variable `ROBOT_CONTROLLER` with the name of the controller you want to use.
For example, to use the omni-drive orthexp controller:

    export ROBOT_CONTROLLER=omni_orthexp
    roslaunch navigation_launch navigation.launch

The controller can also be set via the ROS parameter

    rosparam set path_follower/controller_type omni_orthexp



More Information
----------------

For more information please refer to the [wiki](https://github.com/cogsys-tuebingen/gerona/wiki).

