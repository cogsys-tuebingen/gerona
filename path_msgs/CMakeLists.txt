cmake_minimum_required(VERSION 2.8.3)
project(path_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
)

add_message_files(
  FILES
    Goal.msg
    Obstacle.msg
    DirectionalPath.msg
    PathSequence.msg
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
    FollowPath.action
    NavigateToGoal.action
    PlanAvoidance.action
    PlanPath.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
   CATKIN_DEPENDS
     actionlib_msgs
     geometry_msgs
     message_runtime
     nav_msgs
)




file(GLOB_RECURSE message_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS msg/*.msg action/*.action)
add_custom_target(${PROJECT_NAME}_show_messages SOURCES ${message_files})
