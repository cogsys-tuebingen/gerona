#ifndef ROBOTCONTROLLER_POTENTIAL_FIELD_TT_H
#define ROBOTCONTROLLER_POTENTIAL_FIELD_TT_H

/// THIRD PARTY
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

/// SYSTEM
#include <pcl_ros/point_cloud.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller_potential_field.h>


/// The Potential_Field class
class RobotController_Potential_Field_TT : public RobotController_Potential_Field
{
public:
    RobotController_Potential_Field_TT();

protected:

    /// Sets the goal position.
    virtual void setGoalPosition();


};

#endif // ROBOTCONTROLLER_POTENTIAL_FIELD_TT_H
