#ifndef COURSEPREDICTOR_H
#define COURSEPREDICTOR_H

#include <Eigen/Core>
#include <ros/ros.h>

class PathFollower;

//! Predict direction of movement of the robot.
class CoursePredictor
{
public:
    CoursePredictor(PathFollower *path_driver);

    void update();
    void reset();

    //! Predict direction of movement
    Eigen::Vector2d predictDirectionOfMovement();

    /**
     * @brief Predict and smooth direction of movement.
     *
     * This prediction is only updated if the robot moved for at least a certain distance and it is smoothed to reduce
     * the effect of jitter in the robots movement.
     *
     * @return Vector pointing in the direction of movement, relative to robot orientation.
     */
    Eigen::Vector2d smoothedDirection();

    ros::Duration getUpdateIntervall() const;
    void setUpdateIntervall(const ros::Duration &getUpdateIntervall);

private:
    PathFollower *path_driver_;

    ros::Duration update_intervall_;

    //! Position of the robot (in world frame), when direction was updated the last time.
    Eigen::Vector2d last_position_;
    //! Time, when the direction prediction was updated the last time
    ros::Time last_update_time_;
    //! Position of the robot (in world frame), when the smoothed direction was updated the last time.
    Eigen::Vector2d last_position_smoothed_;
    //! smoothed direction of movement.
    Eigen::Vector2d smoothed_direction_;
};

#endif // COURSEPREDICTOR_H
