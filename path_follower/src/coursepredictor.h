#ifndef COURSEPREDICTOR_H
#define COURSEPREDICTOR_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

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
     * The direction is smoothed by sampling a small number of recent positions and fitting a line to them.
     * Please note: at the beginning, when there are less than 2 position samples, a zero vector is returned. Thus make
     * sure always to check the result using `result.isZero()`, befor using the vector.
     *
     * @return Vector pointing in the direction of movement, relative to robot orientation. Zero, if no direction could
     *         be computed (e.g. at initialization, when there are less than 2 position samples).
     */
    Eigen::Vector2d smoothedDirection();

    ros::Duration getUpdateIntervall() const;
    void setUpdateIntervall(const ros::Duration &getUpdateIntervall);

private:
    typedef boost::circular_buffer<Eigen::Vector2d> buffer_type;

    PathFollower *path_driver_;

    ros::Duration update_intervall_;

    //! List of last known positions. The most recent position is pushed to the back.
    buffer_type last_positions_;

    //! Time, when the direction prediction was updated the last time
    ros::Time last_update_time_;

    void configure();
};

#endif // COURSEPREDICTOR_H
