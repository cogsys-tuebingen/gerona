#ifndef COURSEPREDICTOR_H
#define COURSEPREDICTOR_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <path_follower/utils/parameters.h>

class PoseTracker;

//! Predict direction of movement of the robot.
class CoursePredictor
{
public:
    CoursePredictor(PoseTracker *pose_tracker);

    void update();
    void reset();

    //! Freeze update. No new points will be added -> direction will not change.
    void freeze()
    {
        frozen_ = true;
    }

    //! Deaktivate freeze.
    void unfreeze()
    {
        frozen_ = false;
    }

    //! Predict direction of movement
    Eigen::Vector2d predictDirectionOfMovement() const;

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
    Eigen::Vector2d smoothedDirection() const;

    ros::Duration getUpdateIntervall() const;
    void setUpdateIntervall(const ros::Duration &getUpdateIntervall);

private:
    typedef boost::circular_buffer<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > buffer_type;

    struct CoursePredictorParameters : public Parameters
    {
        WrappedP<ros::Duration, float> update_interval;
        P<int> buffer_size;

        CoursePredictorParameters():
            Parameters("coursepredictor"),

            update_interval(this, "update_interval", 0.1f, "Update interval (in sec) of the course prediction."),
            buffer_size(this, "buffer_size", 5, "Number of last positions that are kept in the buffer.")
        {
            if (buffer_size() < 2) {
                ROS_ERROR("Course Predictor: Buffer size must be at least 2 but is set to %d. Course prediction will not work!",
                          buffer_size());
            }
        }
    } opt_;

    PoseTracker *pose_tracker_;

    //! List of last known positions. The most recent position is pushed to the back.
    buffer_type last_positions_;

    //! Time, when the direction prediction was updated the last time
    ros::Time last_update_time_;

    bool frozen_;
};

#endif // COURSEPREDICTOR_H
