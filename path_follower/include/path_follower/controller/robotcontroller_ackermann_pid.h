#ifndef ROBOTCONTROLLER_ACKERMANN_PID_H
#define ROBOTCONTROLLER_ACKERMANN_PID_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/pidcontroller.hpp>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/visualizer.h>

/**
 * @brief PID controller for robots with car-like/Ackermann drive.
 */
class RobotController_Ackermann_Pid : public RobotController
{
public:
    RobotController_Ackermann_Pid(PathFollower *path_driver);
    virtual void stopMotion();
    virtual void reset();

protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
    //! Specific parameters of this controller.
    struct ControllerParameters : public Parameters
    {
        P<double> dead_time;
        P<double> l;
        P<float> pid_ta;
        P<float> pid_kp;
        P<float> pid_ki;
        P<float> pid_kd;
        P<float> max_steer;

        ControllerParameters():
            dead_time(this, "~dead_time", 0.1, "Time step that is used by predictPose"),
            l(this, "~l", 0.38, "Distance between front and rear axes of the robot."),
            pid_ta(this, "~pid/ta", 0.03, "Update interval of the PID controller."),
            pid_kp(this, "~pid/kp", 1.0, "Proportional coefficient of the PID controller."),
            pid_ki(this, "~pid/ki", 0.001, "Integral coefficient of the PID controller."),
            pid_kd(this, "~pid/kd", 0, "Derivative coefficient of the PID controller."),
            max_steer(this, "~max_steer", 0.52, "Maximal allowed steering angle. Higher angles are capped by this value.")
        {}
    } opt_;


    //! The current move command.
    MoveCommand cmd_;

    //! PID controller for the steering angle
    PidController<1> steer_pid_;

    Visualizer* visualizer_;

    /**
     * \brief The current behaviour of the controller.
     *
     * ON_PATH: The robot is driving on a sub path. This is the default case.
     * APPROACH_SUBPATH_END: The robot is approaching the last waypoint of a sub path.
     * WAIT_FOR_STOP: The robot reached the end of a sub path and is now waiting until motion
     *     stopped (i.e. measured velocity is below some tolerance threshold).
     */
    enum {
        ON_PATH, APPROACH_SUBPATH_END, WAIT_FOR_STOP
    } behaviour_;

    //! Velocity of the last published move command (used for pose prediction)
    mutable float last_velocity_; // mutable, so it can be set by publishMoveCommand()

    /**
     * @brief Select the waypoint for the current iteration.
     *
     * Checks if the current waypoint is already reached. If yes, switch to the next waypoint,
     * otherwise stick with the current waypoint.
     * The selected waypoint can be accessed via next_wp_local_.
     */
    void selectWaypoint();

    /**
     * @brief Compute controller inputs (error and velocity) for the ON_PATH behaviour.
     *
     * @param error Error of the current state to the desired state.
     * @param velocity Desired velocity.
     */
    float getErrorOnPath();

    /**
     * @brief Compute controller inputs (error and velocity) for the APPROACH_SUBPATH_END behaviour.

     * @param error Error of the current state to the desired state.
     * @param velocity Desired velocity.
     */
    float getErrorApproachSubpathEnd();

    /**
     * @brief Calls the PID controller and updates `cmd_`
     * @param error Error between actual and desired state. Used as input for the PID controller.
     */
    void updateCommand(float error);

    /**
     * @brief Determines the velocity for the next command.
     *
     * The actual velocity is based on the desired velocity, but can be influenced by several
     * factors. It can be reduced in tight curves, when near the goal or when driving backward.
     * Further, this method makes sure, the final velocity stays inside the interval of the
     * defined minimum and maximum velocity.
     *
     * @param steer_angle The current steering angle. For high angles, the velocity is reduced.
     * @return The final velocity that can be used for the move command. It is guaranteed to be
     *         within the interval of the defined minimum and maximum velocity.
     */
    float controlVelocity(float steer_angle) const;

    //! Distance of the robot to the Waypoint `wp`.
    double distanceToWaypoint(const Waypoint& wp) const;

    //! Predict pose of the robot in the next time step(?)
    void predictPose(Eigen::Vector2d &front_pred, Eigen::Vector2d &rear_pred) const;

    //! Calculate error in the robots positon to the desired position on the line of the current
    //! path segment.
    double calculateLineError() const;

    /**
     * @brief Calculates the (signed) sideways distance to the waypoint.
     *
     * The sideways distance is the distance of the waypoint to the line that goes through the
     * robot in the direction of its orientation.
     * This is meant as an error measure and thus is signed, depending on if the waypoint lies
     * left or right of the robot.
     */
    double calculateSidewaysDistanceError() const;

    void visualizeCarrot(const Eigen::Vector2d &carrot, int id, float r, float g, float b) const;
};

#endif // ROBOTCONTROLLER_ACKERMANN_PID_H
