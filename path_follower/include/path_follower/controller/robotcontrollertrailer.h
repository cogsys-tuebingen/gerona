#ifndef RobotControllerTrailer_H
#define RobotControllerTrailer_H

/// SYSTEM
#include <tf/transform_listener.h>
#include <string>
/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/pidcontroller.hpp>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/visualizer.h>

/// FORWARD CLASS DEFINITIONS
class PathController;

/**
 * @brief PID controller for robots with car-like/Ackermann drive.
 */
class RobotControllerTrailer : public RobotController
{
public:
    RobotControllerTrailer(PathFollower *path_driver, ros::NodeHandle *nh);
    virtual void stopMotion();
    virtual void reset();
    virtual void setPath(Path::Ptr path);
    virtual void precomputeSteerCommand(Waypoint& wp_now,  Waypoint& wp_next );
    /**
         * @brief isOmnidirectional
         * @return true ***hack to avoid bugs in local_plannr_transformer with multiple subpaths
         */
    virtual bool isOmnidirectional() const {return false;}

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
        P<float> fwd_cap_steer_deg;

        P<float> bwd_cap_steer_deg;
        P<float> max_steer;
        P<float> weight_dist;
        P<float> weight_angle;

        P<std::string> controller_type;

        ControllerParameters():
            dead_time(this, "~dead_time", 0.1, "Time step that is used by predictPose"),
            l(this, "~wheel_base", 0.98, "Distance between front and rear axes of the robot."),
            pid_ta(this, "~pid/ta", 0.03, "Update interval of the PID controller."),

            fwd_cap_steer_deg(this, "~fwd/cap_steer_deg", 5.0, "Maxiimum allowed deviation from precomputed angle."),
            bwd_cap_steer_deg(this, "~bwd/cap_steer_deg", 5.0, "Maxiimum allowed deviation from precomputed angle."),
            max_steer(this, "~max_steer", 1.3, "Maximal allowed steering angle. Higher angles are capped by this value."),
            weight_dist(this, "~pc_weight_dist", 1.0, "Weight of distance error"),
            weight_angle(this, "~pc_weight_angle", 1.0, "Weight of angle error"),
            controller_type(this,"~controller_type","simple","simple or cascade controller")


        {}
    } opt_;
    ros::NodeHandle *nh_;
    PathController *path_ctrl_;
    ros::Subscriber agv_vel_sub_;
    ros::Publisher agv_steer_is_pub_, agv_steer_set_pub_, agv_error_angle_pub_,agv_error_dist_pub_;
    //! The current move command.
    MoveCommand cmd_;

    //! current velocity and trailer angle
    geometry_msgs::Twist agv_vel_;


    // current precomputed steering angles
    double steer_des_fwd_,steer_des_bwd_;

    //! PID controller for the steering angle

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
    void updateCommand(float dist_error, float angle_error);

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

    double calcTotalError (double dist_err,double angle_err );

    /**
     * @brief Calculates the (signed) sideways distance to the waypoint.
     *
     * The sideways distance is the distance of the waypoint to the line that goes through the
     * robot in the direction of its orientation.
     * This is meant as an error measure and thus is signed, depending on if the waypoint lies
     * left or right of the robot.
     */
    double calculateSidewaysDistanceError() const;


    double calcAngleError (const Eigen::Vector2d &front_pred, const Eigen::Vector2d &rear_pred) const;

    void visualizeCarrot(const Eigen::Vector2d &carrot, int id, float r, float g, float b) const;



    bool getTrailerAngle(double& angle) const;


    void updateAgvCb(const geometry_msgs::TwistConstPtr &vel);


    virtual double calculateAngleError();

    tf::TransformListener trailer_listener_;


};

#endif // RobotControllerTrailer_H
