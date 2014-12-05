#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

// THIRD PARTY
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>

// PROJECT
#include <path_follower/utils/path.h>
#include <path_follower/obstacle_avoidance/obstacledetector.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>

class PathFollower;

class RobotController
{
    /* DATA */
public:
    //TODO: better use own struct to get clear variable names (x, y, velocity)
    typedef tf::Vector3 MoveCommand;

    enum ControlStatus
    {
        MOVING,
        OBSTACLE,
        SUCCESS,
        ERROR
    };

    /* ABSTRACT METHODS */
public:
    virtual void publishCommand() = 0;

    //! Immediatley stop any motion.
    virtual void stopMotion() = 0;

    virtual bool isOmnidirectional() const;

    virtual void start() {}

    virtual void behaveOnLine() {}

    /**
     * @return True, when turning point is reached, otherwise false.
     */
    virtual bool behaveApproachTurningPoint() { return false; }

protected:
    /**
     * @brief Computes the next move command.
     * @param cmd Output. The move command for this iteration.
     * @return Status that can be used to indicate errors. `MOVING` if everything is ok.
     */
    virtual ControlStatus computeMoveCommand(MoveCommand* cmd) = 0;

    //! Converts the move command to ros message and publishs it.
    virtual void publish(const MoveCommand &cmd) const = 0;


    /* REGULAR METHODS */
public:
    RobotController(PathFollower *path_driver) :
        path_driver_(path_driver),
        velocity_(0.0f),
        filtered_speed_(0.0f),
        dir_sign_(1.0f)
    {
        initPublisher(&cmd_pub_);
    }

    //! Execute one iteration of path following. This method should not be overwritten by subclasses!
    /*final*/ ControlStatus execute();

    /* RESET FOR A NEW PATH */
    virtual void reset() {}


    virtual void setPath(Path::Ptr path);

    /* BEHAVIOURS */
    //! Initialize the OnLine-Behaviour
    virtual void initOnLine() {}
    //! Initialize the ApproachTurningPoint-Behaviour
    virtual void initApproachTurningPoint() {}

    virtual void setVelocity(float v)
    {
        ROS_WARN_STREAM("setting velocity to " << v);
        velocity_ = v;
    }

    virtual void setDirSign(float s)
    {
        dir_sign_ = s;
    }

    virtual float getDirSign() const
    {
        return dir_sign_;
    }

protected:
    ros::Publisher cmd_pub_;

    PathFollower* path_driver_;

    //! Desired velocity (defined by the action goal).
    float velocity_;

    float filtered_speed_;

    //! Indicates the direction of movement (>0 -> forward, <0 -> backward)
    float dir_sign_;

    //! Current path.
    Path::Ptr path_;
    //! The next waypoint in the robot frame (set by setPath).
    Eigen::Vector3d next_wp_local_;


    virtual void initPublisher(ros::Publisher* pub) const;


    virtual void setFilteredSpeed( const float speed ) {
        filtered_speed_ = speed;
    }

    virtual float getFilteredSpeed() const {
        return filtered_speed_;
    }

    void setStatus(int status);

    //! Calculate the angle between the orientations of the waypoint and the robot.
    virtual double calculateAngleError();
};

#endif // ROBOTCONTROLLER_H
