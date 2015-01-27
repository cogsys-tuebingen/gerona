#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

// THIRD PARTY
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>

// PROJECT
#include <path_follower/utils/path.h>
#include <path_follower/utils/movecommand.h>
#include <path_follower/obstacle_avoidance/obstacledetector.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>

class PathFollower;

class RobotController
{
    /* DATA */
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ControlStatus
    {
        OKAY,
        OBSTACLE,
        REACHED_GOAL,
        ERROR
    };

    /* ABSTRACT METHODS */
public:
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
    //! This is a subset of ControlStatus. computeMoveCommand is not allowed to report obstacles
    enum MoveCommandStatus
    {
        MC_OKAY, MC_REACHED_GOAL, MC_ERROR
    };

    /**
     * @brief Computes the next move command.
     * @param cmd Output. The move command for this iteration.
     * @return Status that can be used to indicate errors. `OKAY` if everything is ok.
     */
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd) = 0;

    //! Converts the move command to ros message and publishs it.
    virtual void publishMoveCommand(const MoveCommand &cmd) const = 0;


    /* REGULAR METHODS */
public:
    RobotController(PathFollower *path_driver) :
        path_driver_(path_driver),
        velocity_(0.0f),
        dir_sign_(1.0f)
    {
        initPublisher(&cmd_pub_);
    }

    virtual ~RobotController() {}

    //! Execute one iteration of path following. This method should not be overwritten by subclasses!
    ControlStatus execute();

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

    //! Indicates the direction of movement (>0 -> forward, <0 -> backward)
    float dir_sign_;

    //! Current path.
    Path::Ptr path_;
    //! The next waypoint in the robot frame (set by setPath).
    Eigen:: Vector3d next_wp_local_;


    virtual void initPublisher(ros::Publisher* pub) const;

    void setStatus(int status);

    //! Calculate the angle between the orientations of the waypoint and the robot.
    virtual double calculateAngleError();

    //! Convert a MoveCommandStatus to its corresponding ControlStatus
    static ControlStatus MCS2CS(MoveCommandStatus s);
};

#endif // ROBOTCONTROLLER_H
