#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <string>
#include <memory> // std::shared_ptr
#include <Eigen/Core>

#include <path_msgs/FollowPathFeedback.h>
#include <path_msgs/FollowPathResult.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/obstaclecloud.hpp>

class Supervisor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Supervisor> Ptr;

    struct State
    {
        typedef std::shared_ptr<State> Ptr;

        State(const Eigen::Vector3d &robot_pose,
              Path::ConstPtr path,
              ObstacleCloud::ConstPtr obstacle_cloud,
              path_msgs::FollowPathFeedback &feedback):
            robot_pose(robot_pose),
            path(path),
            obstacle_cloud(obstacle_cloud),
            feedback(feedback)
        {}

        //! Current pose of the robot in world frame. The first two values are x/y-coords, the third is the yaw angle.
        const Eigen::Vector3d& robot_pose;
        //! The current path
        const Path::ConstPtr path;
        //! The last obstacle cloud
        const ObstacleCloud::ConstPtr obstacle_cloud;

        path_msgs::FollowPathFeedback &feedback;
    };

    struct Result
    {
        Result():
            can_continue(true),
            status(path_msgs::FollowPathResult::MOTION_STATUS_MOVING)
        {}

        bool can_continue;
        //! Status code, see FollowPathResult. Only used if can_continue is false.
        int8_t status;
    };

    virtual void supervise(State &state, Result *out) = 0;

    virtual std::string getName() const = 0;

    virtual void eventNewGoal() {}
    virtual void eventNewWaypoint() {}
};

#endif // SUPERVISOR_H
