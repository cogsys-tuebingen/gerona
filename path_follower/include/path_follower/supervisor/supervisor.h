#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include <path_msgs/FollowPathFeedback.h>
#include <path_msgs/FollowPathResult.h>
#include <path_follower/utils/path.h>

class Supervisor
{
public:
    typedef boost::shared_ptr<Supervisor> Ptr;

    struct State
    {
        typedef boost::shared_ptr<State> Ptr;

        State(const Eigen::Vector3d &robot_pose, Path::ConstPtr path, path_msgs::FollowPathFeedback &feedback):
            robot_pose(robot_pose),
            path(path),
            feedback(feedback)
        {}

        const Eigen::Vector3d& robot_pose;
        const Path::ConstPtr path;

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
