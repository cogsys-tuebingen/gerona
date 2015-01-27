#ifndef DISTANCETOPATHSUPERVISOR_H
#define DISTANCETOPATHSUPERVISOR_H

#include <path_follower/supervisor/supervisor.h>
#include <path_follower/utils/visualizer.h>

/**
 * @brief Supervises the robot to stay near the path.
 *
 * This supervisor checks how far the robot is away from the current
 * path segment and tells it to stop, if the distance exceeds a defined
 * maximum.
 */
class DistanceToPathSupervisor : public Supervisor
{
public:
    DistanceToPathSupervisor(double max_distance_to_path);

    virtual std::string getName() const {
        return "DistanceToPath";
    }

    virtual void supervise(State &state, Result *out);

private:
    double max_dist_;
    Visualizer *visualizer_;

    //! Calculate the distance of the robot to the current path segment.
    double calculateDistanceToCurrentPathSegment(const State &state);
};

#endif // DISTANCETOPATHSUPERVISOR_H
