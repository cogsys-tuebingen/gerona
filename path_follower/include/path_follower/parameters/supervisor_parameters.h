#ifndef SUPERVISOR_PARAMETERS_H
#define SUPERVISOR_PARAMETERS_H

#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/parameters.h>
#include <rosconsole/macros_generated.h>

struct SupervisorParameters : public Parameters
{
    static const SupervisorParameters* getInstance()
    {
        static SupervisorParameters instance(PathFollowerParameters::getInstance());
        return &instance;
    }

    // supervisors
    P<bool> use_path_lookout;
    P<bool> use_waypoint_timeout;
    P<bool> use_distance_to_path;

    P<double> distance_to_path_max_dist;
    P<float> waypoint_timeout_time;

private:
    SupervisorParameters(const Parameters* parent):
        Parameters("supervisor", parent),

        use_path_lookout(this, "use_path_lookout",  false,
                                    "Set to `true` to activate supervisor _path lookout_ (check"
                                    " if there are obstacles somewhere on the path ahead of the"
                                    " robot)."),
        use_waypoint_timeout(this, "use_waypoint_timeout", false,
                                        "Set to `true` to activate supervisor _waypoint timeout_."),
        use_distance_to_path(this, "use_distance_to_path", false,
                                        "Set to `true` to activate supervisor _distance to path_."),

        distance_to_path_max_dist(this, "distance_to_path/max_dist",  10.0,
                                             "Maximum distance the robot is allowed to depart from the path."
                                             " If this threshold is exceeded, the distance_to_path superv will abort."),

        waypoint_timeout_time(this, "waypoint_timeout/time", 30.0,
                                         "If the robot needs more than this time (in seconds), supervisor"
                                         " WaypointTimeout will stop the path execution.")

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
    }
};

#endif // SUPERVISOR_PARAMETERS_H
