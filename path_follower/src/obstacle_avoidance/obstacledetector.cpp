#include <path_follower/obstacle_avoidance/obstacledetector.h>
#include <path_follower/utils/obstacle_cloud.h>

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "obstacle_avoider";
}

bool ObstacleDetector::avoid(MoveCommand * const cmd,
                             const ObstacleAvoider::State &state)
{
    float course = cmd->getDirectionAngle(); //TODO: use CoursePredictor instead of command?

    //! Factor which defines, how much the box is enlarged in curves.
    const float enlarge_factor = 0.5; // should this be a parameter?

    /* Calculate length of the collision box, depending on current velocity.
     * v <= v_min:
     *   length = min_length
     * v > v_min && v < v_sat:
     *   length  interpolated between min_length and max_length:
     *   length = min_length + FACTOR * (max_length - min_length) * (v - v_min) / (v_sat - v_min)
     * v >= v_sat:
     *   length = max_length
     */
    float v = cmd->getVelocity();

    const float diff_to_min_velocity = v - state.parameters.min_velocity();

    float vel_saturation = opt_.velocity_saturation() > 0
            ? opt_.velocity_saturation()
            : state.parameters.max_velocity();
    const float norm = vel_saturation - state.parameters.min_velocity();
    const float span = opt_.max_length() - opt_.min_length();
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, opt_.velocity_factor() * interp);

    float box_length = opt_.min_length() + span * f;

    //ROS_DEBUG_NAMED(MODULE, "Collision Box: v = %g -> len = %g", v, box_length);

    double distance_to_goal = state.path->getCurrentSubPath().wps.back().distanceTo(state.path->getCurrentWaypoint());

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < opt_.crit_length()) {
        box_length = opt_.crit_length();
    }


    bool collision = false;
    if(obstacles_ && !obstacles_->empty()) {
        collision = checkOnCloud(obstacles_, opt_.width(),
                                      box_length, course, enlarge_factor);
    } else if (!obstacles_) {
        ROS_WARN_THROTTLE(1, "no obstacle cloud is available");
    }


    if(collision) {
        // stop motion
        cmd->setVelocity(0);
    }
    return collision;
}
