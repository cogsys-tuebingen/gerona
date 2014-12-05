#include <path_follower/obstacle_avoidance/obstacledetector.h>

bool ObstacleDetector::avoid(tf::Vector3 * const cmd,
                             ObstacleCloud::ConstPtr obstacles,
                             const ObstacleAvoider::State &state)
{
#warning this has to be adjusted to the obstacle cloud!

    float course = atan2(cmd->y(), cmd->x()); //FIXME: use course prediction!

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
    float v = cmd->z();//current_command_.velocity;

    const float diff_to_min_velocity = v - state.parameters.min_velocity();

    const float norm = state.parameters.collision_box_velocity_saturation() - state.parameters.min_velocity();
    const float span = state.parameters.collision_box_max_length() - state.parameters.collision_box_min_length();
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, state.parameters.collision_box_velocity_factor() * interp);

    float box_length = state.parameters.collision_box_min_length() + span * f;

    //ROS_DEBUG("Collision Box: v = %g -> len = %g", v, box_length);

    double distance_to_goal = state.path->getCurrentSubPath().back().distanceTo(state.path->getCurrentWaypoint());

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < state.parameters.collision_box_crit_length()) {
        box_length = state.parameters.collision_box_crit_length();
    }


    bool collision = checkOnCloud(obstacles, state.parameters.collision_box_width(),
                                  box_length, course, enlarge_factor);


    if(collision) {
//        beep(beep::OBSTACLE_IN_PATH);

        // stop motion
        cmd->setZ(0.0);
    }
    return collision;
}
