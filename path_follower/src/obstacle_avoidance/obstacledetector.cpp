#include <path_follower/obstacle_avoidance/obstacledetector.h>

ObstacleDetector::ObstacleDetector():
    use_map_(true),
    use_scan_(true)
{
}

void ObstacleDetector::avoid(tf::Vector3 * const cmd,
                             const ObstacleAvoider::ObstacleCloud &obstacles,
                             const ObstacleAvoider::State &state)
{
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


    // call the obstacle detector. it is dependent of the controller als different driving models may require different
    // handling
    bool collision = isObstacleAhead(state.parameters.collision_box_width(), box_length, course,
                                     enlarge_factor);

    if(collision) {
//        beep(beep::OBSTACLE_IN_PATH);
        // stop motion
        cmd->setZ(0.0);
    }

}

void ObstacleDetector::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
}

void ObstacleDetector::setScan(const sensor_msgs::LaserScanConstPtr &scan, bool isBack)
{
    if (isBack) {
        scan_back_ = scan;
    } else {
        scan_ = scan;
    }
}

void ObstacleDetector::setUseMap(bool use)
{
    use_map_ = use;
}

void ObstacleDetector::setUseScan(bool use)
{
    use_scan_ = use;
}

bool ObstacleDetector::isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor)
{
    // if there is neither map nor scan, be save and assume there is an obstacle, otherwise init with false.
    bool obstacle = !map_ && !scan_;

    if (use_map_) {
        if (!map_) {
            ROS_ERROR_THROTTLE(1, "ObstacleDetector: No map received.");
        } else {
            obstacle |= checkOnMap(width, length, course_angle, curve_enlarge_factor);
        }
    }

    if (use_scan_) {
        if (!scan_ && !scan_back_) {
            ROS_ERROR_THROTTLE(1, "ObstacleDetector: No scan received.");
        }
        if (scan_) {
            obstacle |= checkOnScan(scan_, width, length, course_angle, curve_enlarge_factor);
        }
        if (scan_back_) {
            obstacle |= checkOnScan(scan_back_, width, length, course_angle, curve_enlarge_factor);
        }
    }

    return obstacle;
}
