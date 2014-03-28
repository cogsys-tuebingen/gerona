#include "obstacledetector.h"

#include <Eigen/Core>

using namespace Eigen;

ObstacleDetector::ObstacleDetector()
{
}

void ObstacleDetector::gridMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
}

bool ObstacleDetector::isObstacleAhead(float width, float length, float steering_angle, float curve_enlarge_factor) const
{
    if (!map_) {
        ROS_ERROR_THROTTLE(1, "ObstacleDetector: No map received.");
        return true;
    }

    /// More or less copied from LaserEnvironment::CheckCollision()

    /// Based on http://stackoverflow.com/questions/1217585/parallelogram-contains-point

    /*
     * The parallelogram is defined by three points p,q,rc:
     * p and q are the corners near the robot, r is on the opposite site. The fourth corner is implicitly defined by p,q,r.
     *
     * For steering_angle = 0, the parallelogram is a rectangle. If steering_angle != 0, a and b stay constant, but c is moved to the side,
     * depending on steering_angle:
     *
     * steering_angle = 0:
     *          p------------------r
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          q------------------
     *   ^robot    <-  length  ->
     *
     *
     * steering_angle < 0:
     *          p---+
     *  ##   ## |    +--+
     *  ####### |        +--+
     *  ####### |            +--r
     *  ##   ## |               |
     *          |               |
     *          |               |
     *          q---+           |
     *               +--+       |
     *                   +--+   |
     *                       +--
     *
     * The Box is enlarged toward the inside curve. In the above example, b is moved away from the robot, while a is
     * fixed. For steering_angle > 0 it is vice verca.
     * The amount of this enlarging is controlled by the argument 'curve_enlarge_factor'.
     */


    Vector2f p(0.0f, width/2.0f);
    Vector2f q = -p;

    float sin_angle = std::sin(steering_angle);
    float cos_angle = std::cos(steering_angle);

    if (steering_angle > 0) {
        p(1) += curve_enlarge_factor * sin_angle;
    } else if (steering_angle < 0) {
        q(1) += curve_enlarge_factor * sin_angle;
    }

    Vector2f r = p + length * Vector2f(cos_angle, sin_angle);

    Vector2f pq = q - p;
    Vector2f pr = r - p;

    float det_pq_pr = pq(0)*pr(1) - pq(1)*pr(0);

    const unsigned data_size = map_->info.height * map_->info.width;
    for (unsigned i = 0; i < data_size; ++i) {
        ROS_INFO("OM cell %d = %d", i, map_->data[i]);
        if (map_->data[i] == OCCUPIED) {
            // check if this map point is inside the parallelogram

            Vector2f pa( i % map_->info.width, i / map_->info.width );

            float det_pa_pq = pa(0)*pq(1) - pa(1)*pq(0);
            float det_pa_pr = pa(0)*pr(1) - pa(1)*pr(0);

            float check_1 = -det_pa_pq / det_pq_pr;
            float check_2 =  det_pa_pr / det_pq_pr;

            if (0 <= check_1 && check_1 <= 1  &&  0 <= check_2 && check_2 <= 1) {
                return true;
            }
        }
    }

    return false;
}
