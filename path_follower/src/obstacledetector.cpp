#include "obstacledetector.h"

#include <Eigen/Core>
//#include <opencv2/opencv.hpp> // only for debugging
#include "visualizer.h"

using namespace Eigen;

ObstacleDetector::ObstacleDetector()
{
}

void ObstacleDetector::gridMapCallback(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;
}

bool ObstacleDetector::isObstacleAhead(float width, float length, float course_angle, float curve_enlarge_factor) const
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
     * For course_angle = 0, the parallelogram is a rectangle. If course_angle != 0, a and b stay constant, but c is moved to the side,
     * depending on course_angle:
     *
     * course_angle = 0:
     *          p------------------r
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          q------------------
     *   ^robot    <-  length  ->
     *
     *
     * course_angle < 0:
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
     * fixed. For course_angle > 0 it is vice verca.
     * The amount of this enlarging is controlled by the argument 'curve_enlarge_factor'.
     */


    bool collision = false;


//    cv::Mat debug(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar::all(255));

    // scale to map
    width  /= map_->info.resolution;
    length /= map_->info.resolution;
    curve_enlarge_factor /= map_->info.resolution;

    Vector2f o(-map_->info.origin.position.x / map_->info.resolution,
               -map_->info.origin.position.y / map_->info.resolution);
    Vector2f p = o + Vector2f(0.0f, width/2.0f);
    Vector2f q = o - Vector2f(0.0f, width/2.0f);

    float sin_angle = std::sin(course_angle);
    float cos_angle = std::cos(course_angle);

    if (course_angle > 0) {
        p(1) += curve_enlarge_factor * sin_angle;
    } else if (course_angle < 0) {
        q(1) += curve_enlarge_factor * sin_angle;
    }

    Vector2f r = p + length * Vector2f(cos_angle, sin_angle);

    Vector2f pq = q - p;
    Vector2f pr = r - p;

    float det_pq_pr = pq(0)*pr(1) - pq(1)*pr(0);

    const unsigned data_size = map_->info.height * map_->info.width;
    for (unsigned i = 0; i < data_size; ++i) {
        /* for debugging
        if (map_->data[i] == OCCUPIED)
            debug.data[i] = 127;

        // check if this map point is inside the parallelogram

        Vector2f pa = Vector2f( i % map_->info.width, i / map_->info.width ) - p;

        float det_pa_pq = pa(0)*pq(1) - pa(1)*pq(0);
        float det_pa_pr = pa(0)*pr(1) - pa(1)*pr(0);

        float check_1 = -det_pa_pq / det_pq_pr;
        float check_2 =  det_pa_pr / det_pq_pr;

        if (0 <= check_1 && check_1 <= 1  &&  0 <= check_2 && check_2 <= 1) {
            debug.data[i] = 0;
            if (map_->data[i] == OCCUPIED)
                collision = true;
        }
        */


        if (map_->data[i] == OCCUPIED) {
            // check if this map point is inside the parallelogram

            Vector2f pa = Vector2f( i % map_->info.width, i / map_->info.width ) - p;

            float det_pa_pq = pa(0)*pq(1) - pa(1)*pq(0);
            float det_pa_pr = pa(0)*pr(1) - pa(1)*pr(0);

            float check_1 = -det_pa_pq / det_pq_pr;
            float check_2 =  det_pa_pr / det_pq_pr;

            if (0 <= check_1 && check_1 <= 1  &&  0 <= check_2 && check_2 <= 1) {
                collision = true;
                break; // no need to check the remaining cells
            }
        }
    }

    // visualization
    Visualizer* vis = Visualizer::getInstance();
    if (vis->hasSubscriber()) {
        // transform
        Vector2f e_p1 = (p - o)*map_->info.resolution;
        Vector2f e_p2 = (q - o)*map_->info.resolution;
        Vector2f e_p3 = (r - o)*map_->info.resolution;
        Vector2f e_p4 = (p + pr + pq - o)*map_->info.resolution;
        // convert Eigen vectors to ros points...
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = e_p1(0);  p1.y = e_p1(1);
        p2.x = e_p2(0);  p2.y = e_p2(1);
        p3.x = e_p3(0);  p3.y = e_p3(1);
        p4.x = e_p4(0);  p4.y = e_p4(1);

        // colour is green when the bos is empty and red, if there is an obstacle
        float r = collision ? 1 : 0;
        float g = 1 - r;
        vis->drawLine(1, p1, p2, "laser", "collision_box", r,g,0, 3, 0.05);
        vis->drawLine(2, p2, p4, "laser", "collision_box", r,g,0, 3, 0.05);
        vis->drawLine(3, p1, p3, "laser", "collision_box", r,g,0, 3, 0.05);
        vis->drawLine(4, p3, p4, "laser", "collision_box", r,g,0, 3, 0.05);
    }

//    cv::line(debug, cv::Point((int)p(0),(int)p(1)), cv::Point((int)q(0), (int)q(1)), cv::Scalar(0));
//    cv::line(debug, cv::Point((int)p(0),(int)p(1)), cv::Point((int)r(0), (int)r(1)), cv::Scalar(0));

//    cv::imshow("ObstacleBox", debug);

    return collision;

}
