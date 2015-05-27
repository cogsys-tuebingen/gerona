
#include <path_follower/obstacle_avoidance/obstacledetectorpatsy.h>

#include <Eigen/Core>
//#include <opencv2/opencv.hpp> // only for debugging
#include <path_follower/utils/visualizer.h>

using namespace Eigen;


ObstacleDetectorPolygon::PolygonWithTfFrame ObstacleDetectorPatsy::getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    /// Based on http://stackoverflow.com/questions/1217585/parallelogram-contains-point

    /*
     * The parallelogram is defined by four points p,q,r,s:
     * p and q are the corners near the robot, r and s are on the opposite site.
     *
     * For course_angle = 0, the parallelogram is a rectangle. Ifcourse_angle != 0, a and b
     * stay constant, but c is moved to the side, depending on course_angle:
     *
     * course_angle = 0:
     *          p------------------r
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          q------------------s
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
     *                       +--s
     *
     * The Box is enlarged toward the inside curve. In the above example, q is moved away from
     * the robot, while p is fixed. For course_angle > 0 it is vice verca.
     * The amount of this enlarging is controlled by the argument 'curve_enlarge_factor'.
     */


    Vector2f p(0.0f,  width/2.0f);
    Vector2f q(0.0f, -width/2.0f);

    course_angle *= 0.5f;

    float sin_angle = std::sin(course_angle);
    float cos_angle = std::cos(course_angle);

    // stretch box sidewards, depending on course angle
    if (course_angle > 0) {
        p(1) += curve_enlarge_factor * sin_angle;
    } else if (course_angle < 0) {
        q(1) += curve_enlarge_factor * sin_angle;
    }

    Vector2f r = p + length * Vector2f(cos_angle, sin_angle);
    Vector2f s = q + length * Vector2f(cos_angle, sin_angle);

    PolygonWithTfFrame pwf;
    pwf.frame = "/base_link";

    pwf.polygon.push_back( cv::Point2f(p[0], p[1]) );
    pwf.polygon.push_back( cv::Point2f(q[0], q[1]) );
    pwf.polygon.push_back( cv::Point2f(s[0], s[1]) );
    pwf.polygon.push_back( cv::Point2f(r[0], r[1]) );

    return pwf;
}
