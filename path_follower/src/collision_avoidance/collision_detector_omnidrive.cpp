#include <path_follower/collision_avoidance/collision_detector_omnidrive.h>
#include <algorithm>

using namespace std;

CollisionDetectorPolygon::PolygonWithTfFrame CollisionDetectorOmnidrive::getPolygon(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    PolygonWithTfFrame pwf;
    pwf.frame = "base_link";

    // rotate about course_angle (box should point in driving direction)
    tf::Transform rot(tf::Quaternion(tf::Vector3(0,0,1), course_angle));

    vector<tf::Point> corners;
    corners.push_back(rot * tf::Point(0.0,     width/2, 0.0));
    corners.push_back(rot * tf::Point(0.0,    -width/2, 0.0));
    corners.push_back(rot * tf::Point(length, -width/2, 0.0));
    corners.push_back(rot * tf::Point(length,  width/2, 0.0));

    for (vector<tf::Point>::iterator it = corners.begin(); it != corners.end(); ++it) {
        pwf.polygon.push_back( cv::Point2f(it->x(), it->y()) );
    }

    return pwf;
}
