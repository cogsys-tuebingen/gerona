#include "obstacledetectorpolygon.h"

#include <opencv2/imgproc/imgproc.hpp>
#include "visualizer.h"

using namespace std;

bool ObstacleDetectorPolygon::checkForObstacle(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    bool collision = false;


    // scale to map
    width  /= map_->info.resolution;
    length /= map_->info.resolution;
    curve_enlarge_factor /= map_->info.resolution;

    vector<cv::Point> polygon = getPolygon(width, length, course_angle, curve_enlarge_factor);

    if (polygon.size() == 0) {
        ROS_WARN("Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }


    // FIXME: rotate about course_angle

    const unsigned data_size = map_->info.height * map_->info.width;
    for (unsigned i = 0; i < data_size; ++i) {

        if (map_->data[i] == OCCUPIED) {
            // check if this map point is inside the polygon
            cv::Point point( i % map_->info.width, i / map_->info.width );
            if (cv::pointPolygonTest(polygon, point, false)) {
                collision = true;
                break; // no need to check the remaining cells
            }
        }
    }

    // visualization
    Visualizer* vis = Visualizer::getInstance();
    if (vis->hasSubscriber()) {
        int marker_id = 0;
        vector<cv::Point>::iterator it = polygon.begin();
        cv::Point p1 = *it;

        for (++it; it != polygon.end(); ++it) {
            cv::Point p2 = *it;

            // convert cv::Point to ros points...
            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x;  gp1.y = p1.y;
            gp2.x = p2.x;  gp2.y = p2.y;

            // colour is green when the bos is empty and red, if there is an obstacle
            float r = collision ? 1 : 0;
            float g = 1 - r;
            vis->drawLine(marker_id++, gp1, gp2, "laser", "collision_box", r,g,0, 3, 0.05);

            p1 = p2;
        }
    }

//    cv::line(debug, cv::Point((int)p(0),(int)p(1)), cv::Point((int)q(0), (int)q(1)), cv::Scalar(0));
//    cv::line(debug, cv::Point((int)p(0),(int)p(1)), cv::Point((int)r(0), (int)r(1)), cv::Scalar(0));

//    cv::imshow("ObstacleBox", debug);

    return collision;
}
