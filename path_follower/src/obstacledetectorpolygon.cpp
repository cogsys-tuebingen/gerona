#include "obstacledetectorpolygon.h"

#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp> // only for debugging
#include "visualizer.h"

using namespace std;

bool ObstacleDetectorPolygon::checkForObstacle(float width, float length, float course_angle, float curve_enlarge_factor) const
{
    bool collision = false;

//    cv::Mat debug(map_->info.height, map_->info.width, CV_8UC1, cv::Scalar::all(255));
//    cv::namedWindow("ObstacleBox", CV_WINDOW_KEEPRATIO);

    PolygonWithTfFrame pwf = getPolygon(width, length, course_angle, curve_enlarge_factor);
    PolygonWithTfFrame poly_for_viz = pwf;

    try {
        transformPolygonToMap(&pwf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Error with transform obstacle polygon: %s", ex.what());
        // can't check for obstacles, so better assume there is one.
        return true;
    }

    vector<cv::Point2f> polygon = pwf.polygon;

    if (polygon.size() == 0) {
        ROS_WARN("Obstacle polygon is empty -> no obstacle test is done!");
        return false;
    }


    const unsigned data_size = map_->info.height * map_->info.width;
    for (unsigned i = 0; i < data_size; ++i) {

//        debug.data[i] = map_->data[i] == OCCUPIED ? 127 : 255;

        if (map_->data[i] == OCCUPIED) {
            // check if this map point is inside the polygon
            cv::Point2f point( i % map_->info.width, i / map_->info.width );

            if (cv::pointPolygonTest(polygon, point, false) == 1) {
                collision = true;
        //        break; // no need to check the remaining cells
            }
        }
    }



//    for (size_t i = 1; i < polygon.size(); ++i) {
//        cv::line(debug, polygon[i-1], polygon[i], cv::Scalar(0));
//    }
//    cv::line(debug, polygon.front(), polygon.back(), cv::Scalar(0));
//    cv::imshow("ObstacleBox", debug);
//    cv::waitKey(10);


    // visualization
    Visualizer* vis = Visualizer::getInstance();
    if (vis->hasSubscriber()) {
        // push first element to back, to close the polygon.
        poly_for_viz.polygon.push_back(poly_for_viz.polygon.front());

        vector<cv::Point2f>::iterator it = poly_for_viz.polygon.begin();
        cv::Point2f p1 = *it;

        int marker_id = 0;
        for (++it; it != poly_for_viz.polygon.end(); ++it) {
            cv::Point2f p2 = *it;

            // convert cv::Point2f to ros points...
            geometry_msgs::Point gp1, gp2;
            gp1.x = p1.x;  gp1.y = p1.y;
            gp2.x = p2.x;  gp2.y = p2.y;

            // colour is green when the box is empty and red if there is an obstacle
            float r = collision ? 1 : 0;
            float g = 1 - r;
            vis->drawLine(marker_id++, gp1, gp2, poly_for_viz.frame, "collision_box", r,g,0, 3, 0.05);

            p1 = p2;
        }
    }

    return collision;
}

cv::Point2f ObstacleDetectorPolygon::transformCvPoint(const cv::Point2f &p, string from, string to) const
{
    geometry_msgs::PointStamped pf, pt;
    pf.header.frame_id = from;
    pf.point.x = p.x;
    pf.point.y = p.y;
    pf.point.z = 0;

    // transform to map frame
    tf_listener_.transformPoint(to, pf, pt);


    // transform to map coordinates
    geometry_msgs::Transform trans;
    trans.translation.x = map_->info.origin.position.x;
    trans.translation.y = map_->info.origin.position.y;
    trans.translation.z = map_->info.origin.position.z;
    trans.rotation = map_->info.origin.orientation;

    tf::Transform tf_trans;
    tf::transformMsgToTF(trans, tf_trans);

    // we need the point as tf::Vector3 ...
    tf::Vector3 tf_p(pt.point.x, pt.point.y, 0);
    tf_p = tf_trans.inverse() * tf_p;

    // finaly scale to map resolution
    tf_p /= map_->info.resolution;

    return cv::Point2f(tf_p.x(), tf_p.y());
}

void ObstacleDetectorPolygon::transformPolygonToMap(PolygonWithTfFrame *polygon) const
{
    // transform each point
    vector<cv::Point2f>::iterator iter;
    for (iter = polygon->polygon.begin(); iter != polygon->polygon.end(); ++iter) {
        *iter = transformCvPoint(*iter, polygon->frame, world_frame_);
    }
    // set new frame
    polygon->frame = world_frame_;
}
