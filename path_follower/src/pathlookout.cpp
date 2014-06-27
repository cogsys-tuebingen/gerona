#include "pathlookout.h"

#include <opencv2/highgui/highgui.hpp>

PathLookout::PathLookout()
{
    cv::namedWindow("Map", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Path", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Intersection", CV_WINDOW_KEEPRATIO);
}

void PathLookout::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;

    // convert to image
    map_image_ = cv::Mat(map->info.height, map->info.width, CV_8UC1);
    // Directly copy. Now 0 -> free, >0 -> obstacle. For visualization, it would usually be inverted, here, however,
    // this is perfectly fine, as the obstacles have to be "true" for the intersection with the path image.
    // Note, that it is important, that the path image is then really binary with only 0 and 255, to make sure, that the
    // bitwise_and can not miss any obstacle pixels.
    std::copy(map->data.begin(), map->data.end(), map_image_.data);
    //map_image_.data = &map->data[0];  <- no copy, only references. Would be nice, but yields problems with const...

    // make sure that the path image is of the same size than the map
    path_image_ = cv::Mat(map->info.height, map->info.width, CV_8UC1, cv::Scalar(0));
}

void PathLookout::setPath(const Path &path)
{
    //path_ = path;
    drawPathToImage(path);
}

bool PathLookout::lookForObstacles()
{
    if (map_ == NULL) {
        ROS_WARN("PathLookout has not received any map yet. No obstacle lookout is done.");
        return false;
    }

    // Calculate intersection of the obstacle map and the path ==> provides the obstacles on the path
    cv::Mat intersect;
    cv::bitwise_and(map_image_, path_image_, intersect);

    // debug
    cv::imshow("Map", map_image_);
    cv::imshow("Path", path_image_);
    cv::imshow("Intersection", intersect);
    cv::waitKey(5);

    return false;
}


void PathLookout::drawPathToImage(const Path &path)
{
    // first reset image
    path_image_ = cv::Scalar(0);

    if (path.size() < 2) {
        ROS_WARN("Path has less than 2 waypoints. No obstacle lookout is done.");
        return;
    }

    Path::const_iterator iter = path.begin();

    // get first point
    cv::Point2f p1(iter->x, iter->y);
    p1 = transformPointToMap(p1, "/map");
    // iterate over second to last point
    for (++iter; iter != path.end(); ++iter) {
        cv::Point2f p2(iter->x, iter->y);
        p2 = transformPointToMap(p2, "/map");

        cv::line(path_image_, p1, p2, cv::Scalar(255), 2); //TODO: more reasonable value for thickness
        p1 = p2;
    }
}

cv::Point2f PathLookout::transformPointToMap(const cv::Point2f &p, std::string from) const
{
    //TODO: this is copy waste... (already in ObstacleDetectorPolygon). Find some way to make it globally accessable.

    // lookup transform from point frame to map frame.
    tf::StampedTransform trans_to_map_frame;
    tf_listener_.lookupTransform(map_->header.frame_id, from, ros::Time(0), trans_to_map_frame);

    // construct transform from map frame origin to map origin
    tf::Transform trans_to_map_origin;
    tf::poseMsgToTF(map_->info.origin, trans_to_map_origin);
    // need the other way round
    trans_to_map_origin = trans_to_map_origin.inverse();

    // we need the point as tf::Vector3 ...
    tf::Vector3 tf_p(p.x, p.y, 0);

    // finally transform the point
    tf_p = trans_to_map_origin * trans_to_map_frame * tf_p;

    // to get map coordinates, the point has to be scaled to map resolution
    tf_p /= map_->info.resolution;

    // and finished. convert back to cv::Point and return :)
    return cv::Point2f(tf_p.x(), tf_p.y());
}
