#include "maptransformer.h"

void MapTransformer::setMap(const nav_msgs::OccupancyGridConstPtr &map)
{
    map_ = map;

    // this is a static transform, thus it can be precomputed
    tf::poseMsgToTF(map_->info.origin, trans_from_map_cell_to_map_frame_);
}

cv::Point2f MapTransformer::transformPointToMap(const cv::Point2f &p, std::string from) const
{
    //FIXME: check if map_ is set!

    // we need the point as tf::Vector3 ...
    tf::Vector3 tf_p(p.x, p.y, 0);

    // lookup transform from point frame to map frame.
    tf::StampedTransform trans_to_map_frame;
    tf_listener_.lookupTransform(map_->header.frame_id, from, ros::Time(0), trans_to_map_frame);

    // transform from map frame to map cell
    tf::Transform trans_to_map_origin = trans_from_map_cell_to_map_frame_.inverse();

    // finally transform the point
    tf_p = trans_to_map_origin * trans_to_map_frame * tf_p;

    // to get map coordinates, the point has to be scaled to map resolution
    tf_p /= map_->info.resolution;

    // and finished. convert back to cv::Point and return :)
    return cv::Point2f(tf_p.x(), tf_p.y());
}

cv::Point2f MapTransformer::transformPointFromMap(const cv::Point2f &p, std::string to) const
{
    // we need the point as tf::Vector3 ...
    tf::Vector3 tf_p(p.x, p.y, 0);

    // scale from map coordinates to frame coordinates
    tf_p *= map_->info.resolution;

    // lookup transform from map frame to desired frame.
    tf::StampedTransform trans_from_map_frame;
    tf_listener_.lookupTransform(to, map_->header.frame_id, ros::Time(0), trans_from_map_frame);

    // finally transform the point
    tf_p = trans_from_map_frame * trans_from_map_cell_to_map_frame_ * tf_p;

    // and finished. convert back to cv::Point and return :)
    return cv::Point2f(tf_p.x(), tf_p.y());
}
