#include "markerpublisher.h"
#include <cmath>
#include <tf/tf.h>

using namespace std;
using namespace Eigen;

MarkerPublisher::MarkerPublisher():
    time_(0)
{
    publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 100);
}

void MarkerPublisher::setTime(const ros::Time &time)
{
    time_ = time;
}

void MarkerPublisher::publish(const visualization_msgs::Marker &marker) const
{
    publisher_.publish(marker);
}

void MarkerPublisher::publishGoalMarker(Vector2f position, float theta) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = time_;
    marker.pose.position = vectorToPoint(position);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "follow_path/goal";
    marker.id = 0;

    // Set the marker type.
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Publish the marker
    publish(marker);
}

void MarkerPublisher::removeGoalMarker() const
{
    visualization_msgs::Marker marker;
    marker.ns = "follow_path/goal";
    marker.id = 0;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::DELETE;

    // Publish the marker
    publish(marker);
}

void MarkerPublisher::publishLineMarker(Eigen::Vector2f p1, Eigen::Vector2f p2, int id, std_msgs::ColorRGBA color) const
{
    visualization_msgs::Marker line;

    line.header.frame_id = "/map";
    line.header.stamp = time_;
    line.ns = "follow_path/lines";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = id;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line.scale.x = 0.03;
    line.color = color;

    // set the points
    line.points.push_back(vectorToPoint(p1));
    line.points.push_back(vectorToPoint(p2));

    publish(line);
}

void MarkerPublisher::publishArrowMarker(Eigen::Vector2f point, float angle, int id, std_msgs::ColorRGBA color) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = time_;
    marker.pose.position = vectorToPoint(point);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    marker.ns = "follow_path/arrow";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color = color;

    publish(marker);
}

void MarkerPublisher::publishArrowMarker(Eigen::Vector2f point, Eigen::Vector2f direction, int id, std_msgs::ColorRGBA color) const
{
    float angle = atan2(direction[1], direction[0]);
    publishArrowMarker(point, angle, id, color);
}

geometry_msgs::Point MarkerPublisher::vectorToPoint(Vector2f v)
{
    geometry_msgs::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = 0;

    return p;
}
