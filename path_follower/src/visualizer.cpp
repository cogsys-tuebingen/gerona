#include "visualizer.h"
#include <visualization_msgs/Marker.h>

Visualizer::Visualizer() :
    private_nh_("~")
{
    vis_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/marker", 100);
}

Visualizer *Visualizer::getInstance()
{
    static Visualizer instance;

    return &instance;
}

bool Visualizer::hasSubscriber()
{
    return vis_pub_.getNumSubscribers() > 0;
}

void Visualizer::drawArrow(int id, const geometry_msgs::Pose &pose, const std::string &ns, float r, float g, float b, double live) const
{
    visualization_msgs::Marker marker;
    marker.pose = pose;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.75;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.type = visualization_msgs::Marker::ARROW;

    vis_pub_.publish(marker);
}

void Visualizer::drawLine(int id, const geometry_msgs::Point &from, const geometry_msgs::Point &to, const std::string &frame,
                          const std::string &ns, float r, float g, float b, double live, float scale) const
{
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.points.push_back(from);
    marker.points.push_back(to);
    marker.scale.x = scale;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    vis_pub_.publish(marker);
}

void Visualizer::drawCircle(int id, const geometry_msgs::Point &center, double radius, const std::string &frame,
                            const std::string &ns, float r, float g, float b, double live) const
{
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.type = visualization_msgs::Marker::CYLINDER;

    vis_pub_.publish(marker);
}

void Visualizer::drawMark(int id, const geometry_msgs::Point &pos, const std::string &ns, float r, float g, float b) const
{
    visualization_msgs::Marker marker;
    marker.pose.position = pos;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(3);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.5;
    marker.type = visualization_msgs::Marker::CUBE;

    vis_pub_.publish(marker);
}
