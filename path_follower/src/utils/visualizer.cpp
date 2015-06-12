#include <path_follower/utils/visualizer.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

using namespace Eigen;

Visualizer::Visualizer() :
    private_nh_("~")
{
    vis_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/marker", 100);
    marray_vis_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("/path_coord_array", 100);
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

bool Visualizer::MarrayhasSubscriber()
{
    return marray_vis_pub_.getNumSubscribers() > 0;
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

void Visualizer::drawLine(int id, const Eigen::Vector2d &from, const Eigen::Vector2d &to, const std::string &frame, const std::string &ns, float r, float g, float b, double live, float scale) const
{
    geometry_msgs::Point p1, p2;
    p1.x = from[0];
    p1.y = from[1];
    p2.x = to[0];
    p2.y = to[1];

    drawLine(id, p1, p2, frame, ns, r, g, b, live, scale);
}

void Visualizer::drawCircle(int id, const geometry_msgs::Point &center, double radius, const std::string &frame,
                            const std::string &ns, float r, float g, float b, float alpha, double live) const
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
    marker.color.a = alpha;
    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.type = visualization_msgs::Marker::CYLINDER;

    vis_pub_.publish(marker);
}

void Visualizer::drawMark(int id, const geometry_msgs::Point &pos, const std::string &ns, float r, float g, float b,
                          const std::string &frame) const
{
    visualization_msgs::Marker marker;
    marker.pose.position = pos;
    marker.ns = ns;
    marker.header.frame_id = frame;
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

void Visualizer::drawText(int id, const geometry_msgs::Point &pos, const std::string &text, const std::string &ns,
                          float r, float g, float b, const std::string &frame, double live) const
{
    visualization_msgs::Marker marker;
    marker.pose.position = pos;
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
    marker.scale.z = 0.2;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = text;

    vis_pub_.publish(marker);
}


void Visualizer::drawSteeringArrow(int id, geometry_msgs::Pose robot_pose, double angle, double r, double g, double b)
{
    robot_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(robot_pose.orientation) + angle);
    drawArrow(id, robot_pose, "steer", r, g, b);
}

void Visualizer::visualizeLine(const Line2d &line)
{
    Vector2d from = line.GetOrigin() - 5 * line.GetDirection();
    Vector2d to   = line.GetOrigin() + 5 * line.GetDirection();

    geometry_msgs::Point f,t;
    f.x = from(0);
    f.y = from(1);
    t.x = to(0);
    t.y = to(1);

    drawLine(2, f, t, "/base_link", "line", 0.7, 0.2, 1.0, 1, 0.1);
}

ros::Publisher Visualizer::getMarkerPublisher()
{
    return vis_pub_;
}

ros::Publisher Visualizer::getMarkerArrayPublisher()
{
    return marray_vis_pub_;
}
