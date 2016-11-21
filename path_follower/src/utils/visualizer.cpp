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

void Visualizer::drawArrow(const std::string& frame, int id, const geometry_msgs::Pose &pose, const std::string &ns, float r, float g, float b, double live, double scale) const
{
    visualization_msgs::Marker marker;
    marker.pose = pose;
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
    marker.scale.x = 0.75 * scale;
    marker.scale.y = 0.05 * scale;
    marker.scale.z = 0.05 * scale;
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


void Visualizer::drawSteeringArrow(const std::string& frame, int id, geometry_msgs::Pose robot_pose, double angle, double r, double g, double b)
{
    robot_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(robot_pose.orientation) + angle);
    drawArrow(frame, id, robot_pose, "steer", r, g, b);
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

    drawLine(2, f, t, "base_link", "line", 0.7, 0.2, 1.0, 1, 0.1);
}

void Visualizer::drawFrenetSerretFrame(const std::string& frame, int id, Eigen::Vector3d robot_pose, double xe, double ye, double p_ind,
                                       double q_ind, double theta_p)
{
    visualization_msgs::MarkerArray path_coord_marray;

    visualization_msgs::Marker prototype;
    prototype.header.frame_id = frame;
    prototype.header.stamp = ros::Time();
    prototype.action = visualization_msgs::Marker::ADD;
    prototype.id = id;
    prototype.color.a = 1.0;
    prototype.type = visualization_msgs::Marker::ARROW;


    //robot position vector module
    double r = hypot(robot_pose[0] - p_ind, robot_pose[1] - q_ind);

    //robot position vector angle in world coordinates
    double theta_r = atan2(robot_pose[1] - q_ind, robot_pose[0] - p_ind);

    visualization_msgs::Marker path_robot_marker = prototype;
    path_robot_marker.ns = "path_robot_vector";
    path_robot_marker.color.r = 1;
    path_robot_marker.color.g = 1;
    path_robot_marker.color.b = 0;
    path_robot_marker.scale.x = r;//0.05;
    path_robot_marker.scale.y = 0.02;
    path_robot_marker.scale.z = 0.02;

//    geometry_msgs::Point from, to;
//    from.x = robot_pose[0];
//    from.y = robot_pose[1];
//    to.x = p_ind;
//    to.y = q_ind;

//    path_robot_marker.points.push_back(to);
//    path_robot_marker.points.push_back(from);
    path_robot_marker.pose.position.x = p_ind;
    path_robot_marker.pose.position.y = q_ind;
    path_robot_marker.pose.position.z = 0;
    path_robot_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta_r);


    visualization_msgs::Marker path_abscissa_marker = prototype;
    path_abscissa_marker.ns = "path_coord_abscissa";
    path_abscissa_marker.color.r = 1;
    path_abscissa_marker.color.g = 0;
    path_abscissa_marker.color.b = 0;
    path_abscissa_marker.scale.x = 0.3;
    path_abscissa_marker.scale.y = 0.05;
    path_abscissa_marker.scale.z = 0.05;

    path_abscissa_marker.pose.position.x = p_ind;
    path_abscissa_marker.pose.position.y = q_ind;
    path_abscissa_marker.pose.position.z = 0.0;
    path_abscissa_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta_p);


    visualization_msgs::Marker abscissa_distance_marker = prototype;
    abscissa_distance_marker.ns = "abscissa_distance";
    abscissa_distance_marker.color.r = 0;
    abscissa_distance_marker.color.g = 1;
    abscissa_distance_marker.color.b = 1;
    abscissa_distance_marker.scale.x = xe;
    abscissa_distance_marker.scale.y = 0.02;
    abscissa_distance_marker.scale.z = 0.02;

    abscissa_distance_marker.pose.position.x = p_ind;
    abscissa_distance_marker.pose.position.y = q_ind;
    abscissa_distance_marker.pose.position.z = 0.0;
    abscissa_distance_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta_p);


    visualization_msgs::Marker path_ordinate_marker = prototype;
    path_ordinate_marker.ns = "path_coord_ordinate";
    path_ordinate_marker.color.r = 0;
    path_ordinate_marker.color.g = 1;
    path_ordinate_marker.color.b = 0;;
    path_ordinate_marker.scale.x = 0.3;
    path_ordinate_marker.scale.y = 0.05;
    path_ordinate_marker.scale.z = 0.05;

    path_ordinate_marker.pose.position.x = p_ind;
    path_ordinate_marker.pose.position.y = q_ind;
    path_ordinate_marker.pose.position.z = 0.0;
    path_ordinate_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta_p + M_PI/2.0);


    visualization_msgs::Marker ordinate_distance_marker = prototype;
    ordinate_distance_marker.ns = "ordinate_distance";
    ordinate_distance_marker.color.r = 0;
    ordinate_distance_marker.color.g = 1;
    ordinate_distance_marker.color.b = 1;
    ordinate_distance_marker.scale.x = ye;
    ordinate_distance_marker.scale.y = 0.02;
    ordinate_distance_marker.scale.z = 0.02;

    ordinate_distance_marker.pose.position.x = p_ind;
    ordinate_distance_marker.pose.position.y = q_ind;
    ordinate_distance_marker.pose.position.z = 0.0;
    ordinate_distance_marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta_p + M_PI/2.0);



    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_robot_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_abscissa_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(abscissa_distance_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_ordinate_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(ordinate_distance_marker));

    marray_vis_pub_.publish(path_coord_marray);
}

ros::Publisher Visualizer::getMarkerPublisher()
{
    return vis_pub_;
}

ros::Publisher Visualizer::getMarkerArrayPublisher()
{
    return marray_vis_pub_;
}
