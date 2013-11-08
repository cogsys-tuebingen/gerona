#ifndef MARKERPUBLISHER_H
#define MARKERPUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

/**
 * @brief Provides methods to publish rviz markers.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class MarkerPublisher
{
public:
//    // some default colors
//    enum Colors {
//        RED, GREEN, BLUE, CYAN, MARGENTA, YELLOW, VIOLET, ORANGE
//    };

    MarkerPublisher();

    void setTime(const ros::Time &time);

    /**
     * @brief Publish a rviz marker.
     * @param marker The marker.
     */
    void publish(const visualization_msgs::Marker &marker) const;

    /**
     * @brief Sends a marker to rviz which visualizes the goal as an arrow.
     * The arrow starts at the goal position and points in the direction of the goal orientation.
     * @param goal The goal pose with position and orientation of the goal.
     */
    void publishGoalMarker(Eigen::Vector2f position, float theta) const;

    /**
     * @brief Remove a prior published goal marker.
     */
    void removeGoalMarker() const;

    /**
     * @brief Sends a line marker to rviz.
     *
     * Creates a line marker from point p1 to point p2 (x,y-coords, z = 0) and publishes it.
     *
     * @param p1 Start point of the line.
     * @param p2 End point of the line.
     * @param id Id of the line. A published line will replace older lines with the same id.
     * @param color Color of the line.
     */
    void publishLineMarker(Eigen::Vector2f p1, Eigen::Vector2f p2, int id, std_msgs::ColorRGBA color) const;

    void publishArrowMarker(Eigen::Vector2f point, float angle, int id, std_msgs::ColorRGBA color) const;
    void publishArrowMarker(Eigen::Vector2f point, Eigen::Vector2f direction, int id, std_msgs::ColorRGBA color) const;

private:
    ros::NodeHandle node_handle_;
    ros::Publisher publisher_;
    ros::Time time_;

    /**
     * @brief Convert an Eigen::Vector2f to a geometry_msgs::Point.
     *
     * The z-value of the point is set to 0.
     *
     * @param v Some vector.
     * @return a Point with the values of the given vector.
     */
    static geometry_msgs::Point vectorToPoint(Eigen::Vector2f v);
};

#endif // MARKERPUBLISHER_H
