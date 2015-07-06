#ifndef VISUALIZER_H
#define VISUALIZER_H

/// STL
#include <string>

/// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

/// OTHER
#include <Eigen/Core>

/// PROJECT
#include <utils_general/Line2d.h>

/**
 * @brief Provides functions to visualize things in Rviz (e.g. arrows, lines, ...)
 *
 * This is a singleton class. This means, there is only one instance of the class, which is accessable from everywhere.
 * To get a pointer to this instance use the following code:
 *
 * <code>
 *    Visualizer *vis = Visualizer::getInstance();
 * </code>
 */
class Visualizer
{
public:
    //! Get a pointer to the Visualizer object.
    static Visualizer* getInstance();

    /**
     * @brief Check if there are any subscribers to the markers.
     *
     * There is no need to publish markers, as long as nobody subscribes for them. This method can be used to check this,
     * before doing computations for the markers.
     *
     * @return true if there is at least one subscriber.
     */
    bool hasSubscriber();
    bool MarrayhasSubscriber();

    /**
     * @brief Publish an arrow marker.
     * @param id    ID of the marker.
     * @param pose  Pose of the arrow (position and pointing direction)
     * @param ns    Namespace of the marker.
     * @param r     Marker color, red.
     * @param g     Marker color, green.
     * @param b     Marker color, blue.
     * @param live  Lifetime of the marker.
     */
    void drawArrow(int id, const geometry_msgs::Pose &pose, const std::string& ns, float r, float g, float b, double live = 3) const;

    /**
     * @brief Publish a line marker.
     * @param id    ID of the marker.
     * @param from  Start point of the line.
     * @param to    End point of the line.
     * @param frame TF-frame in which the line is placed.
     * @param ns    Namespace of the marker.
     * @param r     Marker color, red.
     * @param g     Marker color, green.
     * @param b     Marker color, blue.
     * @param live  Lifetime of the marker.
     * @param scale Thickness of the line.
     */
    void drawLine(int id, const geometry_msgs::Point &from, const geometry_msgs::Point &to, const std::string &frame,
                  const std::string& ns, float r, float g, float b, double live = 3, float scale = 0.3) const;

    // same but with Eigen::Vector instead of geometry_msgs::Point.
    void drawLine(int id, const Eigen::Vector2d &from, const Eigen::Vector2d &to, const std::string &frame,
                  const std::string& ns, float r, float g, float b, double live = 3, float scale = 0.3) const;

    /**
     * @brief Publish a circle marker.
     * @param id      ID of the marker.
     * @param center  Center of the circle.
     * @param radius  Radius of the circle.
     * @param frame   TF-frame in which the marker is placed.
     * @param ns      Namespace of the marker.
     * @param r       Marker color, red.
     * @param g       Marker color, green.
     * @param b       Marker color, blue.
     * @param alpha   Marker color, alpha (for transparency)
     * @param live    Lifetime of the marker.
     */
    void drawCircle(int id, const geometry_msgs::Point &center, double radius, const std::string &frame,
                    const std::string& ns, float r, float g, float b, float alpha=1, double live = 3) const;

    /**
     * @brief Publish a simple marker to visualize a single point.
     * @param id    ID of the marker.
     * @param pos   Position of the marker.
     * @param ns    Namespace of the marker.
     * @param r     Marker color, red.
     * @param g     Marker color, green.
     * @param b     Marker color, blue.
     * @param frame TF-Frame, in which the position is given.
     */
    void drawMark(int id, const geometry_msgs::Point &pos, const std::string& ns, float r, float g, float b,
                  const std::string &frame="/map") const;

    /**
     * @brief Publish a marker to display some text.
     * @param id    ID of the marker.
     * @param pos   Position of the marker.
     * @param text  Text that is displayed.
     * @param ns    Namespace of the marker.
     * @param r     Marker color, red.
     * @param g     Marker color, green.
     * @param b     Marker color, blue.
     * @param frame TF-Frame, in which the position is given.
     * @param live  Lifetime of the marker.
     */
    void drawText(int id, const geometry_msgs::Point &pos, const std::string &text, const std::string& ns,
                  float r, float g, float b, const std::string &frame="/map", double live = 1) const;

    /**
     * @brief Draw an arrow to visualize the steering angle.
     * @param id     ID of the marker.
     * @param robot_pose  Pose of the robot.
     * @param angle  Steering angle.
     * @param r      Marker color, red.
     * @param g      Marker color, green.
     * @param b      Marker color, blue.
     */
    void drawSteeringArrow(int id, geometry_msgs::Pose robot_pose, double angle, double r, double g, double b);

    /**
     * @brief Wrapper for drawLine, that only takes a line object.
     * @param line
     */
    void visualizeLine(const Line2d &line);

    /**
     * @brief Draw a moving Frenet-Serret frame with the distance vector.
     * @param id        ID of the marker.
     * @param robot_pose  Pose of the robot.
     * @param xe        Error in F-S frame, x component.
     * @param ye        Error in F-S frame, y component.
     * @param p_ind     Current point on the path, x component.
     * @param q_ind     Current point on the path, y component.
     * @param theta_p   Slope of the path in the current point.
     */
    void drawFrenetSerretFrame(int id, Eigen::Vector3d robot_pose, double xe, double ye, double p_ind,
                                           double q_ind, double theta_p);

    ros::Publisher getMarkerPublisher();
    ros::Publisher getMarkerArrayPublisher();

private:
    ros::NodeHandle private_nh_;
    ros::Publisher vis_pub_;
    ros::Publisher marray_vis_pub_;

    Visualizer();
};

#endif // VISUALIZER_H
