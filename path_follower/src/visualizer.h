#ifndef VISUALIZER_H
#define VISUALIZER_H

/// STL
#include <string>

/// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief The Visualizer class
 */
class Visualizer
{
public:
    static Visualizer* getInstance();

    bool hasSubscriber();

    void drawArrow(int id, const geometry_msgs::Pose &pose, const std::string& ns, float r, float g, float b, double live = 3) const;

    void drawLine(int id, const geometry_msgs::Point &from, const geometry_msgs::Point &to, const std::string &frame,
                  const std::string& ns, float r, float g, float b, double live = 3, float scale = 0.3) const;

    void drawCircle(int id, const geometry_msgs::Point &center, double radius, const std::string &frame,
                    const std::string& ns, float r, float g, float b, double live = 3) const;

    void drawMark(int id, const geometry_msgs::Point &pos, const std::string& ns, float r, float g, float b) const;

private:
    ros::NodeHandle private_nh_;

    ros::Publisher vis_pub_;


    Visualizer();
};

#endif // VISUALIZER_H
