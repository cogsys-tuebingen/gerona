/*
 * statistics.h
 *
 *  Created on: 31.05.2012
 *      Author: buck
 */

#ifndef PATHLOGGER_H
#define PATHLOGGER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

class PathLogger
{
public:
    PathLogger();
    virtual ~PathLogger();

    void map_callback(const nav_msgs::OccupancyGridConstPtr &msg);

    void init();
    void tick();

private:
    tf::TransformListener listener;

    ros::Subscriber sub_;
    ros::Publisher pub_;

    nav_msgs::OccupancyGridPtr latest_map_;

    tf::Vector3 last_pos_;
    bool has_last_pos_;
    std::vector<geometry_msgs::Point> positions_;
};

#endif // PATHLOGGER_H
