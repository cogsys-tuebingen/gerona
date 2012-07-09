#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include "point_types.h"

/**
 * @brief An occupancy grid implementation.
 *
 * Base of this class is the GridMap class implemented with Alina Kloss at the practical course
 * "Praktikum Mobilde Roboter". It was modified to fit with my programm.
 *
 * @author Alina Kloss, Felix Widmaier
 * @version $Id$
 */
class GridMap
{
public:
    typedef pcl::PointCloud<PointXYZRGBT> PointCloudXYZRGBT;

    static const double PROP_OCCUPIED = 0.9;
    static const double PROP_FREE = 0.1;

    GridMap(const nav_msgs::OccupancyGrid &map);

    void update(Eigen::Vector2f robot_position, PointCloudXYZRGBT classification);

    nav_msgs::OccupancyGrid getMap();


private:
    static const double ODDS_INIT = 1.0;

    uint rows_;
    uint cols_;
    nav_msgs::OccupancyGrid map_;
    std::vector<std::vector<double> > odds_;

    void moveMap(Eigen::Vector2f robot_position);

    double odds(double pofx) const; // (36.3)
    void updateOddsOfCell(Eigen::Vector2i cell, double p); // (36.4)
    double p(Eigen::Vector2i cell) const;    // (36.5)
};

#endif // GRIDMAP_H
