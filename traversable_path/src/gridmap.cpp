#include "gridmap.h"
#include <cmath>
#include <float.h>

using namespace std;
using namespace Eigen;

GridMap::GridMap(const nav_msgs::OccupancyGrid &map)
{
    cols_ = map.info.width;
    rows_ = map.info.height;
    map_ = map;

    odds_ = vector<vector<double> > (cols_, vector<double>(rows_, ODDS_INIT));
}

void GridMap::update(Eigen::Vector2f robot_position, PointCloudXYZRGBT cloud)
{
    moveMap(robot_position);

    for (PointCloudXYZRGBT::iterator point_it = cloud.begin(); point_it != cloud.end(); ++point_it) {
        int x, y;
        x = (point_it->x - map_.info.origin.position.x) / map_.info.resolution;
        y = (point_it->y - map_.info.origin.position.y) / map_.info.resolution;

        // (int)-casts to supress comparison warning (no danger of overflow here)
        if (x < (int)map_.info.width && y < (int)map_.info.height && x >= 0 && y >= 0) {
            updateOddsOfCell(Vector2i(x,y), point_it->traversable ? GridMap::PROP_FREE : GridMap::PROP_OCCUPIED);
        } else {
            //ROS_WARN("Out of Map. (row,col) = (%d, %d)", col, row);
        }
    }
}

nav_msgs::OccupancyGrid GridMap::getMap()
{
    for (uint y = 0; y < map_.info.height; y++)
    {
        for(uint x = 0; x < map_.info.width; x++)
        {
            size_t index = y * map_.info.width + x;
            map_.data[index] = p(Vector2i(x, y)) < 0.5 ? 0 : 100;
        }
    }
    return map_;
}

void GridMap::moveMap(Eigen::Vector2f map_origin)
{
    //! The minimum distance, the robot must have moved, to recenter the map to the robot.
    const float MIN_ROBOT_MOVEMENT_DISTANCE = 2.0;

    // set origin so that the robot is in the center of the map
    map_origin[0] -= (signed int) map_.info.width  * map_.info.resolution / 2;
    map_origin[1] -= (signed int) map_.info.height * map_.info.resolution / 2;

    // only update if the robot has moved more than MIN_ROBOT_MOVEMENT_DISTANCE since last update (to improve
    // performance)
    float distance = sqrt(  pow(map_origin[0] - map_.info.origin.position.x, 2)
                          + pow(map_origin[1] - map_.info.origin.position.y, 2) );
    if (distance > MIN_ROBOT_MOVEMENT_DISTANCE) {
        std::vector<std::vector<double> > newodds
                = vector<vector<double> > (map_.info.width, vector<double>(map_.info.height, ODDS_INIT));

        // get transformation from old map to new.
        int transform_x = (map_origin[0] - map_.info.origin.position.x) / map_.info.resolution;
        int transform_y = (map_origin[1] - map_.info.origin.position.y) / map_.info.resolution;

        for (size_t x = 0; x < cols_; ++x) {
            for (size_t y = 0; y < rows_; ++y) {
                if (odds_[x][y] != ODDS_INIT) {
                    int new_x, new_y;
                    new_x = x - transform_x;
                    new_y = y - transform_y;

                    // (int)-casts to supress comparison warning (no danger of overflow here)
                    if (new_x >= 0 && new_x < (int)cols_ && new_y >= 0 && new_y < (int)rows_) {
                        newodds[new_x][new_y] = odds_[x][y];
                    }
                }
            }
        }

        map_.info.origin.position.x = map_origin[0];
        map_.info.origin.position.y = map_origin[1];
        odds_ = newodds;
    }
}

double GridMap::odds(double pofx) const
{
    ROS_ASSERT(pofx < 1);
    return pofx/(1-pofx);
}

void GridMap::updateOddsOfCell(Eigen::Vector2i cell, double p)
{
    odds_[cell[0]][cell[1]] = odds(p) * odds_[cell[0]][cell[1]];
    if (isinf(odds_[cell[0]][cell[1]])) {
        odds_[cell[0]][cell[1]] = DBL_MAX;
    }
}

double GridMap::p(Eigen::Vector2i cell) const
{
    double odd = odds_[cell[0]][cell[1]];

    ROS_ASSERT(odd != -1.0f);

    return odd/(1+odd);
}
