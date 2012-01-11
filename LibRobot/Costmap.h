/*
 *  Costmap.h
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <sys/types.h>

class Costmap
{
public:
    Costmap(int threshold, int dilate, int erode);

    /**
     * Inflate the given map.
     * Freespace is eroded 'erode' times,
     * Obstacles are dilated 'dilate' times
     *
     * @param map The map to be inflated
     * @param width Width of the map
     * @param height Height of the map
     */
    std::vector<int8_t> grow(std::vector<int8_t> map, int width, int height);

private:
    static const int UNKNOWN =  -1;
    static const int FREE    =   0;
    static const int BLOCKED = 100;

    int threshold_;
    int dilate_;
    int erode_;
};

#endif // COSTMAP_H
