/*
 *  Costmap.h
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef COSTMAP_H
#define COSTMAP_H
#include <opencv/cv.h>

#include <vector>
#include <sys/types.h>

class Costmap
{
public:
    Costmap(int threshold, int dilate, int erode);
    ~Costmap ();
    /**
     * Inflate the given map.
     * Freespace is eroded 'erode' times,
     * Obstacles are dilated 'dilate' times
     *
     * @param map The map to be inflated
     * @param width Width of the map
     * @param height Height of the map
     */
     void grow(const std::vector<int8_t>& map, unsigned width, unsigned height,
               std::vector<int8_t>& result);

private:
    static const int UNKNOWN =  -1;
    static const int FREE    =   0;
    static const int BLOCKED = 100;

    int threshold_;
    int dilate_;
    int erode_;
    unsigned map_width_,map_height_;
    CvMat *blocked_map_, *free_map_,*dest_blocked_map_, *dest_free_map_,*res_mat_;
    IplConvKernel *block_shape_,*clear_shape_;
};

#endif // COSTMAP_H
