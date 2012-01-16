/*
 *  Costmap.h
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef COSTMAP_H
#define COSTMAP_H

#include <stddef.h>
#include <opencv/cv.h>

#include <vector>
#include <sys/types.h>

class Costmap
{
public:
    Costmap();
    ~Costmap ();

    /**
     * Inflate the given map.
     * Unknown space is eroded 'erode' times,
     * Obstacles are dilated 'dilate' times
     *
     * @param map The map to be inflated
     * @param width Width of the map
     * @param height Height of the map
     * @param dilate Number of iterations for dilate (default: 4)
     * @param erode Number of iterations for erode (default: 4)
     * @param threshold Threshold for blocked cells (v > threshold is blocked) (default: 10)
     * @param sample_factor Map dimensions are divided by this value (default: 1 = none)
     */
     void grow(const std::vector<int8_t>& map, unsigned width, unsigned height,
               std::vector<int8_t>& result,
               int dilate, int erode, int threshold, unsigned sample_factor);


     /**
      * Inflate the given map.
      * Unknown space is eroded 'erode' times,
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

    unsigned map_width_,map_height_;
    CvMat *blocked_map_, *free_map_,*dest_blocked_map_, *dest_free_map_,*res_mat_;
    IplConvKernel *block_shape_,*clear_shape_;

private:
    void refresh();
    void split(const std::vector<int8_t>& map, u_char* blocked, u_char* free, int threshold);

};

#endif // COSTMAP_H
