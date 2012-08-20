/**
 * @file distance_segmentation.h
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef DISTANCE_SEGMENTATION_H
#define DISTANCE_SEGMENTATION_H

// C/C++
#include <vector>

// Project
#include "../LaserBeam.h"

namespace lib_laser_processing {

struct DistanceSegment
{
    unsigned int start;
    unsigned int length;
};

class DistanceSegmentation
{
public:
    DistanceSegmentation( loat threshold );

    void findSegments( const std::vector<LaserBeam>& beams, std::vector<DistanceSegment>& segments );

private:
    float threshold_;

};

} // namespace

#endif // DISTANCE_SEGMENTATION_H
