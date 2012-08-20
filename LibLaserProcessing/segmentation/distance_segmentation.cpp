/**
 * @file distance_segmentation.cpp
 * @date Aug 2012
 * @author marks, buck
 */

// Project
#include "distance_segmentation.h"

namespace lib_laser_processing {

DistanceSegmentation::DistanceSegmentation( float threshold )
    : threshold_( threshold )
{}

void DistanceSegmentation::findSegments( const std::vector<LaserBeam> &beams, std::vector<DistanceSegment> &segments)
{
    segments.clear();

    unsigned int seg_length = 1;
    bool in_segment = true;
    float d;
    DistanceSegment seg;
    seg.start = 0;

    unsigned int size = beams.size();
    for ( unsigned int i = 0; i < size - 1; ++i ) {
        d = beams[i+1].range - beams[i].range;

        if ( std::abs(d) < threshold_ ) {
            ++seg_length;
            in_segment = true;
            continue;
        }

        // segment is over
        seg.length = seg_length;
        lengths.push_back( seg );
        seg_length = 1;
        seg.start = i;
        in_segment = false;
    }

    // last segment is not closed?
    if ( in_segment ) {
        lengths.push_back( seg );
    }
}

} // namespace
