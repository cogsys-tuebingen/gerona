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

void DistanceSegmentation::update( const std::vector<LaserBeam> &beams )
{
    beams_ = beams;
    segms_.clear();
    DistanceSegment s;
    s.start = 0;
    s.length = 1;

    // For all beams
    bool in_segment = true;
    int end = beams.size() - 1;
    for ( int i = 0; i < end - 1; ++i ) {
        // End of segment?
        if ( !beams_[i+1].valid
             || std::abs( beams_[i].range - beams_[i+1].range ) > threshold_ ) {
            segms_.push_back( s );
            s.start = i + 1;
            s.length = 1;
            in_segment = false;
            continue;
        }

        // Still in segment
        ++s.length;
        in_segment = true;
    }

    // Close last segment?
    if ( in_segment )
        segms_.push_back( s );
}

} // namespace
