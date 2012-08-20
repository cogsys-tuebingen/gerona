/**
 * @file leg_detector.cpp
 * @date Aug 2012
 * @author marks, buck
 */

// Project
#include "leg_detector.h"

namespace lib_laser_processing {

LegDetector::LegDetector()
{
}

void LegDetector::update( DistanceSegmentation &segms )
{
    // For each segment
    DistanceSegmentation::Iterator it = segms.iterator();
    while ( it.nextSegment()) {

    }
}

} // namespace
