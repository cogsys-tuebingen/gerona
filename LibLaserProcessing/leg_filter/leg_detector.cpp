/**
 * @file leg_detector.cpp
 * @date Aug 2012
 * @author marks, buck
 */

// C/C++
#include <limits>

// Eigen
#include <Eigen/Core>

// Project
#include "leg_detector.h"

using namespace Eigen;

namespace lib_laser_processing {

LegDetector::LegDetector()
    : min_beams_( 5 )
{
    if ( min_beams_ < 5 )
        std::cout << "ERROR: Min beams < 5" << std::endl;
}

void LegDetector::update( DistanceSegmentation &segms )
{
    // For each segment
    DistanceSegmentation::Iterator it = segms.iterator();
    while ( it.nextSegment()) {
        // Enough beams?
        if ( it.length() < min_beams_ ) {
            it.invalidateSegment();
            continue;
        }

        // Compute center of mass (COM)
        Vector2d com( Vector2d::Zero());
        while ( it.nextBeam())
            com += it.beam().pos;
        com /= (double)it.length();

        // Compute minimum, maximum and average distance to COM
        double min = std::numeric_limits<double>::max();
        double max = 0;
        double avr = 0;
        double d;
        it.resetBeams();
        while ( it.nextBeam()) {
            d = (com - it.beam().pos).norm();
            avr += d;
            if ( min > d ) min = d;
            if ( max < d ) max = d;
        }
        avr /= (double)it.length();

        // Check
        if ( max > 0.17 ) {
            it.invalidateSegment();
            continue;
        }
        /*if ( max > 0.20 || com.x() < -1.0 || com.x() > 8.0
             || com.y() < -0.75 || com.y() > 1.0 ) {
            it.invalidateSegment();
            continue;
        }*/

        std::cout << min << " " << max << " " << avr << std::endl;

        Vector2d center( com + 0.04*com.normalized());
        double min_c = std::numeric_limits<double>::max();
        double max_c = 0;
        double avr_c = 0;
        it.resetBeams();
        while ( it.nextBeam()) {
            d = (center - it.beam().pos).norm();
            avr_c += d;
            if ( min_c > d ) min_c = d;
            if ( max_c < d ) max_c = d;
        }
        avr_c /= (double)it.length();

        /*std::cout << min << " " << max << " " << avr << " "
                  << min_c << " " << max_c << " " << avr_c << std::endl;*/

        if ( avr_c > 0.09 || avr_c < 0.05
             || max_c > 0.16 || max_c < 0.06
             || min_c > 0.08 || min_c < 0.03 ) {
            it.invalidateSegment();
            continue;
        }



    }
}

} // namespace
