/**
 * @file leg_detector.cpp
 * @date Aug 2012
 * @author marks, buck
 */

// C/C++
#include <limits>

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

// Project
#include "leg_detector.h"

using namespace Eigen;
using namespace boost::accumulators;

namespace lib_laser_processing {

LegDetector::LegDetector()
    : min_beams_( 5 )
{
    if ( min_beams_ < 5 )
        std::cout << "ERROR: Min beams < 5" << std::endl;
}

void LegDetector::update( DistanceSegmentation &segms )
{
    legs_.clear();

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
        double count = 0;
        it.resetBeams();
        while ( it.nextBeam()) {
            if ( !it.beam().valid )
                continue;

            d = (com - it.beam().pos).norm();
            avr += d;
            if ( min > d ) min = d;
            if ( max < d ) max = d;
            count++;
        }
        avr /= count;

        // Check
        if ( max > 0.2 ) {
            //it.invalidateSegment();
            continue;
        }
        /*if ((max > 0.20 || com.x() < -1.0 || com.x() > 8.0
             || com.y() < -0.75 || com.y() > 1.0 )) {
            it.invalidateSegment();
            continue;
        }*/

        Vector2d center( com + 0.04*com.normalized());
        double min_c = std::numeric_limits<double>::max();
        double max_c = 0;
        double avr_c = 0;
        count = 0;
        it.resetBeams();
        while ( it.nextBeam()) {
            if ( !it.beam().valid )
                continue;

            d = (center - it.beam().pos).norm();
            avr_c += d;
            count++;
            if ( min_c > d ) min_c = d;
            if ( max_c < d ) max_c = d;
        }
        avr_c /= count;

        /*std::cout << min << " " << max << " " << avr << " "
                  << min_c << " " << max_c << " " << avr_c << std::endl;

        if ( avr_c > 0.09 || avr_c < 0.05
             || max_c > 0.16 || max_c < 0.06
             || min_c > 0.08 || min_c < 0.03 ) {
            it.invalidateSegment();
            continue;
        }*/

        // Standard deviation
        accumulator_set<double, stats<tag::variance> > dists;
        it.resetBeams();
        while ( it.nextBeam()) {
            if ( !it.beam().valid )
                continue;
            dists((center - it.beam().pos).norm());
        }
        double std_dev = std::sqrt( variance( dists ));
        //std::cout << std_dev << std::endl;
        if ( std_dev > 0.05 ) {
            //it.invalidateSegment();
            continue;
        }

        /*double p, q, r;
        it.resetBeams();
        while ( it.nextBeam())
            if ( it.beam().valid) {
                p = it.beam().range;
                break;
            }
        while ( it.nextBeam())
            if ( it.beam().valid) {
                q = it.beam().range;
                break;
            }
        double conv = 0;
        count = 0;
        while ( it.nextBeam()) {
            if ( !it.beam().valid)
                continue;
            r = it.beam().range;
            if ( 0.5*(r + p) - 0.005 > q )
                conv++;
            p = q;
            q = r;
            count++;
        }
        //std::cout << conv << " " << conv/count << std::endl;
        if ( conv / count < 0.35 ) {
            //it.invalidateSegment();
           // continue;
        }*/

        /// @todo hack
        if ( com.norm() > 15.0 )
            continue;

        Leg leg;
        leg.pos = com;
        leg.is_single_leg = max < 0.12;
        legs_.push_back( leg );
    }
}

} // namespace
