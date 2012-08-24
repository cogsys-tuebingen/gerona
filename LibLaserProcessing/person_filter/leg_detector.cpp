/**
 * (c) Cognitive Systems, University of Tübingen
 *
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
}

void LegDetector::update( DistanceSegmentation &segms )
{
    legs_.clear();

    // For each segment
    DistanceSegmentation::Iterator it = segms.iterator();
    while ( it.nextSegment()) {
        // Enough beams?
        if ( it.length() < min_beams_ )
            continue;

        /// @todo Simple check if a segment is far too large

        // Compute center of mass (COM)
        Vector2d com( Vector2d::Zero());
        while ( it.nextBeam())
            com += it.beam().pos;
        com /= (double)it.length();

        // Compute maximum and average distance to COM
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
            if ( max < d ) max = d;
            count++;
        }
        avr /= count;

        // Check maximum distance
        if ( max > 0.2 )
            continue;

        // Estimated leg center
        Vector2d center( com + 0.04*com.normalized());

        /*double min_c = std::numeric_limits<double>::max();
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
        avr_c /= count;*/

        // Standard deviation
        accumulator_set<double, stats<tag::variance> > dists;
        it.resetBeams();
        while ( it.nextBeam()) {
            if ( !it.beam().valid )
                continue;
            dists((center - it.beam().pos).norm());
        }
        double std_dev = std::sqrt( variance( dists ));

        // Check std dev
        if ( std_dev > 0.05 )
            continue;

        /// @todo This is a hack. Set a proper maximum distance somewhere
        if ( com.norm() > 15.0 )
            continue;

        // Well, this segment might be a leg...
        Leg leg;
        leg.pos = com;
        leg.is_single_leg = max < 0.12; /// @todo Think about this
        legs_.push_back( leg );
    }
}

} // namespace
