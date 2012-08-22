/**
 * @file leg_detector.h
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef LEG_DETECTOR_H
#define LEG_DETECTOR_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

// Project
#include <utils/LibLaserProcessing/LaserBeam.h>
#include <utils/LibLaserProcessing/segmentation/distance_segmentation.h>
#include <utils/LibLaserProcessing/leg_filter/leg.h>

namespace lib_laser_processing {

class LegDetector
{
public:
    LegDetector();

    void update( DistanceSegmentation& segments );

    const std::vector<Leg>& getLegs() const {
        return legs_;
    }

private:

    /// Minimum number of beams per leg
    unsigned int min_beams_;

    std::vector<Leg> legs_;
};

}

#endif // LEG_DETECTOR_H
