/**
 * @file leg_detector.h
 * @date Aug 2012
 * @author marks, buck
 */

#ifndef LEG_DETECTOR_H
#define LEG_DETECTOR_H

// Eigen
#include <Eigen/Core>

// Project
#include <utils/LibLaserProcessing/LaserBeam.h>
#include <utils/LibLaserProcessing/segmentation/distance_segmentation.h>

namespace lib_laser_processing {

class LegDetector
{
public:
    LegDetector();

    void update( DistanceSegmentation& segments );

private:

    /// Minimum number of beams per leg
    unsigned int min_beams_;
};

}

#endif // LEG_DETECTOR_H
