/**
 * (c) Cognitive Systems, University of Tübingen
 *
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
#include <utils/LibLaserProcessing/laser_beam.h>
#include <utils/LibLaserProcessing/segmentation/distance_segmentation.h>
#include <utils/LibLaserProcessing/person_filter/leg.h>

namespace lib_laser_processing {

/**
 * @brief Very simple human leg classifier.
 */
class LegDetector
{
public:
    /**
     * @brief Create and initialize with default values.
     */
    LegDetector();

    /**
     * @brief Process and classify segments
     * @param segments The new segments
     */
    void update( DistanceSegmentation& segments );

    /**
     * @brief Return all legs
     * @return All segments that have been classified as legs
     */
    const std::vector<Leg>& getLegs() const {
        return legs_;
    }

private:

    /// Minimum number of beams per leg
    unsigned int min_beams_;

    /// All segements that have been classified as legs
    std::vector<Leg> legs_;
};

}

#endif // LEG_DETECTOR_H
