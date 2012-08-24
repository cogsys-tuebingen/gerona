/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

#ifndef PERSON_H
#define PERSON_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

// Project
#include <utils/LibLaserProcessing/person_filter/leg.h>

namespace lib_laser_processing {

/**
 * @brief Represents a detected person. A person has one or two legs
 * and a position.
 */
class Person
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Estimated position
    Eigen::Vector2d pos;

    /// Legs corresponding to this person
    std::vector<Leg> legs;
};

} // namespace

#endif // PERSON_H
