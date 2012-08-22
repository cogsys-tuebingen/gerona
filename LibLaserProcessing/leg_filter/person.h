#ifndef PERSON_H
#define PERSON_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

// Project
#include <utils/LibLaserProcessing/leg_filter/leg.h>

namespace lib_laser_processing {

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
