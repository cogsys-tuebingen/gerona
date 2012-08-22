#ifndef PERSON_PROBABILITY_H
#define PERSON_PROBABILITY_H

// Eigen
#include <Eigen/Core>

// Project
#include <utils/LibLaserProcessing/leg_filter/person.h>

namespace lib_laser_processing {

class PersonProbability
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Estimated position
    Eigen::Vector2d pos;

    /// Estimated velocity
    Eigen::Vector2d vel;

    /// First position
    Eigen::Vector2d first_seen;

    Eigen::Vector4d state;

    Eigen::Matrix4d cov;

};

} // namespace

#endif // PERSON_PROBABILITY_H
