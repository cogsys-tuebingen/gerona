/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

#ifndef LEG_H
#define LEG_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

namespace lib_laser_processing {

/**
 * @brief Represents one detcted leg
 */
class Leg
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Position
    Eigen::Vector2d pos;

    /// One leg? Probaly two legs?
    bool is_single_leg;
};

} // namespace

#endif // LEG_H
