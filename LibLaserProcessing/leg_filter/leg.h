#ifndef LEG_H
#define LEG_H

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

namespace lib_laser_processing {

class Leg {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector2d pos;

    bool is_single_leg;
};

} // namespace

#endif // LEG_H
