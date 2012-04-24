#ifndef ANGLEOPTIMIZER_H
#define ANGLEOPTIMIZER_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// ROS
#include <Eigen/Core>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>
#include <utils/LibPath/common/Bresenham2d.h>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

/**
 * @brief Internally used
 */
struct InformationGainBeam
{
    /// Caclulated information gain (number of cells with no information)
    double gain;

    /// Angle of that beam
    double angle;

    bool operator<( const InformationGainBeam& o ) const { return gain < o.gain; }

};

class AngleOptimizer
{
public:
    AngleOptimizer();

    void optimize( const Eigen::Vector3d pose,
              const lib_path::GridMap2d *map,
              Eigen::Vector3d &new_pose );

private:

    double calculateInformationGain( const lib_path::GridMap2d *map,
                                     const Eigen::Vector3d &pose,
                                     const double angle_delta );

    double max_range_;

    double angle_range_;

    double angle_step_;

    lib_path::Bresenham2d bresenh_;
};

} // namespace

#endif // ANGLEOPTIMIZER_H
