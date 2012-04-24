
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>
#include <vector>

// Workspace
#include <utils/LibUtil/MathHelper.h>

// Project
#include "AngleOptimizer.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

AngleOptimizer::AngleOptimizer()
    : max_range_( 10.0 ), angle_range_( 0.5*M_PI ), angle_step_( 10.0*M_PI/180.0 )
{
}

void AngleOptimizer::optimize( const Eigen::Vector3d pose,
                               const lib_path::GridMap2d *map,
                               Eigen::Vector3d &new_pose )
{
    new_pose = pose;

    // Information gain for all calculated directions
    std::vector<InformationGainBeam> beam_vec;
    beam_vec.reserve( (size_t)(2.0*angle_range_/angle_step_) + 1 );

    // For all angles
    InformationGainBeam beam;
    double angle_delta = -angle_range_;
    while ( angle_delta < angle_range_ ) {
        beam.angle = pose(2) + angle_delta;
        beam.gain = calculateInformationGain( map, pose, angle_delta );
        beam_vec.push_back( beam );
        angle_delta += angle_step_;
    }

    // Select best
    if ( beam_vec.size() > 0 ) {
        std::sort( beam_vec.begin(), beam_vec.end());
        new_pose = pose;
        new_pose(2) = MathHelper::NormalizeAngle( beam_vec[ beam_vec.size() - 1].angle );
    }
}

double AngleOptimizer::calculateInformationGain(
        const lib_path::GridMap2d *map,
        const Eigen::Vector3d &pose,
        const double angle_delta )
{
    // Start/end pose for Bresenham
    lib_path::Point2d start( pose.x(), pose.y()), end;
    end.x = pose.x() + cos( pose(2) + angle_delta)*max_range_;
    end.y = pose.y() + sin( pose(2) + angle_delta)*max_range_;

    // End not on map?
    if ( !map->isInMap( end ))
        return 0; /// @todo Thats stupid. Fix this

    // Bresenham
    double info_gain = 0;
    bresenh_.set( map, start, end );
    while ( bresenh_.next()) {
        if ( bresenh_.isOccupied())
            return info_gain; // Stop on obstacles
        else if ( bresenh_.isNoInformation())
            info_gain += 1.0;
    }

    return info_gain;
}

} // namespace


