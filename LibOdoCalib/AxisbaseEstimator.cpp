
// C/C++
#include <cmath>
#include <fstream>

// Workspace
#include <runningstat.h>

// Project
#include "AxisbaseEstimator.h"

using namespace std;

double AxisbaseEstimator::calcAxisbase( double wheelbase, double enc_coeff_left, double enc_coeff_right )
{
    // Logfile
    ofstream ofs( "/tmp/axisbase_log.dat" );

    // Logfile header
    ofs << "# Axisbase Estimator Logfile\n"
        << "# Wheelbase = " << wheelbase << "\n"
        << "# Left encoder coeff = " << enc_coeff_left << "\n"
        << "# Right encoder coeff = " << enc_coeff_right << "\n"
        << "# Radius Ticks_Left Ticks_Right Effective_Radius Dist_Left Dist_Right Axisbase" << endl;

    // Mean and standard deviation
    RunningStat stats;

    // For each circle data set
    for ( size_t i = 0; i < data_.size(); ++i ) {
        CircleData d = data_[i];

        // Effective circle radius
        double r_eff = sqrt( pow( d.radius, 2 ) + 0.25*pow( wheelbase, 2 ));

        // Left encoder distance
        double d_l = enc_coeff_left*(d.end_ticks_left - d.start_ticks_left);

        // Right encoder distance
        double d_r = enc_coeff_right*(d.end_ticks_right - d.start_ticks_right);

        // Axisbase
        double axisbase = 2.0*fabs( r_eff*(d_l - d_r)/(d_l + d_r));

        // Log data
        ofs << d.radius << "\t"
            << (d.end_ticks_left - d.start_ticks_left) << "\t"
            << (d.end_ticks_right - d.start_ticks_right) << "\t"
            << r_eff << "\t"
            << d_l << "\t"
            << d_r << "\t"
            << axisbase << endl;

        // Add to stats estimator
        stats.Push( axisbase );
    }

    // Stats
    ofs << "# Mean axisbase = " << stats.Mean() << "\n"
        << "# Standard deviation axisbase = " << stats.StandardDeviation() << endl;

    return stats.Mean();
}

void AxisbaseEstimator::finishCircle( double circle_r, double ticks_left, double ticks_right )
{
    CircleData d;
    d.radius = circle_r;
    d.start_ticks_left = start_ticks_left_;
    d.end_ticks_left = ticks_left;
    d.start_ticks_right = start_ticks_right_;
    d.end_ticks_right = ticks_right;
    data_.push_back( d );
}

void AxisbaseEstimator::startCircle( double ticks_left, double ticks_right )
{
    start_ticks_left_ = ticks_left;
    start_ticks_right_ = ticks_right;
}
