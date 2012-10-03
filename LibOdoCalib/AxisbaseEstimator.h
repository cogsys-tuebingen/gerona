#ifndef AXISBASEESTIMATOR_H
#define AXISBASEESTIMATOR_H

// C/C++
#include <vector>

class AxisbaseEstimator
{
public:

    struct CircleData
    {
        double radius;
        double start_ticks_left;
        double end_ticks_left;
        double start_ticks_right;
        double end_ticks_right;
    };

    void startCircle( double ticks_left, double ticks_right );
    void finishCircle( double circle_r, double ticks_left, double ticks_right );
    double calcAxisbase( double wheelbase, double enc_coeff_left, double enc_coeff_right );

private:

    std::vector<CircleData> data_;
    double start_ticks_left_, start_ticks_right_;
};

#endif // AXISBASEESTIMATOR_H
