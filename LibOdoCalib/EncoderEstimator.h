#ifndef ENCODERESTIMATOR_H
#define ENCODERESTIMATOR_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Eigen
#include "Eigen/Core"
using namespace Eigen;


///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class EncoderEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * Default contructor.
     */
    EncoderEstimator();

    /**
     * Set starting position and encoder values.
     *
     * @param pose Starting position.
     * @param ticksLeft Left encoder tick sum.
     * @param ticksRight Right encoder tick sum.
     */
    void SetStart( const Vector3d &pose, int ticksLeft, int ticksRight );

    /**
     * Computes new distance and calibration values.
     *
     * @param pose Current position of the robot (most likely the SLAM pose)
     * @param ticksLeft Left encoder tick sum.
     * @param tickRight Right encoder tick sum.
     */
    void Update( const Vector3d &pose, int ticksLeft, int ticksRight );

    /**
     * Returns left encoder calibration.
     *
     * @return Left encoder calibration factor [m/ticks]
     */
    double GetCalibrationLeft() const;

    /**
     * Return right encoder calibration.
     *
     * @return Right encoder calibration factor [m/ticks]
     */
    double GetCalibrationRight() const;

    /**
     * Returns the distance to the starting point.
     *
     * @return Distance [m]
     */
    double GetDistance() const;

private:
    /** Stating position. */
    Vector3d mStartPose,mLastPose;
    /** Encoder calibration factors */
    double mCalibLeft, mCalibRight;
    /** Travelled distance */
    double mDist;

    double mDeltaDist;
    /** Starting encoder values */
    int mStartTicksLeft, mStartTicksRight;
    int mTicksPerTurn;
};

#endif // ENCODERESTIMATOR_H
