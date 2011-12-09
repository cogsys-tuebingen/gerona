#ifndef STRAIGHTLINEDRIVER_H
#define STRAIGHTLINEDRIVER_H

////////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
////////////////////////////////////////////////////////////////////////////////

// Workspace
#include <Stopwatch.h>

// Project
#include "CalibBotInterface.h"

////////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
////////////////////////////////////////////////////////////////////////////////

/**
 * Drive a straight line with a given minimum length. Uses SLAM to
 * control the (front) steer angle.
 */
class StraightLineDriver {
public:

    /**
     * Contructor.
     *
     * @param proxy Connection to the robot. Used to get SLAM updates and
     * to set speed/steer. This object wont call the Read() method!
     */
    StraightLineDriver( CalibBotInterface *proxy );

    /**
     * Sets the starting point of the line (most likely the
     * current position of the robot).
     *
     * @param pose The starting pose.
     */
    void SetStart( const Vector3d &pose );

    /**
     * Sets the speed of the robot (negative values to drive reverse).
     *
     * @param speed The requested speed in [m/s].
     */
    void SetSpeed( double speed );

    /**
     * Sets the minimum length of the straight line. After reaching the requested
     * length the speed of the robot will be set to zero.
     *
     * @param length The minimum length of the straight line [m].
     */
    void SetLength( double length );

    /**
     * Compute new steer angle and set drive params. The driver will stop the robot if
     * the requested length is reached. Call this method if a fresh SLAM pose is
     * available but at least every 0.5 seconds.
     *
     * @return True if the requested minimum length was reached AND the robot stopped.
     */
    bool Update();

private:
    /** Connection to the robot. */
    CalibBotInterface *mProxy;
    /** Starting point of the line */
    Vector3d mStartPose;
    /** Requested minimum length [m] */
    double mLength;
    /** Requested speed of the robot [m/s] */
    double mSpeed;
    /** Last set steering angle */
    double mSteer;
    /** Steer control proportional parameter */
    double mSteerP;
    /** Steer control integral parameter */
    double mSteerI;
    /** Error integral */
    double mSteerErrorI;
    /** Used to measure the time between two updates. */
    Stopwatch mUpdateWatch;

    double mZeroForw;
    double mZeroRev;
    double mSteerSum;
    double mSteerCount;
};

#endif // STRAIGHTLINEDRIVER_H
