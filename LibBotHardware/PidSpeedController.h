/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Bohlmann, Marks
 @date 2010

 @file PidSpeedController.h
 */

#ifndef PIDSPEEDCONTROLLER_H
#define PIDSPEEDCONTROLLER_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "LowPassFilter.h"
#include "LogCollector.h"
#include "SpeedController.h"
#include "UsbConn.h"

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// TODO Merge into SpeedController.h ?

/**
 * Speed controller for ramaxx. Pid-based
 */
class PidSpeedController : public SpeedController {
public:

    /**
     * Contructor.
     *
     * @param conn Connection to the robot.
     * @param actuators The robots actuators.
     * @param odo Odometry object (reports the current velocity).
     */
    PidSpeedController( RamaxxConnection * conn,
                        Actuators * actuators,
                        Odometry * odo );

    /**
     * Destructor.
     */
    virtual ~PidSpeedController();

    /* Inherited from SpeedController */
    void setCalibration( const SpeedCtrlCalibration &calib );

    /* Inherited from SpeedController */
    void setSpeed( float speed );

    /* Inherited from SpeedController */
    float getSpeed() const { return mTargetSpeed; }

    /* Inherited from SpeedController */
    void update( bool immediately = false );

    /* Inherited from SpeedController */
    void deactivate();

    /* Inherited from DataListener */
    void dataChanged( const DataChangeEvent &event );


private:
    /** Possible controller states */
    enum ControllerState { STANDING, FORWARD, SWITCHBRAKE, SLOWREVERSE, REVERSE, STALLED };


   /**
    * Compute and send speed servo value.
    *
    * @param speedMSec robot speed in m/sec
    */
    void setMotorSpeed (double speedMSec);

    /**
     * Compute new speed value (pid based).
     *
     * @param targetSpeed Current target speed.
     * @param speedDiff Difference between target speed and actual speed.
     * @param lastSpeedDiff Difference between target speed and actual speed (last regulation step).
     * @param timeDiff Time since last regulation step [s].
     *
     * @return New speed value [m/s].
     */
    inline double regulate (
            const double targetSpeed,
            const double speedDiff,
            const double lastSpeedDiff,
            const double timeDiff ) const;

    /**
     * Resets the pid regulator (clear error integral and set last speed difference to zero).
     */
    void pidReset();

    /** The robots actuators. */
    Actuators * mActuators;
    /** Object doing odometry. */
    Odometry * mOdo;
    /** Demanded velocity in meter per second. */
    double mTargetSpeed;
    /** Velocity as reported by the odometry (average over 2 values). */
    LowPassFilter<float> mSpeedFilter;
    /** Velocity as reported by the odometry (average over 10 values). */
    LowPassFilter<float> mLtSpeedFilter;
    /** The current state of the controller. */
    ControllerState mState;
    /** Used to determine the time sine the last update. */
    Stopwatch mUpdateTimer;
    /** Used to send zero speed values during the switching state. */
    Stopwatch mSlowreverseTimer;
    /** Speed error intergal */
    double mIntegralE;
    /** True if the speed controller is deactivated. */
    bool mDeactivated;
    /** True if the odometry system reports valid data. False otherwise. */
    bool mOdoValid;
    /** Difference between target speed and actual speed (last regulation step) */
    double mLastSpeedDiff;
    /** Allow eddy current brake (equals EVX is in forward mode) */
    bool mAllowEddyBrake;
    /** Stalled? */
    bool mStall;

    // development
    LogCollector mLog;
};


#endif // PIDSPEEDCONTROLLER_H
