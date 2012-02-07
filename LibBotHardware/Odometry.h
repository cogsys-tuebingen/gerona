/*
 * RamaxxOdometry.h
 *
 *  Created on: Feb 1, 2010
 *      Author: marks
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "RobotSensors.h"
#include "DataListener.h"
#include "../LibRobot/WheelOdometry.h"
#include "SteerServo.h"
#include "Compass.h"

// Eigen vector math
#include "Eigen/Core"

// Import most common Eigen types
using namespace Eigen;



///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

/** Variance of the heading in radian per meter. */
#define ODO_VARIANCE_SYS (( 5.0 * M_PI/180.0 )*( 5.0 * M_PI/180.0 ))

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Interface class for objects calculating the position of the robot.
 */
class Odometry : public DataOwner {
public:

    /**
     * Destructor.
     */
    virtual ~Odometry() {}

    /**
     * Get the current position.
     *
     * @param pose The data will be written to this parameter.
     */
    virtual void getPosition( Vector3d &pose ) const = 0;


    /**
     * Get the current uncorrected (from relative sensors only) position
     * @param pose The data will be written to this parameter.
     */
    virtual void getRawPosition ( Vector3d &pose ) const = 0;


    /**
     * Get the current velocity.
     *
     * @param vel The data will be written to this parameter.
     */
    virtual void getVelocity( Vector3d &vel ) const = 0;

    /**
     * Returns the norm of the current velocity.
     *
     * @return
     *     The norm of the velocity (with negative sign if we are driving backwards).
     */
    virtual double getVelocity() const = 0;

    /**
     * Returns the travelled distance in meters.
     *
     * @return Complete travelled distance [m].
     */
    virtual double getTravelledDistance() const = 0;

    /**
     * Set the position.
     *
     * @param pos The new position.
     */
    virtual void setPosition( const Vector3d &pos ) = 0;

};

/**
 * Computes position and velocity based on sensor data.
 */
class RamaxxOdometry : public Odometry, public DataListener {
public:

    /**
     * Constructor. Position defaults to (0, 0, 0).
     *
     * @param sensors The robots sensors.
     * @param frontSteer Front steer controller
     * @param rearSteer Rear steer controller
     */
    RamaxxOdometry( RobotSensors * sensors, SteerServo * frontSteer, SteerServo * rearSteer );

    /**
     * Destructor.
     */
    virtual ~RamaxxOdometry() { /* Nothing to do. */ };

    /* Inherited from Odoemtry */
    void getPosition( Vector3d &pose ) const;

    void getRawPosition ( Vector3d &pose ) const;

    /* Inherited from Odoemtry */
    void getVelocity( Vector3d &vel ) const;

    /* Inherited from Odoemtry */
    double getVelocity() const;

    /* Inherited from Odoemtry */
    double getTravelledDistance() const;

    /* Inherited from Odoemtry */
    void setPosition( const Vector3d &pos );

    /* Inherited from DataListener. */
    void dataChanged( const DataChangeEvent &event );

private:

    /**
     * Use compass data to correct the heading direction. Kalman based.
     *
     * @param compass The compass sensor.
     */
    void correctHeading( Compass *compass );

    void processWheelEncoderEvent( WheelEncoder * encoder, const DataChangeEvent &event );
    void processPniCompassEvent( Compass * compass, const DataChangeEvent &event );
    void processI2cCompassEvent( Compass *compass, const DataChangeEvent &event );


    /** The sensors of the robot. */
    RobotSensors * mSensors;
    /** Computes the position for given steer angles and wheel encoder data. */
    Ra::WheelOdometry mWheelOdo, mRawWheelOdo;
    /** Current heading variance */
    double mHeadingError;
    /** Front steer controller */
    SteerServo * mSteerFront;
    /** Rear steer controller */
    SteerServo * mSteerRear;
    /** Current position and heading direction */
    Vector3d mPose;
    /** Current position and heading direction uncorrected */
    Vector3d mRawPose;

    /** Current velocity */
    Vector3d mVelo;
    /** Current velocity in m/s, with negative sign if we are driving backwards */
    double mVelNorm;
};

#endif /* ODOMETRY_H_ */
