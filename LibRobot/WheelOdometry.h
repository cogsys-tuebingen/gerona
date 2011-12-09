/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2010

 @file WheelOdometry.h
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Eigen
#define EIGEN2_SUPPORT
#include "Eigen/Core"

// Import most common Eigen types
using namespace Eigen;


// Project
#include "AckermannSteering.h"


///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

namespace Ra {

/**
 * Odometry using steer angles and wheel encoder measurements.
 */
class WheelOdometry {
public:
    
    /**
     * Constructor.
     *
     * @param wheelbase The robots wheelbase [m]. Defaults to 0.34.
     * @param robotWidth Distance between the wheels on the rear axis [m].
     */
    WheelOdometry( double wheelbase = 0.34, double robotWidth = 0.325 );

    /**
     * Computes the new position of the robot.
     *
     * @param frontSteer Front steer angle [rad]. 
     * @param rearSteer Rear steer angle [rad].
     * @param dt Time since last update [s]. Used to compute the velocity.
     * @param leftEncoder Rear left wheel encoder distance delta [m].
     * @param rightEncoder Rear right encoder distance delta [m].
     */
    void Update( double frontSteer, double rearSteer, double dt, double leftEncoder, double rightEncoder );
    
    /**
     * Computes the new position of the robot.
     *
     * @param frontSteer Front steer angle [rad]. 
     * @param rearSteer Rear steer angle [rad].
     * @param dt Time since last update [s]. Used to compute the velocity.
     * @param encoder Wheel encoder distance delta [m].
     * @param leftEncoder True if the wheel encoder is mounted on the rear left 
     * wheel. False otherwise.
     */
    void Update( double frontSteer, double rearSteer, double dt, double encoder, bool leftEncoder = true );
    
    /**
     * Return the latest position estimation in world coordinates.
     *
     * @param pose The result will be written to this variable.
     */
    void GetPose( Vector3d &pose ) const;
    
    /**
     * Get the latest position delta in robot coordinates.
     *
     * @param pose The position delta will be written to this parameter.
     */ 
    void GetDeltaPose( Vector3d &pose ) const;
    
    /**
     * Sets the current position estimation.
     *
     * @param pose The new position in world coordinates.
     */
    void SetPose( const Vector3d &pose );

    /**
     * Sets the wheelbase.
     *
     * @param wheelbase New wheelbase [m].
     */
    void SetRobotWheelbase( const double &wheelbase );
    
    /**
     * Returns the wheelbase of the robot.
     *
     * @return The wheelbase.
     */
    double GetRobotWheelbase() const;

    /**
     * Sets the distance between the wheel on the rear axis.
     *
     * @param robotWidth New robot width [m].
     */
    void SetRobotWidth( const double &robotWidth );
    
    /**
     * Returns the distance between the wheel on the rear axis.
     *
     * @return Robot width [m].
     */
    double GetRobotWidth() const;

    /**
     * Returns the velocity vector of the robot in world coordinates.
     *
     * @param velo The velocity will be written to this parameter.
     */
    void GetVelocity( Vector3d &velo ) const;
    
    /**
     * Returns the velocity of the robot.
     *
     * @return The velocity. Negative if the robot is driving backwards.
     */
    double GetVelocity() const;

private:

    void InternalUpdate( const double &dt, const double &dist );
    void Rotate( const double &phi, const double &dt );
    void UpdatePose( const Vector3d &deltaPose, const double &dt );

    Vector3d            mPose;
    Vector3d            mDeltaPose;
    Vector3d            mVelocity;
    double              mVelocityNorm;
    double              mRobotWidth;
    AckermannSteering   mSteerModel;
};

}; // namespace Ra

#endif // ODOMETRY_H
