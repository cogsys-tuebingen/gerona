/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2010

 @file AckermannSteering.h
 */

#ifndef ACKERMANNSTEERING_H
#define ACKERMANNSTEERING_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#ifndef EIGEN2_SUPPORT
#define EIGEN2_SUPPORT
#endif
// Eigen
#include "Eigen/Core"

// Import most common Eigen types
using namespace Eigen;



///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

namespace Ra {

/**
 * Represents a car with Ackermann steering.
 */
class AckermannSteering {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Possible turning directions. */
    enum TurnDirection { LEFT, NONE, PARALLEL, RIGHT };

    /**
     * Constructor. Initial wheelbase is 0.34 meter, the steering angles default
     * to zero degree.
     */
    AckermannSteering();

    /**
     * Sets the current steer angles.
     *
     * @param front Front steer angle [rad].
     * @param rear Rear steer angle [rad].
     */
    void SetSteerAngles( const double &front, const double &rear );

    /**
     * Sets the current steer angles.
     *
     * @param front Front steer angle [degree].
     * @param rear Rear steer angle [degree].
     */
    void SetSteerAnglesDeg( const double &frontDeg, const double &rearDeg );

    /**
     * Return rear steer angle.
     *
     * @return Rear steer angle [rad].
     */
    double GetRearSteer() const { return mSteerRear; }

    /**
     * Return front steer angle.
     *
     * @return Front steer angle [rad].
     */
    double GetFrontSteer() const { return mSteerFront; }

    /**
     * Sets the robots wheelbase.
     *
     * @param wheelbase The new wheelbase [m].
     */
    void SetWheelbase( const double &wheelbase );

    /**
     * Return the robots wheelbase.
     *
     * @return The wheelbase [m].
     */
    double GetWheelbase() const { return mWheelbase; }

    /**
     * Returns the turning radius of the robots front wheels.
     *
     * @return The front turning radius [m]
     */
    double GetFrontRadius() const { return mFrontRadius; }

    /**
     * Returns the turning radius of the robots center point.
     *
     * @return The center turning radius [m]
     */
    double GetCenterRadius() const { return mCenterRadius; }

    /**
     * Returns the turning radius of the robots rear wheels.
     *
     * @return The rear turning radius [m].
     */
    double GetRearRadius() const { return mRearRadius; }
    
    /**
     * Returns the instantous center of rotation if there is any rotation. 
     * TODO doc
     *
     * @param vector The result will be written to this variable.
     */
//    void GetIcpCoordinates( Vector2d &vector ) const { vector = mCenter; }
    void GetIcpCoordinates( Vector2d &vector ) const { vector[0] = mCenterX; vector[1] = mCenterY; }

    /**
     * Is the robot turning?
     *
     * @return The current turning direction (left, right or none).
     */
    TurnDirection GetTurn() const { return mTurn; }


private:

    double          mFrontRadius, mCenterRadius, mRearRadius;
    TurnDirection   mTurn;
    double          mWheelbase;
    double          mSteerFront, mSteerRear;
   // Vector2d        mCenter;
    double          mCenterX;
    double          mCenterY;
};

}; // namespace Ra

#endif // ACKERMANNSTEERING_H
