/*
 * MouseFlodometry.h
 *
 *  Created on: Sep 30, 2009
 *      Author: marks
 */

#ifndef MOUSEFLODOMETRY_H_
#define MOUSEFLODOMETRY_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <string>
#include <sys/time.h>
#include <fstream>

// GSL
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

// Player
//#include <libplayercore/playercore.h>

// Workspace
#include <Global.h>

// Project
#include "QuMessage.h"
#include "Adns.h"
#include "Compass.h"
#include "Odometry.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Minimum SQ value of valid sensor data
#define MIN_SQ 60

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

//class OneWheelFlodometry : public AdnsListener, public Odometry {
//public:
//
//    /**
//     * Constructor.
//     */
//    OneWheelFlodometry();
//
//    /**
//     * Destructor.
//     */
//    virtual ~OneWheelFlodometry() {};
//
//    /**
//     * Returns the travelled distance in meters.
//     *
//     * @return
//     *      The distance in [m].
//     */
//    virtual double getTravelledDistance() const { return mDist; }
//
//    virtual void getPosition( player_pose2d * pos ) { /* TODO */ }
//
//    /**
//     * Processes new position data.
//     *
//     * @param data  The new position data.
//     */
//    virtual void processPositionData( AdnsSensorId &id, Uint time, const vector<AdnsPositionData>& data );
//
//    /**
//     * Processes a new error message.
//     *
//     * @param error The error message.
//     */
//    virtual void processError( const AdnsError &error );
//
//    /**
//     * Processes new pixdump data.
//     *
//     * @param id The sensor id.
//     * @param data  The pixdump data.
//     */
//    virtual void processPixdumpData( AdnsSensorId &id, const vector<int> &data );
//
//    /**
//     * Writes the driven distance to the x coordinate of the given variable.
//     *
//     * @param pose The current odometry data will be written to this variable.
//     */
//    void getOdometryPosition( player_pose2d &pose );
//
//    /**
//     * Set the rotation of the ADNS sensor.
//     *
//     * @param alpha The rotation in radian.
//     */
//    void setSensorRotation( double alpha ) { mRot = alpha; }
//
//    /**
//     * Set the forward scale factor of the ADNS sensor.
//     *
//     * @param scale Scale factor.
//     */
//    void setSensorScaleForward( double scale ) { mScaleForward = scale; }
//
//    /**
//     * Set the backward scale factor of the ADNS sensor.
//     *
//     * @param scale Scale factor.
//     */
//    void setSensorScaleBackward( double scale ) { mScaleBackward = scale; }
//
//    /**
//     * return raw mouse sensor data
//     *
//     *
//     */
//    void getSensorData (double& rawSumDx, double& rawSumDy ,double& sumDx, double& sumDy ) const;
//
//
//
//private:
//    /** The distance traveled in meters. */
//    double mDist;
//    /** Rotation of sensor zero. */
//    double mRot;
//    /** Forward scale of sensor zero. */
//    double mScaleForward;
//    /** Backward scale of sensor zero. */
//    double mScaleBackward;
//    /** cumulated raw mouse sensor dx/dy */
//    double mRawSumDx, mRawSumDy;
//    double mSumDx, mSumDy;
//};

/*
class MouseFlodometry {
public:
    MouseFlodometry();
    virtual ~MouseFlodometry();

    virtual void update() = 0;
    virtual void processAdnsMsg( const QuMessage &msg );
    virtual void setData( AdnsPositionData& data, Uint time );
    //void addChip( AdnsChip * chip );
    void setPosition( const player_pose2d &pos );
    bool isValid() { return mValid; }
    const player_pose2d& getPosition() { return mPos; }
    void getPosition( player_pose2d & pos ) { pos = mPos; }
    double getHeadingError() { return mHeadingError; }
    void setHeadingError( double headingErr ) { mHeadingError = headingErr; }
    double getHeading() { return mPos.pa; }
    void setHeading( double heading ) { mPos.pa = heading; }
    void getPositionData( player_position2d_data& posData );

protected:
    vector<AdnsPositionData> mAdnsData;
    bool mValid;
    bool mNewData;
    //vector<AdnsChip*> mChips;
    player_pose2d mPos;
    player_pose2d mVel;
    double mHeadingError;
    timeval mLastUpdate;
};

class LeastSquareFlodometry : public MouseFlodometry {
public:
    LeastSquareFlodometry();
    virtual ~LeastSquareFlodometry();

    virtual void update();

private:
    double computeDistance( player_pose2d &oldPos, player_pose2d &newPos );
    void computeVelocity( player_pose2d oldPos, player_pose2d newPos, double timediff );

    player_pose2d mOldPos;
    double mCmpDataCount;
    double mCmpZero;
};
*/

#endif /* MOUSEFLODOMETRY_H_ */
