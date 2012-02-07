/*
 * MouseFlodometry.cpp
 *
 *  Created on: Sep 30, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <math.h>

// Workspace
#include <Misc.h>

// Project
#include "Ramaxx.h"
#include "QuMessage.h"
#include "Adns.h"
#include "MouseFlodometry.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class OneWheelFlodometry /////////////////////////////////////////////////////////

//OneWheelFlodometry::OneWheelFlodometry() {
//    mDist = 0;
//    mScaleForward = mScaleBackward = 1.0;
//    mRot = M_PI + M_PI / 2.0;
//    mRawSumDx = mRawSumDy = 0;
//    mSumDx = mSumDy = 0;
//}
//
//void OneWheelFlodometry::processError( const AdnsError &error ) {
//    // Write an error message
//    cout << error.toString() << endl;
//}
//
//void OneWheelFlodometry::processPixdumpData( AdnsSensorId &id, const vector<int> &data ) {
//    // Ignore
//}
//
//void OneWheelFlodometry::processPositionData( AdnsSensorId &id, Uint time, const vector<AdnsPositionData> &data ) {
//
//    if ( data.size() <= 0 || abs( data[0].getDx()) > 255 || abs( data[0].getDy()) > 255 ) {
//        cout << "OneWheelFlodometry: Received invalid data." << endl;
//        return;
//    }
//    mRawSumDx += (double)data[0].getDx();
//    mRawSumDy += (double) data[0].getDy();
//
//    // Rotate
//    double dx = cos( mRot ) * (double)data[0].getDx() + sin( mRot ) * (double)data[0].getDy();
//    double dy = cos( mRot ) * (double)data[0].getDy() - sin( mRot ) * (double)data[0].getDx();
//
//    if ( dx == 0 ) {
//        return; // Nothing to do
//    }
//
//    // Scale & add distance
//    if ( dx > 0 ) {
//        dx *= mScaleForward;
//        dy *= mScaleForward;
//        mDist += sqrt( pow( dx, 2 ) + pow( dy, 2 ));
//    } else {
//        dx *= mScaleBackward;
//        dy *= mScaleBackward;
//        mDist -= sqrt( pow( dx, 2 ) + pow( dy, 2 ));
//    }
//
//    // Update all the sums
//    mSumDx += dx;
//    mSumDy += dy;
//
//    // Send measured distance back to the avr32
//    QuMessage distMsg;
//    distMsg.length = 4;
//    distMsg.type = MT_FLODO_DIST;
//    vector<U8> msgData;
//    msgData.resize( 4 );
//    S32 distCm = (S32)(mDist * 100.0);
//    msgData[0] = distCm >> 24;
//    msgData[1] = distCm >> 16;
//    msgData[2] = distCm >> 8;
//    msgData[3] = distCm;
//    distMsg.data = msgData;
//    Ramaxx::getRamaxx()->queueMsg( distMsg );
//}
//
//
//void OneWheelFlodometry::getSensorData (double& rawSumDx, double& rawSumDy ,
//        double& sumDx, double& sumDy ) const
//{
//    rawSumDx = mRawSumDx;
//    rawSumDy = mRawSumDy;
//    sumDx = mSumDx;
//    sumDy = mSumDy;
//
//}
//
//
//void OneWheelFlodometry::getOdometryPosition( player_pose2d &pose ) {
//    pose.px = mDist;
//}

//// class MouseFlodometry ////////////////////////////////////////////////////
/*
MouseFlodometry::MouseFlodometry()
{
    mPos.px = mPos.py = mPos.pa = 0.0;
    mVel.px = mVel.py = mVel.pa = 0.0;
    mValid = false;
    mNewData = false;
    mHeadingError = 0;
}

MouseFlodometry::~MouseFlodometry() {
    // Nothing to do
}

void MouseFlodometry::addChip( AdnsChip * chip ) {
    mChips.push_back( chip );
}

void MouseFlodometry::setData( AdnsPositionData &data, Uint time ) {
    for ( size_t i = 0; i < mChips.size(); ++i ) {
        if ( mChips[i]->setData( &data, time )) {
            mNewData = true;
        }
    }
}

void MouseFlodometry::setPosition( const player_pose2d &pos ) {
    mPos.px = pos.px;
    mPos.py = pos.py;
    mPos.pa = pos.pa;

    gettimeofday( &mLastUpdate, 0 );
}

void MouseFlodometry::getPositionData( player_position2d_data &posData ) {
    posData.pos = mPos;
    posData.vel = mVel;
}

void MouseFlodometry::processAdnsMsg( const QuMessage &msg ) {
    switch ( msg.type ) {
    case MT_ADNS_ERROR:
        { AdnsError error( msg );
        cout << " Ramaxx: ADNS ERROR: " << error.toString() << endl; }
        break;
    case MT_ADNS_POSITION:
        Uint timeStamp;
        adnsParsePositionMsg( msg, &mAdnsData, &timeStamp );
        for ( size_t i = 0; i < mAdnsData.size(); ++i ) {
            if ( mAdnsData[i].isValid()) {
                setData( mAdnsData[i], timeStamp );
            }
        }
        if ( mNewData ) {
            update();
        }
        break;
    case MT_ADNS_PIXDUMP:
        break;
    default:
        break;
    }
}

//// class LeastSquareFlodometry /////////////////////////////////////////////

LeastSquareFlodometry::LeastSquareFlodometry() {
    mHeadingError = 0;
    mCmpDataCount = 0;
    gettimeofday( &mLastUpdate, 0 );

    // Left sensor (chip number 0)
    int chipNumber = 0;
    double height = 11.1;   // Height above the ground [cm]
    AdnsSensorId id( 128, chipNumber );
    gsl_vector * chipPos = gsl_vector_alloc( 4 );
    gsl_vector_set_all( chipPos, 0 );
    gsl_vector_set( chipPos, 0, 0.0 );
    gsl_vector_set( chipPos, 1, 16.2 );
    AdnsChip * chip00 = new AdnsChip( id, chipPos, -M_PI / 2.0, height );
    addChip( chip00 );

    // Right sensor (chip number 1);
    chipNumber = 1;
    id.setChipNumber( chipNumber );
    gsl_vector_set( chipPos, 0, 0.0 );
    gsl_vector_set( chipPos, 1, -15.8 );
    AdnsChip * chip01 = new AdnsChip( id, chipPos, -M_PI / 2.0, height );
    addChip( chip01 );
}

LeastSquareFlodometry::~LeastSquareFlodometry() {
    // Nothing to do
}

void LeastSquareFlodometry::update() {

    if ( !mNewData ) {
        return; // No data to process
    }
    mNewData = false;

    double n = (double)mChips.size();
    gsl_vector * x;
    gsl_vector * c;

    double c0sum = 0;
    double c1sum = 0;
    double xc0sum = 0;
    double xc1sum = 0;
    double dataSum = 0;
    Uint minSq = 255;
    for ( size_t i = 0; i < mChips.size(); ++i ) {
        x = mChips[i]->getChipPos();
        c = mChips[i]->getData();

        c0sum += gsl_vector_get( c, 0 );
        c1sum += gsl_vector_get( c, 1 );
        xc0sum += gsl_vector_get( x, 0 );
        xc1sum += gsl_vector_get( x, 1 );

        dataSum += fabs( gsl_vector_get( c, 0 ));
        dataSum += fabs( gsl_vector_get( c, 1 ));

        if ( mChips[i]->getSq() < minSq ) {
            minSq = mChips[i]->getSq();
        }
    }
    xc0sum += c0sum;
    xc1sum += c1sum;
    c0sum /= n;
    c1sum /= n;
    xc0sum /= n;
    xc1sum /= n;

    if ( minSq < MIN_SQ ) {
        // We need valid data from all chips
        mValid = false;
        return;
    } else {
        mValid = true;
    }

    if ( dataSum < 1E-10 ) {
        // Nothing to do, all data zero
        mVel.px = mVel.py = 0.0;
        return;
    }

    double alpha = 0;
    double alphaDiv = 0;
    for ( size_t i = 0; i < mChips.size(); ++i ) {
        x = mChips[i]->getChipPos();
        c = mChips[i]->getData();

        alpha += gsl_vector_get( c, 0 ) * gsl_vector_get( x, 1 );
        alpha -= gsl_vector_get( c, 1 ) * gsl_vector_get( x, 0 );
        alpha += gsl_vector_get( c, 0 ) * c1sum;
        alpha -= gsl_vector_get( x, 1 ) * c0sum;
        alpha += c1sum * gsl_vector_get( x, 0 );
        alpha -= gsl_vector_get( c, 1 ) * c0sum;

        alphaDiv += pow( gsl_vector_get( x, 1 ) + c1sum, 2 );
        alphaDiv += pow( gsl_vector_get( x, 0 ) + c0sum, 2 );
        alphaDiv -= gsl_vector_get( c, 0 ) * gsl_vector_get( x, 1 );
        alphaDiv += gsl_vector_get( c, 1 ) * gsl_vector_get( x, 0 );
        alphaDiv -= xc1sum * gsl_vector_get( x, 1 );
        alphaDiv -= xc1sum * gsl_vector_get( x, 1 );
        alphaDiv -= xc0sum * gsl_vector_get( x, 0 );
        alphaDiv -= xc0sum * gsl_vector_get( c, 0 );
        alphaDiv -= xc1sum * gsl_vector_get( c, 2 );
    }

    alpha /= alphaDiv;

    double a0 = ( c0sum - ( alpha * xc1sum )) * 0.01; // [m]
    double a1 = ( c1sum + ( alpha * xc0sum )) * 0.01;

    // Paranoia
    if ( fabs( alpha ) > MAX_ALPHA_STEP ) {
        alpha = alpha > 0 ? MAX_ALPHA_STEP : -MAX_ALPHA_STEP;
    }
    double dist = sqrt( pow( a0, 2 ) + pow( a1, 2 ));
    if (  dist > MAX_DIST_SETP ) {
        a0 *= ( (double)MAX_DIST_SETP / dist );
        a1 *= ( (double)MAX_DIST_SETP / dist );
    }

    mOldPos = mPos;
    mPos.px += cos( mPos.pa ) * a0 + sin( mPos.pa ) * a1;
    mPos.py += cos( mPos.pa ) * a1 - sin( mPos.pa ) * a0;
    mPos.pa += alpha;

    Misc::normalizeAngle( mPos.pa );

    timeval now;
    gettimeofday( &now, 0 );
    double timediff = Misc::getTimeDiff( &mLastUpdate, &now );
    computeVelocity( mOldPos, mPos, timediff );

    gettimeofday( &mLastUpdate, 0 );
    mHeadingError +=  0.01 * ( dist / 0.35 );
}

double LeastSquareFlodometry::computeDistance( player_pose2d &oldPos, player_pose2d &newPos ) {
    return sqrt( pow( newPos.px - oldPos.px, 2 ) + pow( newPos.py - oldPos.py, 2 ));
}

void LeastSquareFlodometry::computeVelocity( player_pose2d oldPos, player_pose2d newPos, double timediff ) {
    mVel.px = ( newPos.px - oldPos.px ) * timediff;
    mVel.py = ( newPos.px - oldPos.py ) * timediff;
    mVel.pa = ( newPos.pa - oldPos.pa ) * timediff;
}*/
