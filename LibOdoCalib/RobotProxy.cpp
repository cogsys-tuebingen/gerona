
#include "RobotProxy.h"
#include <iostream>
#include <math.h>
#include <libplayercommon/error.h>

using namespace std;

RobotProxy::RobotProxy( PlayerClient *robot, int odoIfNum, int slamIfNum, int rawOdoIfNum ) {
    // Init member
    mRobot = robot;
    mSlamPose = mOdoPose = Vector3d::Zero();

    // Create odometry proxy
    mOdoProxy = new Position2dProxy( robot, odoIfNum );
    mOdoProxy->NotFresh();

    // Create slam localize proxy
    mSlamProxy = new LocalizeProxy( robot, slamIfNum );
    mSlamProxy->NotFresh();

    // TODO Create raw odometry proxy

}

void RobotProxy::Read() {
    // Unset fresh data flag
    mOdoProxy->NotFresh();
    mSlamProxy->NotFresh();

    // Read new data
    mRobot->Read();

    if ( mOdoProxy->IsValid() && mOdoProxy->IsFresh()) {
        // Set new odometry position
        mOdoPose[0] = mPosProxy->GetXPos();
        mOdoPose[1] = mPosProxy->GetYPos();
        mOdoPose[2] = mPosProxy->GetYaw();
    }

    if ( mSlamProxy->IsValid() && mSlamProxy->IsFresh()) {
        // Set new slam position
        if ( mSlamProxy->GetHypothCount() > 0 ) {
            player_pose2d meanPose = mSlamProxy->GetHypoth( 0 ).mean;
            mSlamPose[0] = meanPose.px;
            mSlamPose[1] = meanPose.py;
            mSlamPose[2] = meanPose.pa;
        } else {
            PLAYER_ERROR( "Slam localize interface published zero hypothesis" );
        }
    }
}

void RobotProxy::GetOdometryPose( Vector3d &pose ) const {
    pose = mOdoPose;
}

void RobotProxy::GetSlamPose( Vector3f &pose ) {
    pose = mSlamPose;
}

bool RobotProxy::IsFreshOdometryPose() const {
    return mOdoProxy->IsFresh();
}

bool RobotProxy::IsFreshSlamPose() const {
    return mSlamProxy->IsFresh();
}

void RobotProxy::SetCarlike( const float &speed, const float &steerRad ) {
    mPosProxy->SetCarlike( speed, steerRad );
}

PlayerClient* RobotProxy::GetPlayerClient() {
    return mRobot;
}


