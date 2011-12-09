#ifndef ROBOTPROXY_H
#define ROBOTPROXY_H

#include <libplayerc++/playerc++.h>
#include "Eigen/Core"

using namespace PlayerCc;
using namespace Eigen;


class RobotProxy {

public:
    RobotProxy( PlayerClient *robot, int odoIfNum = 0, int slamIfNum = 1, int rawOdoIfNum = 2 );

    void Read();

    void GetOdometryPose( Vector3d &pose ) const;
    void GetSlamPose( Vector3d &pose ) const;

    bool IsFreshSlamPose() const;
    bool IsFreshOdometryPose() const;

    PlayerClient* GetPlayerClient();

    void SetCarlike( const float &speed, const float &steer );

private:
    PlayerClient    *mRobot;
    Position2dProxy *mOdoProxy;
    LocalizeProxy   *mSlamProxy;
    double           mTravelledDistance;
    Vector3d        mOdoPose;
    Vector3d        mSlamPose;
};

#endif // ROBOTPROXY_H
