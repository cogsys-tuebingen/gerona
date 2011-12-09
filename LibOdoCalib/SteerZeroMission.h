#ifndef STEERZEROMISSION_H
#define STEERZEROMISSION_H

#include "OdoCalibration.h"
#include "DistanceTracker.h"
#include "RobotProxy.h"
#include "Eigen/Core"

using namespace Eigen;


class SteerZeroMission : public RobotMission
{
public:

    SteerZeroMission( RobotProxy *robot );

    /* Inherited from CalibrationMission */
    bool Execute();

    /* Inherited from CalibrationMission */
    bool Done();

private:
    RobotProxy *mRobot;
    float mSteerRad;
    float mSpeed;
    DistanceTracker mDist;
    bool mDone;
};

#endif // STEERZEROMISSION_H
