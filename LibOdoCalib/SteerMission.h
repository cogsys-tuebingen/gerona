#ifndef STEERMISSION_H
#define STEERMISSION_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <ConfigFileReader.h>
#include "RobotMission.h"
#include "CalibBotInterface.h"

// Project

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class SteerMission : public RobotMission {
public:

    SteerMission( ConfigFileReader *config, CalibBotInterface *proxy );
    virtual ~SteerMission() {};

    /* Inherited from CalibrationMission */
    bool execute();

private:
    // Member
    CalibBotInterface *mProxy;

    // Config parameters
    double mForwardSpeed;
    double mReverseSpeed;
    double mSteerMin;
    double mSteerMax;
    double mSteerStep;
    double mArcLength;
    double mWheelbase;
};

#endif // STEERMISSION_H
