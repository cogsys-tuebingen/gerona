#ifndef ENCODERMISSION_H
#define ENCODERMISSION_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <ConfigFileReader.h>

// Project
#include "StraightLineDriver.h"
#include "EncoderEstimator.h"
#include "RobotMission.h"
#include "CalibBotInterface.h"
///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class EncoderMission : public RobotMission {
public:

    /**
     * Constructor.
     *
     * @param config Coniguration file.
     * @param proxy Communicates with the robot.
     */
    EncoderMission( ConfigFileReader *config, CalibBotInterface *proxy );

    /* Inherited from CalibrationMission */
    bool execute();


private:
    /** CalibBotInterface with the robot. */
    CalibBotInterface *mProxy;
    /** Used to drive a straight line */
    StraightLineDriver mLineDriver;
    /** Computes the encoder calibration */
    EncoderEstimator mEncEst;
    /** Number of calibration runs */
    int mNumRuns;
    /** Minimum distance */
    double mMinDist;
    /** Mission speed [m/s] */
    double mSpeed;
};

#endif // ENCODERMISSION_H
