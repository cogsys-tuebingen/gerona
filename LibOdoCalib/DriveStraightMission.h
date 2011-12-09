#ifndef DRIVESTRAIGHTMISSION_H
#define DRIVESTRAIGHTMISSION_H

#include <iostream>
#include <fstream>
// Project
#include "LaserEnvironment.h"
#include "DriveStraightAction.h"
#include "ConfigFileReader.h"
#include "RobotMission.h"
#include "StatsEstimator.h"

class DriveStraightMission : public RobotMission
{
public:
    /**
     * Constructor.
     *
     * @param config Configuration file.
     * @param proxy Communicates with the robot.
     */
    DriveStraightMission( ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy );


    bool execute();
private:
    DriveStraightAction *forward_driver_,*backward_driver_;
    bool driveFwdBwd (double course,double beta_target,const string &key_base, ConfigFileReader& result);
    CalibBotInterface   *proxy_;
    ofstream            log_stream_;
    Stopwatch           mission_timer_;
    int                 planner_segment_num_;
    double              beta_target_;
    int                 mode_start_;
};

#endif // DRIVESTRAIGHTMISSION_H
