#ifndef DRIVECIRCLEMISSION_H
#define DRIVECIRCLEMISSION_H

#include <iostream>
#include <fstream>
// Project
#include "LaserEnvironment.h"
#include "DriveCircleAction.h"
#include "ConfigFileReader.h"
#include "RobotMission.h"
#include "CalibBotInterface.h"
#include "StatsEstimator.h"


struct ServoVal
{
    ServoVal () {front=rear = 2250;front_rad=rear_rad=0.0;}
    ServoVal(int f, int r, double b) {front=f;rear=r;front_rad=rear_rad=b;}
    int front;
    int rear;
    double front_rad,rear_rad;
};
typedef map<int,ServoVal> ServoValMap;

class DriveCircleMission : public RobotMission
{
public:
    /**
     * Constructor.
     *
     * @param config Configuration file.
     * @param proxy Communicates with the robot.
     */
    DriveCircleMission( ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy );


    bool execute();
private:
    bool driveCircle (double course,double beta_target,const string &key_base, ConfigFileReader& result);
    void buildAngleServoMap(ConfigFileReader& data,const string& key_base);
    void convertResultsToMatlab(ConfigFileReader& data,const string& key_base);
    void writeMatlabResultLine(ConfigFileReader& data,const string& key, const string& fname);
    DriveCircleAction  *forward_driver_,*backward_driver_;
    ConfigFileReader    angles_config_;
    CalibBotInterface   *proxy_;
    ofstream            log_stream_;
    Stopwatch           mission_timer_;
    int                 planner_segment_num_;
    ServoValMap         servo_vals_;
    int                 mode_start_;
    int                 drive_left_right_;
    int                 direction_;
    double              steer_front_rad_,steer_rear_rad_;
};

#endif // DRIVESTRAIGHTMISSION_H
