/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/15/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#ifndef TRAJECTORYMISSION_H
#define TRAJECTORYMISSION_H

#include <fstream>
#include "Global.h"
#include "RobotMission.h"
class CalibBotInterface;
class ConfigFileReader;
class RobotAction;

/**
  this mission lets the robot execute a number of configurable simple actions like
  drive ahead for a given distance or time, and compares the resulting odometry
  and localization positions

  Configuration is done with a configfile as follows:
  <configkey>::action01::type = {"moveDist","moveTime"}
  <configkey>::action01::<actionKey1> = <val1>
  <configkey>::action01::<actionKey2> = <val2>
  <configkey>::action02::<actionKey1> = <val3>
  ...
  <configkey>::resultFile ="<path/to/file>"

  */
class TrajectoryMission : public RobotMission
{
public:
    /**
        ctor

        parses config , creates subordinate actions
      */
    TrajectoryMission(ConfigFileReader *config, const string& config_key, CalibBotInterface *proxy);
    virtual ~TrajectoryMission ();

    /**

      */
    virtual bool execute ();

protected:
    void configure (ConfigFileReader *config, const string& config_key);

private:
    list<RobotAction*> action_list_;
    CalibBotInterface *proxy_;
    ofstream  log_stream_;
    string    result_fname_;
};

#endif // TRAJECTORYMISSION_H
