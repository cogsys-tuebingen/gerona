/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/22/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#ifndef MOVETIMEACTION_H
#define MOVETIMEACTION_H


#include "MoveAction.h"
#include "SteerAngleController.h"

class ConfigFileReader;
/**
  action which moves robot for a given time
  */
class MoveTimeAction : public MoveAction
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
      ctor

      creates and configures this action
      */
    MoveTimeAction(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);

    virtual ~MoveTimeAction ();
    /**
      @return true if successful move for the configured time
      */
    virtual bool execute ();

    virtual void getResults(const string &key, ConfigFileReader &results);

private:
    /// time to move
    int  move_time_msec_;
    bool do_steer_ctrl_;


};


#endif // MOVETIMEACTION_H
