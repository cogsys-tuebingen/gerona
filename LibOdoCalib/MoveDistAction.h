/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/16/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#ifndef MOVEDISTACTION_H
#define MOVEDISTACTION_H
#include "MoveAction.h"
#include "SteerAngleController.h"

class ConfigFileReader;
/**
  action which moves robot until a minimum distance is reached
  */
class MoveDistAction : public MoveAction
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
      ctor

      creates and configures this action
      */
    MoveDistAction(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);

    virtual ~MoveDistAction ();
    /**
      @return true if successfull reached distance, false otherwise
      */
    virtual bool execute ();

    virtual void getResults(const string &key, ConfigFileReader &results);
private:
    /// distance to drive
    double  target_dist_;


};

#endif // MOVEDISTACTION_H
