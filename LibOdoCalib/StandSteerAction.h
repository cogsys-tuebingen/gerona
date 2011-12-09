#ifndef STANDSTEERACTION_H
#define STANDSTEERACTION_H
#include "MoveAction.h"
class StandSteerAction : public MoveAction
{
public:
    /**
      ctor

      creates and configures this action
      */
    StandSteerAction(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);

    virtual ~StandSteerAction ();

    /**
      @return true if successful move for the configured time
      */
    virtual bool execute ();

    virtual void getResults(const string &key, ConfigFileReader &results);

private:
    /// time to move
    int  steer_time_msec_;


};

#endif // STANDSTEERACTION_H
