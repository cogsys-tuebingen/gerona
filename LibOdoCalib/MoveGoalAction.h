#ifndef MOVEGOALACTION_H
#define MOVEGOALACTION_H

class MoveGoalAction : public MoveAction
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MoveGoalAction(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);

    virtual bool execute () ;
    virtual void getResults (const string &key, ConfigFileReader &results);
protected:
    /// speed in msec
    double speed_ms_;


};

};

#endif // MOVEGOALACTION_H
