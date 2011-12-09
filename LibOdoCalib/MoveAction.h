/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/24/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#ifndef MOVEACTION_H
#define MOVEACTION_H
#include "RobotAction.h"
#include "SteerAngleController.h"

class MoveAction : public RobotAction
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MoveAction(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);

    virtual bool execute () = 0;
    virtual void getResults (const string &key, ConfigFileReader &results);
protected:
    /// speed in msec
    double speed_ms_;
    /// steer angles front/ rear
    double  steer_rad_front_, steer_rad_rear_;

    Vector3d slam_pose_,odo_pose_;

    ClientSteerAngleController ctrl_front_,ctrl_rear_;

};

#endif // MOVEACTION_H
