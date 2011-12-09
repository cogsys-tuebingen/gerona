#ifndef DRIVECIRCLEACTION_H
#define DRIVECIRCLEACTION_H
#include <fstream>
#include <iostream>
#include "Eigen/Core"
using namespace Eigen;

#include "Global.h"
#include "CircleEstimator.h"
#include "StatsEstimator.h"
#include "Stopwatch.h"
#include "RobotAction.h"
#include "SteerAngleController.h"

class ConfigFileReader;

class LaserEnvironment;

class DriveCircleAction: public RobotAction
{
public:
    DriveCircleAction (ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy);
    ~DriveCircleAction ();
   virtual  bool    execute ();

//    void setParam(const string& key, constr string& value);


    void    setSpeed (double speed);

    void    setDirection (int dir) {direction_=dir;}

    /**
      set how many circles the robot should drive
      @param turns can be uneven
      */
    void    setTurnsCnt (double turns) {turns_=turns;}

    void    setServos(int servo_front, int servo_rear, double rad_front, double rad_rear);

    void    setSteerAngles (double rad_front, double rad_rear);

    /**
      write results of drive with base key in results
      */
    virtual void    getResults (const string& key,ConfigFileReader& results);
private:
    string              config_key_;
    double   calcLaserDistance();





    ClientSteerAngleController ctrl_front_, ctrl_rear_;

    double              speed_ms_;
    double              steer_rad_front_,steer_rad_rear_;
    int                 servo_front_,servo_rear_;
    double              err_points_thresh_;
    int                 direction_;
    double              turns_;
    CircleEstimator     circle_estimator_;
    double              min_delta_dist_;
    double              fullcircle_tolerance_;
    double              circle_err_tolerance_;
    StatsEstimator<double> wheelbase_stats_;
    StatsEstimator<double> beta_stats_;
    StatsEstimator<double> yawrate_stats_;
    StatsEstimator<double> laserdist_stats_;
    StatsEstimator<double> radius_stats;
    StatsEstimator<float> hall1_front_stats_,hall2_front_stats_;
    StatsEstimator<double> hall_front_angle_stats_,hall_rear_angle_stats_;


};

#endif // DriveCircleAction_H
