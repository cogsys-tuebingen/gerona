#ifndef DRIVESTRAIGHTACTION_H
#define DRIVESTRAIGHTACTION_H
#include <fstream>
#include <iostream>
#include "Eigen/Core"
using namespace Eigen;

#include "Global.h"
#include "DualAxisCalib.h"
#include "Stopwatch.h"
#include "RobotAction.h"
#include "Calibration.h"
#include "StatsEstimator.h"
#include "EncoderEstimator.h"

class ConfigFileReader;
class CalibBotInterface;
class LaserEnvironment;

class DriveStraightAction : public RobotAction
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DriveStraightAction (ConfigFileReader *config, const string& configKey,CalibBotInterface *proxy);
    ~DriveStraightAction ();
    virtual bool    execute ();

//    void setParam(const string& key, constr string& value);

    void    setCourse ( double yaw);

    void    setSpeed (float speed);

    void    setBetaTarget (double beta) {beta_target_=beta;}

    void    setServoStart (int front, int rear) {servo_front_mid_=front;servo_rear_mid_=rear;}

    void    setServoMids (int front_mid, int rear_mid);

    void    getServoStats(float& front_mean, float& front_std, float& rear_mean,float& rear_std);

    void    getOutputStats(StatsEstimator<double>& beta_err_stats, StatsEstimator<double>& yawrate_err_stats);

    /**
      write results of drive with base key in results
      */
    virtual void    getResults (const string& key,ConfigFileReader& results);

    void    logData (const Vector3d& p1, const Vector3d& p2, double course,
                     double pniYaw, double betaSet, double beta, int servoF, int servoR, bool isCtrl,
                     double yawRate, double ls_beta, DVector& hall_volts);
    ICtrl       dual_axis_calib_;
    double              beta_target_;
    float               speed_, speed_zero_;
    LaserEnvironment    laser_env_;
    double              min_delta_dist_;
    double              course_;
    int                 noise_front_, noise_rear_;
    double              tuning_a_;
    int                 servo_front_fixed_,servo_rear_fixed_;
    int                 servo_front_mid_,servo_rear_mid_;
    int                 servo_front_start_,servo_rear_start_;
    StatsEstimator<float> servo_front_stats_,servo_rear_stats_;
    vector<StatsEstimator<float> > hall_stats_;
    StatsEstimator<double> beta_err_stats_,yaw_err_stats_, yawrate_err_stats_;
    float_t              beta_interval_;
    int                 interval_threshold_;
    double              max_drive_distance_,min_drive_distance_;
    int                 sleep_time_ms_;
    int                 ctrl_sleep_;
    int                 ctrl_cnt_;
    int                 controlled_count_;
    EncoderEstimator    encoder_estimator_;


};

#endif // DRIVESTRAIGHTACTION_H
