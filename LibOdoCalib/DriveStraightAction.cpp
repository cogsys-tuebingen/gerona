#include <Eigen/Dense>
#include "RobotAction.h"
#include "Calibration.h"
#include "Stopwatch.h"
#include "ConfigFileReader.h"
#include "CalibBotInterface.h"
#include "LaserEnvironment.h"
#include "StatsEstimator.h"
#include "BetaEstimator.h"
#include "DriveStraightAction.h"
#include "Misc.h"
#include "LowPassFilter.h"


DriveStraightAction::DriveStraightAction(ConfigFileReader *config, const string& config_key,CalibBotInterface *proxy)
    :RobotAction(proxy)
{
    // configure the controller
    ctrl_sleep_ = config->GetInt(config_key+"::ctrlSleep",500);
    ctrl_cnt_ = config->GetInt(config_key+"::ctrlCnt",1);
    tuning_a_ = config->GetDouble( config_key+"::tuningA", 1);
    double ki_yaw = config->GetDouble(config_key+"::kiYaw",0);
    double ks11 = config->GetDouble(config_key+"::ks11",1);
    double ks12 = config->GetDouble(config_key+"::ks12",0.25);
    double ks21 = config->GetDouble(config_key+"::ks21",1);
    double ks22 = config->GetDouble(config_key+"::ks22",-0.175);
    double kp_yaw = config->GetDouble(config_key+"::kpYaw",1);    
    Matrix2d ksinv;
    ksinv << ks11,ks12,ks21,ks22;
    dual_axis_calib_.SetKSinv(ksinv);
    double servo_factor = config->GetDouble(config_key+"::servoFactor",(3000.0-2250.0)/(20.0*M_PI/180.0));
    dual_axis_calib_.SetServoDelta(POS_FRONT,servo_factor);
    dual_axis_calib_.SetServoDelta(POS_REAR,servo_factor);
    dual_axis_calib_.SetKIyaw(ki_yaw);
    dual_axis_calib_.SetKPyaw(kp_yaw);
    direction_ = config->GetInt(config_key+"::direction",POS_FRONT);
    if (direction_==POS_FRONT)
        dual_axis_calib_.CalcController(1.0,tuning_a_);
    else
        dual_axis_calib_.CalcController(-1.0,tuning_a_);



    min_delta_dist_ = config->GetDouble(config_key+"::minDelta",0.03);
    servo_front_fixed_ = config->GetInt(config_key+"::servoFront",-9999);
    servo_rear_fixed_ = config->GetInt(config_key+"::servoRear",-9999);
    servo_front_mid_ = config->GetInt(config_key+"::servoFrontMid",2126);
    servo_rear_mid_ = config->GetInt(config_key+"::servoRearMid",2149);
    servo_front_start_ = config->GetInt(config_key+"::servoFrontStart",2250);
    servo_rear_start_ = config->GetInt(config_key+"::servoRearStart",2250);

    setServoMids(servo_front_mid_,servo_rear_mid_);


    beta_interval_ = config->GetDouble(config_key+"::betaInterval",10.0)*M_PI/180.0;
    interval_threshold_=config->GetInt(config_key+"::interval_threshold",10);
    max_drive_distance_ = config->GetDouble(config_key+"::maxDriveDistance",3.0);
    min_drive_distance_ = config->GetDouble(config_key+"::minDriveDistance",2.0);
    sleep_time_ms_ = config->GetInt(config_key+"::sleepTimeMs",20);
    cout << "mindeltadist="<<min_delta_dist_<< endl;
    //mNoiseFront = config->GetInt(configKey+"::noiseFront",0);
    //mNoiseRear = config->GetInt(configKey+"::noiseRear",0);
    cout << "configuring key="<<config_key<< " direction="<<direction_<<endl;
    speed_ = config->GetDouble(config_key+"::speed",0.5);
    speed_zero_ = 0.0;
    noise_front_ = 0;
    noise_rear_ = 0;
    course_ = 0.0;
    beta_target_ = 0.0;
    log_stream_ = 0;
    mission_timer_ = 0;

}


DriveStraightAction::~DriveStraightAction ()
{
}


void DriveStraightAction::setCourse(double yaw)
{
    course_ = yaw;
}


void DriveStraightAction::setSpeed (float speed)
{
    speed_ = speed;
}





void    DriveStraightAction::setServoMids (int front_mid, int rear_mid)
{
    dual_axis_calib_.SetServoMid(POS_FRONT,front_mid);
    dual_axis_calib_.SetServoMid(POS_REAR,rear_mid);
}


bool DriveStraightAction::execute()
{

  //  dual_axis_calib_.CalcController(speed_,tuning_a_);


    Vector3d slamPose,newPose;
    BetaEstimator beta_estimator(min_delta_dist_,0.01,0.6);
    proxy_->Read(); // Read incoming messages

    Stopwatch timer;
    timer.restart();
//    int n = 0;



    LowPassFilter<double> beta_filter(3);
    LowPassFilter<double> last_betas(interval_threshold_);
    LowPassFilter<double> yawrate_filter(3);
    double yaw=0.0, yaw_rate=0.0;
    LowPassFilter<float> meanServoF(5);
    LowPassFilter<float> meanServoR(5);
    controlled_count_ = 0;
    int cnt = 0;
    dual_axis_calib_.Reset();
    servo_front_stats_.reset();
    servo_rear_stats_.reset();
    hall_stats_.resize(4);
    for (int h=0;h<4;++h) {
        hall_stats_[h].reset();
    }
    beta_err_stats_.reset();
    yaw_err_stats_.reset();
    yawrate_err_stats_.reset();
    double beta= beta_target_;    
    float servoF=servo_front_start_ , servoR=servo_rear_start_;
    if (servo_front_fixed_>0 && servo_rear_fixed_>0) {
        servoF=servo_front_fixed_;servoR=servo_rear_fixed_;
    }
    cout << "start moving servo front="<<servoF<< " servo rear="<<servoR<<endl;
    // start moving
    Vector3d prevPose,delta;
    proxy_->GetSlamPose(prevPose);
    startMoving(speed_,servoF,servoR,0.3,6000);
    Vector3d startPose;
    proxy_->GetSlamPose(startPose);

    course_ =startPose(2);

    beta = calcBetaAngle(prevPose,startPose,direction_);
    cout << "beta after move start corrected" << beta << endl;
    prevPose =slamPose;
    *log_stream_<< "%minDelta = "<<min_delta_dist_<< endl;
    *log_stream_<< "%tuninga = "<<tuning_a_<< endl;
     *log_stream_<< "%course = "<<course_<< endl;
     *log_stream_<< "%betaset = "<<beta_target_<< endl;



    if ((beta>30.0*M_PI/180.0)||((beta<-30.0*M_PI/180.0))) {
        beta = beta_target_;
    }
    Stopwatch ctrl_timer;
    ctrl_timer.restart();
    bool controlStarted=false;
    while (true) {
        // Read incoming messages
        cout << "proxy read"<<endl;
        proxy_->Read();
        // check collision and react accordingly
        double threshold = 0.6;
        bool isColliding = checkCollision(beta,threshold);
        if (isColliding) {
            proxy_->SetSpeedAndServos(speed_zero_,servoF,servoR);

            proxy_->GetSlamPose( slamPose );
            delta=slamPose-startPose;
            cout << "Stopping driven dist="<<delta.head<2>().norm()<<endl;
            if (delta.head<2>().norm()>min_drive_distance_) {
                return true;
            } else {
                return false;
            }
        }

        // get new slam pose
        if ( !proxy_->IsFreshSlamPose()) {
            usleep( 10000 ); // Wait 10 msec
            continue;
        }      
        ++cnt;
        proxy_->GetSlamPose( newPose );
        beta_estimator.update(newPose);
        delta=newPose-prevPose;

        double ls_beta=0.0;
        // calc new beta if sensible
        double betaTemp = beta;
        if (delta.head<2>().norm()>min_delta_dist_) {
            betaTemp = calcBetaAngle(prevPose,newPose,direction_);
            prevPose=slamPose;
            if (isnan(betaTemp)) {
                cout << "old beta isnan prevPose="<<prevPose<< " newpose="<<newPose<< endl;
                betaTemp=beta;
            }
            if ((betaTemp>30.0*M_PI/180.0)||((betaTemp<-30.0*M_PI/180.0))) {
                // impossible beta value
                betaTemp = beta;
            }
        }
        bool beta_valid=beta_estimator.calcLsBetaAngle(1,direction_,ls_beta);
        if (beta_valid) {
            if (isnan(ls_beta)) cout << "valid butcalcls failed"<<endl;
            beta=ls_beta;
        } else {
            beta = betaTemp;

        }
        last_betas.Update(beta);
        beta=beta_filter.Update(beta);

        // calc yaw rate and pniyaw
        int ms = timer.msElapsed();
        timer.restart();
        yaw = newPose(2);
        double delta_yaw=Misc::normalizeAngle(yaw-slamPose(2));

        yaw_rate=1000.0*delta_yaw/ms;
        yawrate_filter.Update(yaw_rate);
        double pniYaw = proxy_->GetPniYaw();
        if (isnan(beta)) {
            cout << "arrrg"<<endl;
        }
        // execute controller
        if (cnt>=ctrl_cnt_) {
            dual_axis_calib_.Execute(ms/1000.0,beta_target_,beta,course_,yaw,servoF,servoR);
            cnt=0;
        }

        // set servos
        if (servo_front_fixed_>0) servoF=servo_front_fixed_;
        if (servo_rear_fixed_>0) servoR=servo_rear_fixed_;
        proxy_->SetSpeedAndServos(speed_,servoF,servoR);

        // update stats
        meanServoF.Update(servoF);
        meanServoR.Update(servoR);
        if (controlStarted) {
            double beta_err=fabs(Misc::normalizeAngle(beta-beta_target_));
            beta_err_stats_.update(beta_err);
            yawrate_err_stats_.update(fabs(yaw_rate));
            int ticks_left=proxy_->GetLeftEncoderTicks();
            int ticks_right=proxy_->GetRightEncoderTicks();
            encoder_estimator_.Update(newPose,ticks_left,ticks_right);

        }
        DVector hall_volts;
        hall_volts.resize(4);
        proxy_->GetHallVoltages(hall_volts);
        cout << "betatraget2 "<<beta_target_ << " beta:"<<beta << "last_betas" <<last_betas.GetMean()<< endl;
        bool isControlled = false;
        if (mission_timer_->msElapsed()>2000) {
            isControlled = last_betas.IsInInterval(beta_target_-beta_interval_,beta_target_+beta_interval_);
        }
        if (isControlled) {
            cout << "iscontrolled2"<<endl;
            ++controlled_count_;
            if (!controlStarted) {
                int ticks_left=proxy_->GetLeftEncoderTicks();
                int ticks_right=proxy_->GetRightEncoderTicks();
                encoder_estimator_.SetStart(prevPose,ticks_left,ticks_right);
            }
            controlStarted = true;
            beta_estimator.setMinDist(min_delta_dist_*2.0);
            servo_front_stats_.update(servoF);
            servo_rear_stats_.update(servoR);
            for (int h=0;h<4;++h) {
                hall_stats_[h].update(hall_volts[h]);
            }
            if (fabs(yawrate_filter.GetValue())<4.0*M_PI/180.0) {
    //            if (direction_==MODE_FORWARD) {
     //               cout <<" switching to tuninga "<<tuning_a_-0.1 << " speed="<<speed_<<endl;
       //           dual_axis_calib_.CalcController(speed_,tuning_a_-0.1);
      //          }
            }
        }
        cout << "loggingdata"<<endl;
        logData(slamPose,newPose,course_, pniYaw,beta_target_, beta,servoF,servoR,isControlled,yaw_rate,ls_beta, hall_volts);
        slamPose = newPose;
        delta=slamPose-startPose;
        if (delta.head<2>().norm()>max_drive_distance_) {
            cout <<"max drive dist reached"<<endl;
            proxy_->SetSpeedAndServos(speed_zero_,servoF,servoR);
            break;
        }
        cout << "sleeping "<<sleep_time_ms_ << " msec"<<endl;
        usleep(sleep_time_ms_*1000);
    }

    return true;
}


void DriveStraightAction::logData(const Vector3d& p1, const Vector3d& p2, double course,
                                  double pniYaw, double betaSet, double beta, int servoF, int servoR, bool isControlled,
                                  double yawRate,double ls_beta, DVector& hall_volts)
{
    if (!log_stream_ || !mission_timer_) return;

    *log_stream_ << mission_timer_->msElapsed()<<" "<<p1.x()<<" "<<p1.y()<<" "<<p1.z() <<" "
                <<p2.x()<<" "<<p2.y()<<" "<<p2.z() <<" "
                << course << " "<<pniYaw << " "<<betaSet << " "<<beta <<" "
                <<servoF << " "<<servoR << " "<<speed_<<" "<<isControlled<<" "<<yawRate<<" "
                <<ls_beta ;
    for (int h=0;h<hall_volts.size();++h) {
        *log_stream_ << " "<<hall_volts[h];
    }
    *log_stream_<< endl;
}


void DriveStraightAction::getServoStats(float& front_mean, float& front_std, float& rear_mean,float& rear_std)
{
    front_mean=servo_front_stats_.getMean();
    front_std=servo_front_stats_.getStd();
    rear_mean=servo_rear_stats_.getMean();
    rear_std=servo_rear_stats_.getStd();

}


void DriveStraightAction::getOutputStats(StatsEstimator<double> &beta_err_stats, StatsEstimator<double> &yawrate_err_stats)
{
    beta_err_stats=beta_err_stats_;
    yawrate_err_stats=yawrate_err_stats_;
}


void DriveStraightAction::getResults (const string &key, ConfigFileReader &results)
{
    results.SetInt(key+"::direction",direction_);
    results.SetInt(key+"::ctrlSleep",ctrl_sleep_);
    results.SetInt(key+"::controlledCount",controlled_count_);
    results.SetDouble(key+"::betaTarget",beta_target_);
    results.SetDouble(key+"::frontMean",servo_front_stats_.getMean());
    results.SetDouble(key+"::frontStd",servo_front_stats_.getStd());
    results.SetDouble(key+"::rearMean",servo_rear_stats_.getMean());
    results.SetDouble(key+"::rearStd",servo_rear_stats_.getStd());
    results.SetDouble(key+"::betaErrMean",beta_err_stats_.getMean());
    results.SetDouble(key+"::betaErrStd",beta_err_stats_.getStd());
    results.SetDouble(key+"::yawMean",yaw_err_stats_.getMean());
    results.SetDouble(key+"::yawStd",yaw_err_stats_.getStd());
    results.SetDouble(key+"::yawRateErrMean",yawrate_err_stats_.getMean());
    results.SetDouble(key+"::yawRateErrStd",yawrate_err_stats_.getStd());
    results.SetDouble(key+"::yawTarget",course_);
    results.SetDouble(key+"::tuningA",tuning_a_);
    results.SetDouble(key+"::speed",speed_);
    for (int h=0;h<hall_stats_.size();++h) {
        char buffer[256];
        snprintf(buffer,255,"%s::hall%dMean",key.c_str(),h);
        results.SetDouble(buffer,hall_stats_[h].getMean());
        snprintf(buffer,255,"%s::hall%dStd",key.c_str(),h);
        results.SetDouble(buffer,hall_stats_[h].getStd());

    }
    results.SetDouble(key+"::encoderLeft",encoder_estimator_.GetCalibrationLeft());
    results.SetDouble(key+"::encoderRight",encoder_estimator_.GetCalibrationRight());

}
