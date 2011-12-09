#include "DriveStraightMission.h"
#include "DriveStraightAction.h"
#include "LowPassFilter.h"
#include "Misc.h"
DriveStraightMission::DriveStraightMission(ConfigFileReader *config, const string& configKey, CalibBotInterface *proxy)
    :proxy_(proxy)

{
    forward_driver_=new DriveStraightAction(config,configKey+"::Forward",proxy);
    backward_driver_=new DriveStraightAction(config,configKey+"::Backward",proxy);
    planner_segment_num_ = config->GetInt(configKey+"::segmentNum",60);
    beta_target_ = config->GetDouble(configKey+"::betaTarget",-9999)*M_PI/180.0;
    mode_start_ = config->GetInt(configKey+"::startDirection",POS_FRONT);
}



bool DriveStraightMission::driveFwdBwd(double course,double beta_target,const string& key_base,
                                        ConfigFileReader& results)
{
    char buffer[64];
    int calib_run_num = results.GetInt(key_base+"::lastRun",-1)+1;
    cout << "coursefwdbwd="<<course << endl;
    forward_driver_->setBetaTarget(beta_target);
    backward_driver_->setCourse(course);
    forward_driver_->setCourse(course);
    backward_driver_->setBetaTarget(beta_target);
    DriveStraightAction *cur_driver = forward_driver_;
    int mode = POS_FRONT;
    for (int cnt=0;cnt<1;++cnt) {
        // execute staright drive
        bool finished = cur_driver->execute();
        if (!finished) {return false;}

        // get and save results
        snprintf(buffer,63,"%02d",calib_run_num+cnt);
        string key=key_base+buffer;
        cur_driver->getResults(key,results);
        results.SetInt(key_base+"::lastRun",calib_run_num+cnt);

        // prepare next run
        float front_mean,front_std,rear_mean,rear_std;
        cur_driver->getServoStats(front_mean,front_std,rear_mean,rear_std);
        if (!(front_mean>1300 && front_mean<3200 && rear_mean>1300 && rear_mean<3200)) {
            cout << "means out of range " <<"front="<<front_mean<<" rear="<<rear_mean<< endl;
            return false;
        }
        if (mode==POS_FRONT) {
            mode=POS_REAR;
            cur_driver= backward_driver_;
            cout << "switching backward"<< endl;
        } else {
            mode = POS_FRONT;
            cur_driver = forward_driver_;
        }

        cur_driver->setServoStart((int)(front_mean+0.5),(int)(rear_mean+0.5));
        cur_driver->setServoMids((int)(front_mean+0.5),(int)(rear_mean+0.5));
        Vector3d slamPose;
        proxy_->GetSlamPose(slamPose);
    //    cur_driver->setCourse(slamPose(2));
    }
    int run=results.GetInt(key_base+"::lastRun",-1);
    cout <<" rune="<<run << endl;
    results.Save(key_base+"Results.conf");
    return true;
}



bool DriveStraightMission::execute()
{
    // load results and find current num
    ConfigFileReader results;
    string key_base="straight";
    results.Load(key_base+"Results.conf");

    log_stream_.open("straightAction.log");
    log_stream_ << "%beta target ="<<beta_target_ << endl;

    mission_timer_.restart();
    forward_driver_->setLog(&log_stream_,&mission_timer_);
    backward_driver_->setLog(&log_stream_,&mission_timer_);
    Vector3d slamPose,newPose;
    proxy_->Read();
    while (!proxy_->IsFreshSlamPose()) {
        cout << "waiting for proxies"<<endl;
        usleep(10000);
        proxy_->Read();
    }
    proxy_->GetSlamPose(slamPose);
    double bestCourse = slamPose(2);
    cout << "bestcourse="<<bestCourse << " x="<< slamPose(1)<< endl;
    double beta_target = beta_target_;
    proxy_->Read();
    proxy_->GetSlamPose(slamPose);

    float servo15_front, servo15_rear,std_front, std_rear;
    float zero_servo_front, zero_servo_rear;


    bool success=driveFwdBwd(bestCourse,beta_target,key_base,results);
    forward_driver_->getServoStats(servo15_front,std_front,servo15_rear,std_rear);
    //proxy_->SetSpeedAndServos(0,servo15_front,servo15_rear);

    if (!success) return false;



    return true;
}

