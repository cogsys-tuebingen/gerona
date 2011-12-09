#include "ConfigFileReader.h"
#include "DriveCircleMission.h"
#include "DriveCircleAction.h"
#include "LowPassFilter.h"
#include "Calibration.h"
#include "Misc.h"
DriveCircleMission::DriveCircleMission(ConfigFileReader *config, const string& config_key, CalibBotInterface *proxy)
    :proxy_(proxy)

{
    forward_driver_=new DriveCircleAction(config,config_key+"::Forward",proxy);
    backward_driver_=new DriveCircleAction(config,config_key+"::Backward",proxy);
    planner_segment_num_ = config->GetInt(config_key+"::segmentNum",60);
    string fname = config->GetString(config_key+"::straightResults","");
    steer_front_rad_=config->GetDouble(config_key+"::steerFrontDeg",18.0)*M_PI/180.0;
    steer_rear_rad_=config->GetDouble(config_key+"::steerRearDeg",0)*M_PI/180.0;
    direction_ = config->GetInt(config_key+"::direction",0);
    drive_left_right_ = config->GetInt(config_key+"::driveLeftRight",CALIB_LEFT);
}



void DriveCircleMission::writeMatlabResultLine(ConfigFileReader &results, const string &key, const string& fname)
{
    bool write_header = !Misc::fileExists(fname.c_str());
    ofstream out;
    out.open(fname.c_str(),ios_base::app);
    if (write_header) {
        out << "%servofront servorear steerFrontPred steerRearPred yawrate beta wheelbase wheelbasestd laserdist laserdiststd hall1mean hall1std hall2mean hall2std hall3mean hall3std hall4mean hall4std"<<endl;

    }
    int servo_front=results.GetInt(key+"::servoFront",2250);
    int servo_rear=results.GetInt(key+"::servoRear",2250);
    double steer_rad_front =results.GetDouble(key+"::steerFrontPred",0.0);
    double steer_rad_rear=results.GetDouble(key+"::steerRearPred",0.0);
    double yaw_rate=results.GetDouble(key+"::yawRate",0.0);
    double beta=results.GetDouble(key+"::beta",0.0);

    double wheelbase=results.GetDouble(key+"::wheelbase",0.0);
    double wheelbase_std=results.GetDouble(key+"::wheelbaseStd",0.0);
    double laser_dist=results.GetDouble(key+"::laserDist",0.0);
    double laser_dist_std=results.GetDouble(key+"::laserDistStd",0.0);

    out << servo_front << " "<<servo_rear << " "<<steer_rad_front<< " "<<steer_rad_rear<<" "
           << yaw_rate<< " "<< beta << " "<<wheelbase<<" "<<wheelbase_std<< " "
              << laser_dist << " "<<laser_dist_std<<" ";
    for (int h=0;h<4;++h) {
        char buffer[256];
        snprintf(buffer,255,"%s::hall%dMean",key.c_str(),h);
        double hall_mean=results.GetDouble(buffer,0.0);
        snprintf(buffer,255,"%s::hall%dStd",key.c_str(),h);
        double hall_std =results.GetDouble(buffer,0.0);
        cout << "keystd:"<<buffer << " hall mean"<<hall_mean<< " hallstd"<<hall_std<<endl;
        out << hall_mean <<" "<<hall_std << " ";
    }
    out << endl;

    out.close();

}


void DriveCircleMission::convertResultsToMatlab(ConfigFileReader &data, const string &key_base)
{
    ofstream out;
    string out_fname(key_base);
    out_fname.append("_results_mat.log");
    out.open(out_fname.c_str());

    out <<
 "% runNo radTarget front_mean rear_mean front_std rear_std ctrl_count rad_err_mean yaw_rate_err_mean calib_left calib_right hall1_mean hall1_std hall2_mean hall2_std ..."<<endl;
    int last_run = data.GetInt(key_base+"::lastRun",-1);
    if (last_run<0) {
        cout << "no angle data in input config"<<endl;
        return;
    }
    char buffer[64];
    for (int i=0;i<=last_run;++i) {
        snprintf(buffer,63,"%02d",i);
        string key=key_base+buffer;
        int direction = data.GetInt(key+"::direction",-1);
        double servo_rad=data.GetDouble(key+"::betaTarget",-9999.0);
        double front_mean=data.GetDouble(key+"::frontMean",2250.0);
        double front_std=data.GetDouble(key+"::frontStd",2250.0);
        double rear_mean=data.GetDouble(key+"::rearMean",2250.0);
        double rear_std=data.GetDouble(key+"::rearStd",2250.0);
        int ctrl_cnt =data.GetInt(key+"::controlledCount",0);
        double beta_err_mean=data.GetDouble(key+"::betaErrMean",0.0);
        double yaw_rate_err_mean=data.GetDouble(key+"::yawRateErrMean",0.0);
        double calib_left = data.GetDouble(key+"::encoderLeft",0.0);
        double calib_right = data.GetDouble(key+"::encoderRight",0.0);
        out << i<<" "<<servo_rad<<" "<<front_mean<<" "<<rear_mean<<" "<<front_std<<" "
               <<rear_std<< " "<<ctrl_cnt<<" "<<beta_err_mean<< " "<< yaw_rate_err_mean
               <<" "<<calib_left <<" "<<calib_right<<" ";
        for (int h=0;h<4;++h) {
            char buffer[256];
            snprintf(buffer,255,"%s::hall%dMean",key.c_str(),h);
            double hall_mean=data.GetDouble(buffer,0.0);
            snprintf(buffer,255,"%s::hall%dStd",key.c_str(),h);
            double hall_std =data.GetDouble(buffer,0.0);
            cout << "keystd:"<<buffer << " hall mean"<<hall_mean<< " hallstd"<<hall_std<<endl;
            out << hall_mean <<" "<<hall_std << " ";
        }
        out << endl;
    }
    out.close();
}


void DriveCircleMission::buildAngleServoMap(ConfigFileReader& data,const string& key_base)
{
    int last_run = data.GetInt(key_base+"::lastRun",-1);
    if (last_run<0) {
        cout << "no angle data in input config"<<endl;
        return;
    }
    char buffer[64];
    bool found_zero = false;

    map<int,StatsEstimator<double> > front_servos, rear_servos;


    for (int i=0;i<=last_run;++i) {
        snprintf(buffer,63,"%02d",i);
        string key=key_base+buffer;
        int direction = data.GetInt(key+"::direction",-1);
        if (direction<0 || direction!=direction_) {
            // missing entry or wrong direction
            continue;
        }
        double servo_rad=data.GetDouble(key+"::betaTarget",-9999.0);
        if (servo_rad<-9000.0) {
            cout << "error in configfile key="<<key<<endl;
            return;
        }
        int angle_deg = round(servo_rad*180.0/M_PI);
        front_servos[angle_deg].update(data.GetDouble(key+"::frontMean",2250));
        rear_servos[angle_deg].update(data.GetDouble(key+"::rearMean",2250));
        servo_vals_[angle_deg].front_rad = servo_rad;
        servo_vals_[angle_deg].rear_rad = servo_rad;
        cout << "found angle deg="<<angle_deg<<endl;
        if (angle_deg==0) found_zero=true;
    }

    if (!found_zero) {
        cout << "no values for zero steering angle"<<endl;
    }
    for (ServoValMap::iterator it=servo_vals_.begin();it!=servo_vals_.end();++it) {
        int angle_deg=it->first;
        it->second.front = round(front_servos[angle_deg].getMean());
        it->second.rear = round(rear_servos[angle_deg].getMean());
        cout << "angle="<<angle_deg << " front="<<it->second.front << " rear="<<it->second.rear << endl;
    }

    /*
        int front = round(data.GetDouble(key+"::frontMean",2250));
        int rear = round(data.GetDouble(key+"::rearMean",2250));
        servo_vals_[angle_deg].front = front;
        servo_vals_[angle_deg].front_rad = servo_rad;
        servo_vals_[angle_deg].rear = rear;
        servo_vals_[angle_deg].rear_rad = servo_rad;
      */

}


bool DriveCircleMission::execute()
{
    // load results and find current num
    ConfigFileReader results;
    string key_base="circle";
    results.Load(key_base+"Results.conf");
    int calib_run_num = results.GetInt(key_base+"::lastRun",-1)+1;

    // prepare log
    log_stream_.open("circleAction.log");
    string mat_fname="circle_mat_results.log";
    mission_timer_.restart();
    forward_driver_->setLog(&log_stream_,&mission_timer_);
    forward_driver_->setDirection(POS_FRONT);
    backward_driver_->setLog(&log_stream_,&mission_timer_);
    backward_driver_->setDirection(POS_REAR);
    Vector3d slamPose,newPose;
    proxy_->Read();
    while (!proxy_->IsFreshSlamPose()) {
        cout << "waiting for proxies"<<endl;
        usleep(10000);
        proxy_->Read();
    }
    proxy_->GetSlamPose(slamPose);
    int cnt = 0;
    char buffer[64];
    forward_driver_->setSteerAngles(steer_front_rad_,steer_rear_rad_);
    bool success = forward_driver_->execute();
    if (success) {
        cout << "writing circle results"<< endl;
        snprintf(buffer,63,"%02d",calib_run_num+cnt);
        string key=key_base+buffer;
        forward_driver_->getResults(key,results);
        results.SetInt(key+"::lastRun",calib_run_num+cnt);
        results.Save(key_base+"Results.conf");
        writeMatlabResultLine(results,key,mat_fname);
      }
/*
    if (servo_vals_.find(0)==servo_vals_.end()) {
        cout << "missing zero steering angles"<<endl;
        return false;
    }
    ServoVal zs = servo_vals_[0];
    // drive the list of given steering angles
    int cnt = 0;
    char buffer[64];
    for (IList::iterator steer_it=steer_degs_.begin();steer_it!=steer_degs_.end();++steer_it) {
        int steer_deg=*steer_it;
        cout << "steer deg ="<<steer_deg << endl;
        // drive with front wheel steering

        //forward_driver_->setServos(s.front,zs.rear,s.front_rad,zs.rear_rad);
        //forward_driver_->setSteerAngles(steer_deg*M_PI/180.0,zs.rear_rad);
        forward_driver_->setSteerAngles(steer_deg*M_PI/180.0,0.0);
        bool success = forward_driver_->execute();
        if (success) {
            snprintf(buffer,63,"%02d",calib_run_num+cnt);
            string key=key_base+buffer;
            forward_driver_->getResults(key,results);
            results.SetInt(key+"::lastRun",calib_run_num+cnt);
            results.Save(key_base+"Results.conf");
            writeMatlabResultLine(results,key,mat_fname);
            ++cnt;
        }*/
        /*
        if (servo_vals_.find((-1*steer_deg))!=servo_vals_.end()) {
            ServoVal s=servo_vals_[(-1*steer_deg)];
            // drive with rear wheel steering
            cout << "rear steering front="<<zs.front << " rear="<<s.rear<< endl;
            forward_driver_->setServos(zs.front,s.rear,zs.front_rad,s.rear_rad);
            bool success = forward_driver_->execute();
            if (success) {
                forward_driver_->getResults(key_base,results);
                snprintf(buffer,63,"%02d::",calib_run_num+cnt);
                string key=key_base+buffer;
                forward_driver_->getResults(key,results);
                results.SetInt(key+"::lastRun",calib_run_num+cnt);
                results.Save(key_base+"Results.conf");
                ++cnt;
            }
        }*/



   //}


    return true;
}

