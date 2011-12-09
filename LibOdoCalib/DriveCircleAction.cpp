#include <Eigen/Dense>
#include "Stopwatch.h"
#include "ConfigFileReader.h"
#include "LaserEnvironment.h"
#include "StatsEstimator.h"
#include "Calibration.h"
#include "DriveCircleAction.h"
#include "MathHelper.h"
#include "Misc.h"


DriveCircleAction::DriveCircleAction(ConfigFileReader *config, const string& config_key,CalibBotInterface *proxy)
    :RobotAction(proxy),config_key_(config_key)
{
    cout << "configkey "<<config_key<< endl;
    direction_ = config->GetInt(config_key+"::direction",POS_FRONT);
    speed_ms_ = config->GetDouble(config_key+"::speedMs",0.5);
    cout << "speed="<<speed_ms_<< endl;
    servo_front_ = config->GetInt(config_key+"::servoFront",2350);
    servo_rear_ = config->GetInt(config_key+"::servoRear",2250);
    steer_rad_front_=config->GetDouble(config_key+"::steerDegFront",15)*M_PI/180.0;
    steer_rad_rear_=config->GetDouble(config_key+"::steerDegRear",0)*M_PI/180.0;
    //steerRad_=steerDeg*M_PI/180.0;
    double circleThreshold=config->GetDouble(config_key+"::circleThreshold",0.2);
    circle_estimator_.SetThreshold(circleThreshold);
    err_points_thresh_=config->GetDouble(config_key+"::circlePointsThreshold",0.02);
    turns_ = config->GetDouble(config_key+"::turns",1.0);
    // determines the aberration in degrees from full 360deg which is considered as full circle
    fullcircle_tolerance_=config->GetDouble(config_key+"::fullCircleToleranceDeg",2)*M_PI/180.0;
    min_delta_dist_ = 0.1;
    double steer_ctrl_kp=config->GetDouble(config_key+"::steerCtrlKp",1500);
    double steer_ctrl_ki=config->GetDouble(config_key+"::steerCtrlKi",1000);
    circle_err_tolerance_ = 0.05;
    // configure the steering angle controllers
    ctrl_front_.setPosition(0);
    ctrl_front_.setLimits(1400,3100);
    ctrl_front_.setPiParameters(steer_ctrl_kp,steer_ctrl_ki);
    ctrl_front_.setTargetAngle(steer_rad_front_);
    ctrl_rear_.setPosition(1);
    ctrl_rear_.setLimits(1400,3100);
    ctrl_rear_.setPiParameters(steer_ctrl_kp,steer_ctrl_ki);
    ctrl_rear_.setTargetAngle(steer_rad_rear_);
}


DriveCircleAction::~DriveCircleAction ()
{
}


void DriveCircleAction::setServos(int servo_front, int servo_rear, double rad_front, double rad_rear)
{
    servo_front_=servo_front;
    servo_rear_=servo_rear;
    steer_rad_front_=rad_front;
    steer_rad_rear_=rad_rear;
}





void DriveCircleAction::setSpeed (double speed_msec)
{
    speed_ms_ = speed_msec;
}


double DriveCircleAction::calcLaserDistance()
{
    Vector2d center=circle_estimator_.GetCenter();
    double radiusk = circle_estimator_.GetRadiusK();
    Vector3dList points;
    circle_estimator_.GetPointList(points, err_points_thresh_);
    cout << "errthresh="<<err_points_thresh_ << ": number of points:"<<points.size()<< endl;

    StatsEstimator<double> distStats;
    point2D p1(center.x(),center.y());
    for (Vector3dList::iterator pIt=points.begin();pIt!=points.end();++pIt) {
        double& yaw = (*pIt)(2);
        double& x = (*pIt)(0);
        double& y = (*pIt)(1);
        point2D p2(center.x()+100.0*cos(yaw+M_PI/2.0),center.y()+100.0*sin(yaw+M_PI/2.0));
        point2D p3(x,y);
        point2D p4(x+100.0*cos(yaw),y+100.0*sin(yaw));
        point2D s;
        double dummy1,dummy2;
        segmentIntersection(p1, p2, p3, p4, s,  dummy1, dummy2);
        Vector2d p3s;
        p3s << s.first-p3.first,s.second-p3.second;
        double d=p3s.norm();
        //cout << "distance ="<<d <<endl;
        laserdist_stats_.update(d);
    }
    cout << "Mean distance:"<<laserdist_stats_.getMean() << " std="<<laserdist_stats_.getStd() <<endl;
    return laserdist_stats_.getMean();
}



bool DriveCircleAction::execute()
{
    ConfigFileReader results;
    Vector3d slam_pose,new_pose;
    proxy_->Read(); // Read incoming messages
    proxy_->GetSlamPose( slam_pose );
    Stopwatch timer;
    timer.restart();
// prepare estimators
    wheelbase_stats_.reset();
    beta_stats_.reset();
    yawrate_stats_.reset();
    laserdist_stats_.reset();
    double pni_offset = proxy_->GetPniYaw();
    double pniOffset = 0.0;
    // start moving in circle


    bool is_moving=startMoving(speed_ms_,steer_rad_front_,steer_rad_rear_,0.2,2000);
    if (!is_moving) {
        cout << "robot failed to move. stopping"<< endl;
        proxy_->SetSpeedAndServos (0,servo_front_,servo_rear_);
        return false;
    }
/*
    int servo_front_val =servo_front_;
    int new_servo_front_val = servo_front_;
    int servo_rear_val =servo_rear_;
    int new_servo_rear_val = servo_rear_;

    bool steer_front_adjusted=false,steer_rear_adjusted=false;*/
    /*
    while (!steer_front_adjusted && !steer_rear_adjusted) {
        proxy_->Read();
        steer_front_adjusted = controlSteerAngle(MODE_FORWARD,steer_rad_front_,servo_front_val, new_servo_front_val);
        steer_rear_adjusted = controlSteerAngle(MODE_BACKWARD,steer_rad_rear_,servo_rear_val, new_servo_rear_val);
        cout << "new servo vals="<<new_servo_front_val<<" "<<new_servo_rear_val<<endl;
        if (new_servo_front_val>3000) new_servo_front_val=3000;
        if (new_servo_rear_val>3000) new_servo_rear_val=3000;
        if (new_servo_front_val<1500) new_servo_front_val=1500;
        if (new_servo_rear_val<1500) new_servo_rear_val=1500;

        proxy_->SetSpeedAndServos(speed_ms_,new_servo_front_val,new_servo_rear_val);
        // sleep 300msec to give the servos time to adjust
        usleep(300000);
    }
    servo_front_val=new_servo_front_val;
    servo_rear_val=new_servo_rear_val;
*/

    // robot is now driving a circle
    proxy_->GetSlamPose( slam_pose );
    double starting_yaw_pni = Misc::normalizeAngle(proxy_->GetPniYaw()-pni_offset);
    double starting_yaw_slam = Misc::normalizeAngle(slam_pose(2));
    circle_estimator_.Reset();
    circle_estimator_.AddPoint(slam_pose);
    hall1_front_stats_.reset();
    hall2_front_stats_.reset();
    hall_front_angle_stats_.reset();
    hall_rear_angle_stats_.reset();
    double cur_yaw_pni = starting_yaw_pni;
    double cur_yaw_slam = slam_pose(2);
    double prev_yaw = cur_yaw_slam;
    double yaw_sum = 0;
    double beta=0.0;
    Stopwatch round_timer;
    round_timer.restart();
    proxy_->SetSpeedSteer(speed_ms_,steer_rad_front_,steer_rad_rear_);
    Stopwatch ctrl_timer;
    ctrl_timer.restart();
    ctrl_front_.reset();
    ctrl_rear_.reset();
    bool has_circle=false;
    while (yaw_sum<turns_*2.0*M_PI) {
        usleep(50000);
        // Read incoming messages
        proxy_->Read();

        // check collision
        double threshold = 0.4;
        bool isColliding = checkCollision(beta,threshold);


        // get slam pose
        if ( !proxy_->IsFreshSlamPose()) {
            usleep( 6000 ); // Wait 6 msec
            continue;
        }
        DVector hall_volts(4);
        proxy_->GetHallVoltages(hall_volts);
        double hall_front_angle=calcHallAngle(POS_FRONT,hall_volts);
        double hall_rear_angle=calcHallAngle(POS_REAR,hall_volts);
        hall1_front_stats_.update(hall_volts[0]);
        hall2_front_stats_.update(hall_volts[1]);
        hall_front_angle_stats_.update(hall_front_angle);
        hall_rear_angle_stats_.update(hall_rear_angle);
        proxy_->GetSlamPose( new_pose );
        circle_estimator_.AddPoint(new_pose);
        double diff_yaw=Misc::normalizeAngle(new_pose(2)-prev_yaw);
        yaw_sum+=fabs(diff_yaw);
        prev_yaw = new_pose(2);
        Vector3d delta= new_pose-slam_pose;
        if (delta.head<2>().norm()>min_delta_dist_) {
            // calculate beta
            beta = calcBetaAngle(slam_pose,new_pose, direction_);
            beta_stats_.update(beta);
            slam_pose = new_pose;
        }
        proxy_->SetSpeedSteer(speed_ms_,steer_rad_front_,steer_rad_rear_);
        cur_yaw_slam = new_pose(2);
        cur_yaw_pni = Misc::normalizeAngle(proxy_->GetPniYaw()-pniOffset);//newPose(2);

        double diff_yaw_slam = Misc::normalizeAngle(cur_yaw_slam-starting_yaw_slam);

        // calculate circle data if robot has driven a full circle with at least 6 circlepoints
        if (((yaw_sum>=turns_*2*M_PI)||(isColliding)) &&
            (yaw_sum>M_PI)){
            circle_estimator_.Update();
            Vector2d center=circle_estimator_.GetCenter();
            double radiusk = circle_estimator_.GetRadiusK();
            double laser_dist=calcLaserDistance();
            radius_stats.update(radiusk);

            // calculate wheel base
            double wb, wb_sin,wb_tan;
            double beta_pred = atan(0.5*(tan(hall_front_angle_stats_.getMean())+tan(hall_rear_angle_stats_.getMean())));

            if (fabs(steer_rad_front_)<1e-10) {
                wb = radiusk*tan(steer_rad_rear_);
            } else if (fabs(steer_rad_rear_)<1e-10) {
                wb_sin = radiusk*sin(hall_front_angle_stats_.getMean());
            }
                // rough approx with lr=lf
            wb_tan = fabs(radiusk*cos(beta_pred)*(tan(hall_front_angle_stats_.getMean())-tan(hall_rear_angle_stats_.getMean())));
            cout << "*************update whele base dstats with "<<wb_tan << endl;
            wheelbase_stats_.update(wb_tan);
            double yaw_rate =(2*M_PI-diff_yaw_slam)/(round_timer.msElapsed()/1000.0);
            round_timer.restart();
            yawrate_stats_.update(yaw_rate);

            cout << "yaw sum="<<yaw_sum*180.0/M_PI << endl;
            cout << "center point:"<<center
                    <<"radiusk:" << radiusk<<" wheelbase:"<<wb_sin<< endl;
            cout << "mean beta="<<beta_stats_.getMean()*180.0/M_PI<< "std beta="<<beta_stats_.getStd()
                    << "predicted:"<<beta_pred*180.0/M_PI << endl;
            cout << "wb_tan="<<wb_tan <<"m"<<endl;
            cout << "laser distance ="<<laser_dist<<" yaw rate:"<<yaw_rate*180.0/M_PI<<endl;
            *log_stream_<< timer.msElapsed() << " "<< new_pose.x() << " "<< new_pose.y() << " "
                    << cur_yaw_slam<<" "<<cur_yaw_pni << " " <<center.x()
                    <<" "<<center.y()<<" " << radiusk <<endl;
            cout << "mena hall angel front="<<hall_front_angle_stats_.getMean()*180.0/M_PI <<
                    "rear "<<hall_rear_angle_stats_.getMean()*180.0/M_PI << endl;
            cout << "std hall angel front="<<hall_front_angle_stats_.getStd()*180.0/M_PI <<
                    "rear "<<hall_rear_angle_stats_.getStd()*180.0/M_PI << endl;
            double circErr, circStdErr;
            circle_estimator_.GetCircleQuality(circErr,circStdErr);
            cout << "circle mean abs err:"<<circErr << " stddev:"<<circStdErr<<endl;
            circle_estimator_.Reset();
            if (circErr<circle_err_tolerance_ && circStdErr<circle_err_tolerance_) {
              has_circle=true;
            } else {
              has_circle=false;
            }
          } else {
            *log_stream_<< timer.msElapsed() << " "<< new_pose.x() << " "<< new_pose.y() << " "
                    << cur_yaw_slam<<" "<<cur_yaw_pni << " 0 0 0"<< endl;

        }
        if (isColliding) {
            proxy_->SetSpeedSteer(0.0,steer_rad_front_,steer_rad_rear_);
            cout << "Stopping with no of points:" << circle_estimator_.GetPointList()->size()<< endl;
            if (has_circle) {
                return true;
            } else {
                return false;
            }
        }
        slam_pose = new_pose;

    }

    return has_circle;
}


void DriveCircleAction::getResults(const string &key, ConfigFileReader &results)
{
    results.SetDouble(key+"::speed",speed_ms_);
    results.SetInt(key+"::servoFront",servo_front_);
    results.SetInt(key+"::servoRear",servo_rear_);
    results.SetDouble(key+"::steerFrontPred",steer_rad_front_);
    results.SetDouble(key+"::steerRearPred",steer_rad_rear_);
    results.SetDouble(key+"::yawRate",yawrate_stats_.getMean());
    results.SetDouble(key+"::yawRateStd",yawrate_stats_.getStd());
    results.SetDouble(key+"::beta",beta_stats_.getMean());
    results.SetDouble(key+"::betaStd",beta_stats_.getStd());
    //results.SetDouble(key+"::betaPred",beta_pred);

    cout << "wheelbase mean="<<wheelbase_stats_.getMean() << " std="<<wheelbase_stats_.getStd() << endl;
    results.SetDouble(key+"::wheelbase",wheelbase_stats_.getMean());
    results.SetDouble(key+"::wheelbaseStd",wheelbase_stats_.getStd());
    results.SetDouble(key+"::laserDist",laserdist_stats_.getMean());
    results.SetDouble(key+"::laserDistStd",laserdist_stats_.getStd());
    results.SetDouble(key+"::hallFrontAngleMean",hall_front_angle_stats_.getMean());
    results.SetDouble(key+"::hallRearAngleMean",hall_rear_angle_stats_.getMean());
    results.SetDouble(key+"::hallFrontAngleStd",hall_front_angle_stats_.getStd());
    results.SetDouble(key+"::hallRearAngleStd",hall_rear_angle_stats_.getStd());


    results.SetDouble(key+"::hall1Mean",hall1_front_stats_.getMean());
    results.SetDouble(key+"::hall2Mean",hall2_front_stats_.getMean());
    results.SetDouble(key+"::hall1Std",hall1_front_stats_.getStd());
    results.SetDouble(key+"::hall2Std",hall2_front_stats_.getStd());


}


void DriveCircleAction::setSteerAngles(double rad_front, double rad_rear)
{
    steer_rad_front_ = rad_front;
    steer_rad_rear_ = rad_rear;
    ctrl_front_.setTargetAngle(rad_front);
    ctrl_rear_.setTargetAngle(rad_rear);
    servo_front_ = ctrl_front_.getStaticServoValue(rad_front);
    servo_rear_ = ctrl_rear_.getStaticServoValue(rad_rear);
}
