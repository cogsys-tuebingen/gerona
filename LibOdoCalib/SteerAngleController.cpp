#include "Global.h"
#include <math.h>
#include "SteerAngleController.h"

ClientSteerAngleController::ClientSteerAngleController()
    :position_(0),lower_lim_(1500),upper_lim_(3000),operating_point_(2250),target_rad_(0),sum_t_e_(0)
{
    controller_threshold_=10.0*M_PI/180.0;
    Kp_=400;
    Ki_=600;
}


void ClientSteerAngleController::setTargetAngle (double delta)
{
    target_rad_ = delta;
    operating_point_ = getStaticServoValue(delta);
    cout << "operating point:"<<operating_point_<< endl;

    sum_t_e_=0;
}


void ClientSteerAngleController::setLimits (int lower_lim, int upper_lim)
{
    lower_lim_=lower_lim;
    upper_lim_=upper_lim;
}


void ClientSteerAngleController::reset()
{
    sum_t_e_ = 0;
}


void ClientSteerAngleController::setPosition(int pos)
{
    if (pos!=position_) {
        reset();
        position_ = pos;
    }
}


void ClientSteerAngleController::setPiParameters(double kp, double ki)
{
    Kp_ = kp;
    Ki_ = ki;
}


void ClientSteerAngleController::execute(double delta_t_sec, double is_angle_rad, int& servo_val)
{
    double e=target_rad_-is_angle_rad;
    if (fabs(e)>controller_threshold_) {
        // difference too large, use static value
        sum_t_e_=0;
        servo_val =operating_point_;
        cout << "return operating piont"<<servo_val << " isangle="<<is_angle_rad*180.0/M_PI<<endl;
        return;
    }

    sum_t_e_ +=delta_t_sec*e;
    servo_val = Kp_*e+Ki_*sum_t_e_+operating_point_;

    cout << "operating:"<<operating_point_<<"servo val:"<<servo_val << "Ki*sumte="<<Ki_*sum_t_e_<< " Kp*e"<<Kp_*e<<" ki="<<Ki_<< endl;

    if (servo_val>upper_lim_) {
        servo_val=upper_lim_;
    }
    if (servo_val<lower_lim_) {
        servo_val = lower_lim_;
    }
}


int ClientSteerAngleController::getStaticServoValue (double delta)
{
    // values for thrain
    if (position_==0) {
        return round(1152.818069*delta+2201.367827);
    } else {
        return round( 1371.449133*delta+2218.744219);
    }

    /*
    if (position_==0) {

        // values for gloin front servo
        // todo config
        return  round(1109.401709*delta+2166.786176);
    } else {
        return round(1480.456139*delta+2226.536816);
    }
    */
}

