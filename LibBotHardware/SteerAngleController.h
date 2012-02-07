#ifndef STEERANGLECONTROLLER_H
#define STEERANGLECONTROLLER_H

/**
  pi controller for regulating steering servo angle where the actual angle is measured
  e.g. using hall sensors

  */
class SteerAngleController
{
public:
    SteerAngleController();

    void setTargetAngle (double rad);

    void setLimits (int lower_lim, int upper_lim);

    void execute(double delta_t_sec, double is_rad, int& servo_val);

    void reset ();

    void setPiParameters (double kp, double ki);
    /**
      set position of servos - front 0 or rear 1
      */
    void setPosition (int position);

    /**
      @param rad steering angle in radians
      @return calibrated servo cvalue for given steering angle
      */
    int getStaticServoValue (double rad);

    /**
        if error is greater than this threshold the controller will use static servo value to drive to
      */
    void setThreshold (double threshold) {controller_threshold_=threshold;}
private:
    int     position_;

    double target_rad_;
    double sum_t_e_;
    int     operating_point_;
    int     lower_lim_, upper_lim_;
    double  controller_threshold_;
    double  Kp_,Ki_;

};

#endif // STEERANGLECONTROLLER_H
