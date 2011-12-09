#ifndef ROBOTACTION_H
#define ROBOTACTION_H
#include "LaserEnvironment.h"
#include "Stopwatch.h"
#include "RobotMission.h"
#include "CalibBotInterface.h"
#include "ConfigFileReader.h"

class RobotAction : public RobotMission
{
public:
    RobotAction(CalibBotInterface *proxy);

    virtual bool execute() = 0;
    /**
      set direction of movement
      @param dir 0 (forward) or 1 (backward)
      */
    void    setDirection (int dir) {direction_=dir;}

    void    setLog(ofstream *logStream, Stopwatch* timer) {log_stream_=logStream;mission_timer_=timer;}

    /**
      write results of drive with base key in results
      */
    virtual void    getResults (const string& key,ConfigFileReader& results)=0;

protected:
    bool    checkCollision(double course,double threshold);

    /**
      moves the robot with given speed and servo steers, stops when time limit or distance is reached
      stops when colliding
      @return true if min_dist was driven, false otherwise
      */
    bool    startMoving(double speed_ms, int servo_fr, int servo_r, double min_dist,double time_limit_ms);

    bool    startMoving(double speed_ms, double steer_front_rad, double steer_rear_rad, double min_dist, double max_time_ms);

    double  calcBetaAngle(const Vector3d& p1, const Vector3d& p2, int direction);

    double  calcLsBetaAngle(Vector2dVec& ps, double theta,int direction);

    /**
      calculates measured steering angle from hall angles

      @param position front 0 or rear 1

      */
    double calcHallAngle (int position,const DVector& hall_volts);
    CalibBotInterface  *proxy_;
    LaserEnvironment    laser_env_;
    int                 direction_;
    ofstream            *log_stream_;
    Stopwatch           *mission_timer_;

};

#endif // ROBOTACTION_H
