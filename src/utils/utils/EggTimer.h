/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   2/9/2012
   @file   EggTimer.h

*/ 

#ifndef EGGTIMER_H
#define EGGTIMER_H

#include <sys/time.h>
class EggTimer
{
public:
    EggTimer(int duration_ms);
    EggTimer();
    void restart();
    void restart(int duration_ms);
    int msElapsed() const;
    double sElapsed() const;
    bool isFinished() const;
    bool isRunning() const;
private:
    int     duration_ms_;
    bool    is_running;
    timeval start_;
};

#endif // EGGTIMER_H
