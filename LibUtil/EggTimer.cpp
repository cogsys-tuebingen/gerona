/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   2/9/2012
   @file   EggTimer.cpp

*/ 
#include <cstdio>

#include "EggTimer.h"

EggTimer::EggTimer()
  :duration_ms_(0),is_running(false)
{

}


EggTimer::EggTimer(int duration_ms)
  :duration_ms_(duration_ms),is_running(false)
{

}


void EggTimer::restart()
{
  gettimeofday(&start_, NULL);
  is_running=true;
}


void EggTimer::restart(int duration_ms)
{
  duration_ms_=duration_ms;
  restart();
}

int EggTimer::msElapsed() const {
  if (!is_running) {
    return 0;
  } else {
    timeval t;
    gettimeofday(&t, NULL);

    int msElapsed = (t.tv_sec - start_.tv_sec)*1000;
    msElapsed += (t.tv_usec - start_.tv_usec)/1000;
    return msElapsed;
  }
}


bool EggTimer::isFinished() const
{
  if (!is_running) {
    return true;
  } else {
    timeval t;
    gettimeofday(&t, NULL);
    int elapsed_ms = (t.tv_sec - start_.tv_sec)*1000;
    elapsed_ms += (t.tv_usec - start_.tv_usec)/1000;
    return elapsed_ms>duration_ms_;
  }
}


double EggTimer::sElapsed() const {
    return msElapsed() / 1000.0;
}


