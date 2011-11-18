/*
 * Timer.h
 *
 *  Created on: Sep 27, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

class Timer
{
public:
    Timer();

    void start();
    double stop();

protected:
    struct timeval m_start_profiling, m_end_profiling;

};

#endif // TIMER_H
