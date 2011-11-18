/*
 * Timer.cpp
 *
 *  Created on: Sep 27, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "Timer.h"

Timer::Timer()
{
}

void Timer::start()
{
  gettimeofday(&m_start_profiling, 0);
}

double Timer::stop()
{
  gettimeofday(&m_end_profiling, 0);
  double start_t = m_start_profiling.tv_sec + double(m_start_profiling.tv_usec) / 1e6;
  double end_t = m_end_profiling.tv_sec + double(m_end_profiling.tv_usec) / 1e6;
  double t_diff = end_t - start_t;

  return t_diff * 1000.f;
}
