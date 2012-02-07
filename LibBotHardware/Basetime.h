#ifndef __BASE_TIME_H__
#define __BASE_TIME_H__

#include <iostream>
#include <sys/time.h>

namespace Base
{
/**
* @class Timer 
* @brief A simple timer.
*/
	class Timer
	{
	public:
		static bool init(); // Must be called only once
		static long get_ms();
		static long getOffset();
	private:
		static long offset;
	};

/**
* @class TimeStamp 
* @brief This class provides a timestamp for use in logfiles.
*/
	class TimeStamp
	{
	private:
		long ms; // ms since start
		time_t utc_secs; // number of seconds elapsed since midnight (00:00:00), January 1, 1970, Coordinated Universal Time (UTC), according to the system clock
	public:
		TimeStamp();
		void update();
		const long get_ms();
		const long get_msWithOffset();
		const long getDifference_ms();
		const long getDifference_ms(TimeStamp);
		friend std::ostream& operator<<(std::ostream&, const TimeStamp &);
		friend std::istream& operator>>(std::istream&, TimeStamp &);
	};
}

#endif
