#include "Basetime.h"
#include <assert.h>
#include <iostream>

namespace Base
{
	long Timer::offset;
	
	long Timer::getOffset()
	{
		return offset;
	}

	long Timer::get_ms()
	{
		struct timeval timeOfDay;
		struct timezone tz;
		gettimeofday(&timeOfDay, &tz);

		return timeOfDay.tv_sec*1000 + timeOfDay.tv_usec/1000 - offset;
	}

	bool Timer::init()
	{
		offset = 0;
		offset = get_ms();
		return true;
	}

	TimeStamp::TimeStamp()
	{
		ms = Timer::get_ms();
		time(&utc_secs);
	}
	
	void TimeStamp::update()
	{
		ms = Timer::get_ms();
		time(&utc_secs);
	}

	const long TimeStamp::get_ms()
	{
		return ms;
	}

	const long TimeStamp::get_msWithOffset()
	{
		return ms+Timer::getOffset();
	}

	const long TimeStamp::getDifference_ms()
	{
		return Timer::get_ms()-ms;
	}

	const long TimeStamp::getDifference_ms(TimeStamp other)
	{
		return ms-other.get_ms();
	}

	std::ostream& operator<<(std::ostream& os, const TimeStamp &ts)
	{
		os << ts.ms << " " << ts.utc_secs; 
		return os;
	}
	
	std::istream& operator>>(std::istream& is, TimeStamp &ts)
	{
		is >> ts.ms >> ts.utc_secs; 
		return is;
	}

}
