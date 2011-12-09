/*
 * Misc.h
 *
 *  Created on: Aug 4, 2009
 *      Author: bohlmann
 *
 *      miscellaneous utility functions
 */

#ifndef MISC_H_
#define MISC_H_
#include <sys/time.h>
#include <cmath>
#include <string>

#include "Global.h"


class Misc {
public:

/**
	return elapsed time between two timestamps
	@return time difference as double, unit is seconds
*/
	static double getTimeDiff(const struct timeval* from,
			const struct timeval* to);

/**
 * Return elapsed time between reference time and now.
 * @return time difference as double, unit is seconds
 */
	static double getTimeDiffToNow(const struct timeval* reference);

	/**
	 * Returns a string with the current time.
	 * @return time as string, format s:ms
	 */
	static std::string getPrettyTimeString();

/**
 *  suspends task for given time in milliseconds
 */
	static void  sleepMsec (Uint msec);

/**
 * Normalize an angle to (-PI : PI].
 *
 * @param angle The angle in radian.
 */
        static  inline double normalizeAngle( double angle ) {
            while (angle<-M_PI) {
                angle += ( 2.0 * M_PI );
            }
            while (angle>M_PI) {
                angle -= 2.0*M_PI;
            }
            return angle;
        }


/**
 * Expands a filename. (Replaces ~ by /home/user...)
 */
	static std::string expandFilename( const std::string filename);

 /**
  * Checks if a file exists.
  *
  * @param fileName The name of the file.
  *
  * @return True if the file exists.
  */
    static bool fileExists( const char * fileName );

    /*
     * Checks the sign of a value.
     * Returns -1 for values < 0, +1 for values > 0 and zero if
     * the value is equal to zero
     */
    //template<typename T> static T sign(T n);
    static double sign( double value);

    /*
     * Returns the folder with the configuration files for rabot.
     */
    static std::string getConfigurationFolder();
};

#endif /* MISC_H_ */
