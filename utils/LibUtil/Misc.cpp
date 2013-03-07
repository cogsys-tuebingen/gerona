/*
 * Misc.cpp
 *
 *  @date Aug 4, 2009
 *  @author bohlmann
 */

#include "Misc.h"

#include <wordexp.h>
#include <fstream>
#include <sstream>

using namespace std;

double Misc::getTimeDiff(const struct timeval* from,
		const struct timeval* to)
{
	long seconds = to->tv_sec - from->tv_sec;
	long useconds = to->tv_usec - from->tv_usec;
	if(useconds < 0) {
		useconds += 1000000;
		seconds--;
	}
	double dt = (double)seconds+useconds/1000000.0;
	return dt;
}


double Misc::getTimeDiffToNow(const struct timeval* reference)
{
	timeval now;
	gettimeofday(&now, 0);
	double difference = Misc::getTimeDiff(reference, &now);
	return difference;
}

string Misc::getPrettyTimeString()
{
	timeval now;
	gettimeofday(&now, 0);
	stringstream s;
	s << now.tv_sec << "." << (int)now.tv_usec / 1000;
	return s.str();
}


void Misc::sleepMsec(Uint msec)
{
	struct timespec delay;
	delay.tv_sec = msec/1000;
	delay.tv_nsec = (msec-delay.tv_sec)*1000000;
	nanosleep(&delay,0);
}

std::string Misc::expandFilename(const std::string filename) {
	wordexp_t exp_result;
	wordexp(filename.c_str(), &exp_result, 0);
	return std::string( exp_result.we_wordv[0] );
}

bool Misc::fileExists( const char * fileName ) {
    // Try to open the file
    std::ifstream in( fileName );
    if ( in ) {
        in.close();
        return true;
    }
    return false;
}

/*template<typename T> T Misc::sign(T n)
{
	if (n < 0) return -1;
	if (n > 0) return 1;
	return 0;
}*/
double Misc::sign(double value) {
	if (value < 0) return -1;
	if (value > 0) return 1;
	return 0;
}


std::string Misc::getConfigurationFolder() {
	std::string result = "~/Config/";
	result = Misc::expandFilename(result);
	return result;
}
