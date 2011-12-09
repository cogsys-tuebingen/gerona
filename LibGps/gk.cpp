
#include "gk.h"
#include "ecefgk.h"

namespace GPS
{
	GKObservation::GKObservation(GPS_Record r) : Observation(r.getTimeStamp())
	{
		gpsRecord = r;
		double pos[3];
/*		GPS::wgs84_lla2gk(r.lon, r.lat, r.alt, pos);
		gk_r_m = pos[0];
		gk_h_m = pos[1];
		gk_alt_m = pos[2];*/
	}
}
