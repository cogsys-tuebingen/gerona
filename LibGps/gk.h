#ifndef GK_H
#define GK_H
#include "types.h"
#include "gps.h"

namespace GPS
{
	class GKObservation :
		public Observation
	{
	public:
		GKObservation(GPS_Record);
		double gk_r_m;
		double gk_h_m;
		double gk_alt_m;
		GPS_Record gpsRecord;
		static void writeLogHeader(ostream& os)
		{
			os<<"% GPS GK log"<<endl;
			os<<"% Format:"<<endl;
			os<<"% Time[ms] UTC[s] GK_R GK_H GK_ALT [m] Fix[1-5] lon lat msl_hght [m]heading[rad] speed [m/s] lon_vel lat_vel alt_vel[m/s] eph epv"<<endl;
		}
		friend ostream& operator<<(ostream& os, const GKObservation& obs)
		{
			os<<setprecision(13);
			os<<obs.ts.sec << " " << obs.ts.msec <<" "<<obs.gk_r_m<<" "<<obs.gk_h_m<<" "<<obs.gk_alt_m<<" "<<obs.gpsRecord<<endl;
			return os;
		}
		friend istream& operator>>(istream& is, GKObservation& obs)
		{
			is>>obs.ts.sec >> obs.ts.msec >>obs.gk_r_m>>obs.gk_h_m>>obs.gk_alt_m>>obs.gpsRecord;
			return is;
		}
	};
}

#endif
