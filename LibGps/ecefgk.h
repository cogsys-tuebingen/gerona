#ifndef ECEFGK_H
#define ECEFGK_H
#include<iostream>
#include<fstream>
#include<string>
#include <cmath>


using namespace std;

namespace GPS
{
	void ECEFGK(double*);
	void ECEFGK(double*, double*);
  
  /**
    transform ecef coordinates to latitude longitude alt
  */
  void ecef2Lla(double *ecef, double *lla);

	void wgs84_lla2gk(double lon, double lat, double alt, double result[]);
	void geo_transform_wgs84_lla2ecef(double lon, double lat, double alt, double result[]);
	void geo_transform_wgs84_bessel(double*);
	void geo_ecef2lla(double*);
	void geo_lla2gk(double*);
	double asinhown(double);

}

#endif
