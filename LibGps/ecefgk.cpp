#include<iostream>
#include<fstream>
#include<string>
#include<cmath>
#include "ecefgk.h"



namespace GPS
{
	void ECEFGK(double* Pos_ECEF)
		{
		 Pos_ECEF[0] = Pos_ECEF[0];
		 Pos_ECEF[1] = Pos_ECEF[1];
		 Pos_ECEF[2] = Pos_ECEF[2];
         geo_transform_wgs84_bessel(Pos_ECEF);
         geo_ecef2lla(Pos_ECEF);
         geo_lla2gk(Pos_ECEF);	

	}
  void ecef2Lla(double *ecef, double *lla)
  {
    lla[0]=ecef[0];
    lla[1]=ecef[1];
    lla[2]=ecef[2];
    geo_transform_wgs84_bessel(lla);
    geo_ecef2lla(lla);
  }
  
	void ECEFGK(double *ecef_IN, double *gk_OUT)
	{
		double tmp[3];
		tmp[0] = ecef_IN[0];
		tmp[1] = ecef_IN[1];
		tmp[2] = ecef_IN[2];
		ECEFGK(tmp);
		gk_OUT[0] = tmp[0];
		gk_OUT[1] = tmp[1];
		gk_OUT[2] = tmp[2];
	}

	void geo_transform_wgs84_lla2ecef(double lon, double lat, double alt, double result[])
	{
		double a = 6378137;
		double e = 8.1819190842622e-2;
		double N = a / sqrt(1 - e*e * sin(lat)*sin(lat));

		result[0] = (N+alt) * cos(lat) * cos(lon);
		result[1] = (N+alt) * cos(lat) * sin(lon);
		result[2] = ((1-e*e) * N + alt) * sin(lat);
	}


	void wgs84_lla2gk(double lon, double lat, double alt, double result[])
	{
		geo_transform_wgs84_lla2ecef(lon,lat,alt,result);
		ECEFGK(result);

	}

	void geo_transform_wgs84_bessel(double* Pos)
  {
      // Umwandlung: Bessel                                  
     double wx = Pos[0];
     double wy = Pos[1];
     double wz = Pos[2];
     
     // Parameter (olabonde)
     double cx = -591.28;
     double cy = -81.35;
     double cz = -396.39;
     double rx = 7.16e-6;
     double ry = -3.57e-7;
     double rz = -7.07e-6;
     double sc = 0.999990181; 
     
     Pos[0] = ((wx*1.0+wy*rz +wz*-ry)*sc)+cx;
     Pos[1] = ((wx*-rz+wy*1.0+wz*rx)*sc)+cy;
     Pos[2] = ((wx*ry +wy*-rx+wz*1.0)*sc)+cz;
               
   } 
   
   
	void geo_ecef2lla(double* Pos)
	{
	     
		 /*
		Author:
		Michael Kleder    
		April 2006
	    
		Translation: Woerner
		Jun 19,2007
		*/
	     
		double const PI = 3.141592653589793238462643;
		double x= Pos[0];
		double y= Pos[1];
		double z= Pos[2];
	    
		// WGS84 ellipsoid constants
		double a = 6377397.155; 
		double e = 0.08169680570374;
	    
		// calculations: 
		double b = pow(pow(a,2.0)*(1-(pow(e,2.0))),0.5); 
		double ep = pow((pow(a,2.0)-pow(b,2.0))/pow(b,2.0),0.5);
		double p = pow(pow(x,2.0)+pow(y,2.0),0.5);
		double th  = atan2(a*z,b*p);
	    
	    
	    
		Pos[1] = atan2(y,x);
		Pos[0] = atan2(z+pow(ep,2.0)*b*pow(sin(th),3.0),p-pow(e,2.0)*a*pow(cos(th),3.0));   
	    
		double N = a/pow(1.0-pow(e,2.0)*pow(sin(Pos[0]),2.0),0.5);      
	    

		Pos[2] = p/cos(Pos[0])-N;           
		Pos[1] = fmod(Pos[1],(2.0*PI));
	    
	   
	     
	}
         
         
	void geo_lla2gk(double* Pos)
	{
	     
		 /*
		Author:
		Peter Wasmeier, Technical University of Munich
		p.wasmeier@bv.tum.de
		Jan 18, 2006
	    
		Translation: Woerner
		Jun 19,2007
		*/
	    
		double ELL[3];
		double const PI = 3.141592653589793238462643;
		double const ella = 6377397.155;
		double const ellb = 6356078.96300347;
		double const ellf = 0.003342773153122;
		double const m0 = 1.0;
	    
	    
		ELL[0]= Pos[0];
		ELL[1]= Pos[1];
		ELL[2]= Pos[2];   
	    

		// Do calculations   
		double B = ELL[0];
		double L = ELL[1] - 9.0*PI/180.0;
	    
		// 2. eccentricity
		double es2 = 0.006719218740482;
	    
		double V = pow(1.0+es2*pow(cos(B),2.0),0.5);
		double eta = pow(es2*pow(cos(B),2.0),0.5);
	    
		double Bf = atan(tan(B)/cos(V*L)*(1.0+pow(eta,2.0)/6.0*(1.0-3.0*pow(sin(B),2.0))*pow(L,4.0)));
		double Vf = pow(1.0+es2*pow(cos(Bf),2.0),0.5);
		double etaf = pow(es2*pow(cos(Bf),2.0),0.5);
		double n = (ella-ellb) / (ella+ellb);
	    
		// numerical series for ordinate
		double r1 = (1.0+pow(n,2.0)/4.0+pow(n,4.0)/64.0)*Bf;
		double r2 = 3.0/2.0*n*(1.0-pow(n,2.0)/8.0)*sin(2*Bf);
		double r3 = 15.0/16.0*pow(n,2.0)*(1.0-pow(n,2.0)/4.0)*sin(4*Bf);
		double r4 = 35.0/48.0*pow(n,3.0)*sin(6.0*Bf);
		double r5 = 315.0/512.0*pow(n,4.0)*sin(8.0*Bf);
	    
		// abscissa
		double ys = asinhown(tan(L)*cos(Bf)/Vf*(1.0+pow(etaf,2.0)*pow(L,2.0)*pow(cos(Bf),2.0)*(pow(etaf,2.0)/6.0+pow(L,2.0)/10.0))); 
		double y = m0*pow(ella,2.0)/ellb*ys;

	   
	        
		Pos[0] = y+5e5+ 3.0*1e6;
		Pos[1] = ella/(1.0+n)*(r1-r2+r3-r4+r5)*m0;
		Pos[2] = ELL[2];
	}
	double asinhown(double in)
	{

	return log(in + pow((pow(in,2.0) + 1.0),0.5));

	}
}

	