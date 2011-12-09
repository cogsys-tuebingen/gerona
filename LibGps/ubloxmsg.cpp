#include <string>
#include <iostream>
#include <stdlib.h>
#include <math.h>	
#include "ubloxmsg.h"

namespace GPS
{
	void x02x10::read(char  sentence[1024])
	{
		NSV = (unsigned char) sentence[6];
		Week = (unsigned char) sentence[4] +  (unsigned char) sentence[5]*256;
		ITOW_ms = (unsigned char) sentence[0] +  (unsigned char) sentence[1]*256 + (unsigned char) sentence[2]*65536 + (unsigned char) sentence[3]*16777216; 
				
		signedcheck(ITOW_ms);
		signedcheck(Week);

		for (count = 0; count < NSV; count++)
		{
			// each SV
			SV[count] = sentence[28+24*count];

			Mestemp[0] = sentence[8+24*count];
			Mestemp[1] = sentence[9+24*count];
			Mestemp[2] = sentence[10+24*count];
			Mestemp[3] = sentence[11+24*count];
			Mestemp[4] = sentence[12+24*count];
			Mestemp[5] = sentence[13+24*count];
			Mestemp[6] = sentence[14+24*count];
			Mestemp[7] = sentence[15+24*count];
			Mestemp[8] = '\0';
			double* pf1 = reinterpret_cast<double*>(Mestemp);
		    CPMes[count] = *pf1;

	        Mestemp[0] = sentence[16+24*count];
			Mestemp[1] = sentence[17+24*count];
			Mestemp[2] = sentence[18+24*count];
			Mestemp[3] = sentence[19+24*count];
			Mestemp[4] = sentence[20+24*count];
			Mestemp[5] = sentence[21+24*count];
			Mestemp[6] = sentence[22+24*count];
			Mestemp[7] = sentence[23+24*count];
			Mestemp[8] = '\0';
			double* pf2 = reinterpret_cast<double*>(Mestemp);
		   PRMes[count] = *pf2;

			Mestemp[0] = sentence[24+24*count];
			Mestemp[1] = sentence[25+24*count];
			Mestemp[2] = sentence[26+24*count];
			Mestemp[3] = sentence[27+24*count];
			Mestemp[4] = '\0';
			float* pf3 = reinterpret_cast<float*>(Mestemp); 							
			DOMes[count] = *pf3;	

			MesQI[count] = sentence[29+24*count];	

			CNO[count] = sentence[30+24*count];
			
			LLI[count] = sentence[31+24*count];



			signedcheck(MesQI[count]);
			signedcheck(CNO[count]);

		}
	}



	void x02x20::read(char  sentence[1024])
	{
		unsigned char NumVisUChar = (unsigned char) sentence[6];
		unsigned char NumSVUChar = (unsigned char) sentence[7];
		Week = (unsigned char)sentence[4] +  (unsigned char)sentence[5]*256;
		ITOW_ms = (unsigned char)sentence[0] + (unsigned char)sentence[1]*256 + (unsigned char)sentence[2]*65536 + (unsigned char)sentence[3]*16777216;
		
		signedcheck(ITOW_ms);
		signedcheck(Week);

		NumVis = NumVisUChar;
		NumSV = NumSVUChar;

		for (int count = 0; count < NumSV; count++)
		{
			SVID[count] = (int)sentence[8+6*count];			
			SVFlag[count] = (int)sentence[9+6*count];						
			
			int AzimInt = sentence[10+6*count] + sentence[11+6*count]*256;
			signedcheck(AzimInt);
			Azim[count]=AzimInt;

			char ElevChar = sentence[12+6*count];
			char ElevChar2 = ElevChar;
			signedcheck(ElevChar2);
			Elev[count] = (int)ElevChar2;
			Age[count] = sentence[13+6*count];
			

		}
	}


	void x02x31::read(char  sentence[1024])
	{
		unsigned char sentence2[1024];
		for (int i=0; i<1024; i++)
		{
			sentence2[i] = sentence[i];
		}

		double const PI = 3.141592653589793238462643;

		SVID = sentence2[0] + sentence2[1]*256 + sentence2[2]*65536 + sentence2[3]*16777216;
		HOW  = sentence2[4] + sentence2[5]*256 + sentence2[6]*65536 + sentence2[7]*16777216; // 0:invalid


		week = sentence2[10]*4 + ((sentence2[9])& '\x80')/128*2 + ((sentence2[9])& '\x40')/64*1;
		signedcheck2(week,0.0,0.0);

		toc_s = (sentence2[30]*pow(2.0,8.0)+ sentence2[29]); 
		signedcheck2(toc_s,0.0,4.0);

		Clockdriftrate_sds2	= sentence2[34];
		signedcheck2(Clockdriftrate_sds2,8.0,-55.0);

		Clockdrift_sds = (sentence2[33]*pow(2.0,8.0)+ sentence2[32]);
		signedcheck2(Clockdrift_sds,16.0,-43.0);

		//Clockbias_s	= ( ((sentence2[9])& '\xFC')/4*65536 + sentence2[37]*pow(2.0,8.0)+ sentence2[36]);
		Clockbias_s	=  (sentence2[38]*pow(2.0,14.0)+ sentence2[37]*pow(2.0,6.0)+ ((sentence2[36])& '\xFC')/4);
		signedcheck2(Clockbias_s,22.0,-31.0);//

		IODE1 = sentence2[42]*1;
		signedcheck2(IODE1,0.0,0.0);

		CRS_m = (sentence2[41]*pow(2.0,8.0)+ sentence2[40]);
		signedcheck2(CRS_m,16.0,-5.0);

		deltan_semi_circlesdsee = (sentence2[46]*pow(2.0,8.0)+ sentence2[45]);//
		signedcheck2(deltan_semi_circlesdsee,16.0,-43.0);
		deltan_semi_circlesdsee = deltan_semi_circlesdsee * PI;

		M0_semicircles = (sentence2[44]*pow(2.0,24.0)+ sentence2[50]*pow(2.0,16.0)+ sentence2[49]*pow(2.0,8.0)+ sentence2[48]);
		signedcheck2(M0_semicircles,32.0,-31.0);
		M0_semicircles = M0_semicircles * PI;

		Cuc_rad	= (sentence2[54]*pow(2.0,8.0)+ sentence2[53]);
		signedcheck2(Cuc_rad,16.0,-29.0);//

		e_ = (sentence2[52]*pow(2.0,24.0)+ sentence2[58]*pow(2.0,16.0)+ sentence2[57]*pow(2.0,8.0)+ sentence2[56]);
		signedcheck2(e_,0.0,-33.0);

		Cus_rad	= (sentence2[62]*pow(2.0,8.0)+ sentence2[61]);
		signedcheck2(Cus_rad,16.0,-29.0);

		A_sqr_m	= (sentence2[60]*pow(2.0,24.0)+ sentence2[66]*pow(2.0,16.0)+ sentence2[65]*pow(2.0,8.0)+ sentence2[64]);															 
		signedcheck2(A_sqr_m,0.0,-19.0);

		toe_s = (sentence2[70]*pow(2.0,8.0)+ sentence2[69]);
		signedcheck2(toe_s,0.0,4.0);

		Cic_rad	= (sentence2[74]*pow(2.0,8.0)+ sentence2[73]);
		signedcheck2(Cic_rad,16.0,-29.0);

		Omega0_semicircles	= (sentence2[72]*pow(2.0,24.0)+ sentence2[78]*pow(2.0,16.0)+ sentence2[77]*pow(2.0,8.0)+ sentence2[76]);
		signedcheck2(Omega0_semicircles	,32.0,-31.0);
		Omega0_semicircles = Omega0_semicircles * PI;

		Cis_rad	= (sentence2[82]*pow(2.0,8.0)+ sentence2[81]);
		signedcheck2(Cis_rad,16.0,-29.0);

		io_semicircles = (sentence2[80]*pow(2.0,24.0)+ sentence2[86]*pow(2.0,16.0)+ sentence2[85]*pow(2.0,8.0)+ sentence2[84]);
		signedcheck2(io_semicircles,32.0,-31.0);
		io_semicircles = io_semicircles * PI;

		Crc_m = (sentence2[90]*pow(2.0,8.0)+ sentence2[89]);
		signedcheck2(Crc_m,16.0,-5.0);

		omega_semicircles = (sentence2[88]*pow(2.0,24.0)+ sentence2[94]*pow(2.0,16.0)+ sentence2[93]*pow(2.0,8.0)+ sentence2[92]);
		signedcheck2(omega_semicircles,32.0,-31.0);
		omega_semicircles = omega_semicircles * PI;

		omegadot_semicirclesds  = (sentence2[98]*pow(2.0,16.0)+ sentence2[97]*pow(2.0,8.0)+ sentence2[96]);
		signedcheck2(omegadot_semicirclesds,24.0,-43.0);
		omegadot_semicirclesds = omegadot_semicirclesds * PI;

		idot_semicirclesds = (((sentence2[100])& '\xFC')/4 + sentence2[101]*64);
		signedcheck2(idot_semicirclesds,14.0,-43.0);
		idot_semicirclesds = idot_semicirclesds * PI;
								

	}

	void signedcheck2(double& inout, double size, double scale)
	{
		if (inout >= pow(2.0,size)/2) 
		{
			if (size !=0.0) inout = inout - pow(2.0,size);				
		}

		inout = inout * pow(2.0,scale);	
}

	bool x02x31::SatPos(double Send_time_s, double SatPos[])
	{
		double delta_t_s = Send_time_s - toe_s;
			
			// Correct for sat clock error
		Send_time_s -= delta_t_s * Clockdrift_sds + Clockbias_s;
	
		if (HOW != 0)
		{	
			double A_m = pow(A_sqr_m,2);
			double const GM = 3986004.418e8;
			double const PI = 3.141592653589793238462643;
			double const Omegae_dot = 7.2921151467e-5; 
			double tk_s;
			double s = Send_time_s-toe_s;

			if (s>302400) tk_s = s-2*302400;
			else if (s < -302400) tk_s = s+2*302400;
			else tk_s = s;

			double n0 = sqrt(GM/pow(A_m, 3.0));
			double n = n0 + deltan_semi_circlesdsee;
			double M = M0_semicircles + n * tk_s;
			M = fmod(M+2*PI , 2*PI); 
			double E = M;
			double E_old;
			double dE;

			for (int i = 0; i < 10; i++)
			{
				E_old = E;
				E = M+e_*sin(E);
				dE = fmod( E-E_old , 2*PI);
				if (fabs(dE) < 1.e-12) break;
			}

			E = fmod (E+2*PI , 2*PI);

			double v = atan2 (sqrt(1-pow(e_, 2.0))*sin(E), cos(E)-e_);
			double us =  v + omega_semicircles;
			us = fmod(us, 2*PI);
			double u = us + Cuc_rad*cos(2*us)+ Cus_rad*sin(2*us);
			double r = A_m*(1-e_*cos(E)) + Crc_m*cos(2*us)+CRS_m*sin(2*us);
			double i = io_semicircles + idot_semicirclesds* tk_s + Cic_rad*cos(2*us)+ Cis_rad*sin(2*us);
			double Omega = Omega0_semicircles+(omegadot_semicirclesds-Omegae_dot)*tk_s-Omegae_dot*toe_s;
			Omega = fmod ( Omega + 2*PI, 2*PI);
			double x0 = cos(u)*r;
			double y0 = sin(u)*r;

			SatPos[0] = x0*cos(Omega)-y0*cos(i)*sin(Omega);
			SatPos[1] = x0*sin(Omega)+y0*cos(i)*cos(Omega);
			SatPos[2] = y0*sin(i);
			return true;
		}
		else
		{
			SatPos[0] = 0.0;
			SatPos[1] = 0.0;
			SatPos[2] = 0.0;
			return false;
		}		
	}



	void x01x06::read(char  sentence[1024])
		{

		ITOW_ms =   (unsigned char)sentence[0] + (unsigned char)sentence[1]*256 + (unsigned char)sentence[2]*65536 + (unsigned char)sentence[3]*16777216;
		Frac_ns =   (unsigned char)sentence[4] + (unsigned char)sentence[5]*256 + (unsigned char)sentence[6]*65536 + (unsigned char)sentence[7]*16777216;
		week =      (unsigned char)sentence[8] +  (unsigned char)sentence[9]*256;
		GPSfix =    (unsigned char)sentence[10];
		Flags =     (unsigned char)sentence[11];
		ECEF_X_cm = (unsigned char)sentence[12] + (unsigned char)sentence[13]*256 + (unsigned char)sentence[14]*65536 + (unsigned char)sentence[15]*16777216;
		ECEF_Y_cm = (unsigned char)sentence[16] + (unsigned char)sentence[17]*256 + (unsigned char)sentence[18]*65536 + (unsigned char)sentence[19]*16777216;
		ECEF_Z_cm = (unsigned char)sentence[20] + (unsigned char)sentence[21]*256 + (unsigned char)sentence[22]*65536 + (unsigned char)sentence[23]*16777216;
		Pacc_cm =   (unsigned char)sentence[24] + (unsigned char)sentence[25]*256 + (unsigned char)sentence[26]*65536 + (unsigned char)sentence[27]*16777216;
		ECEFVX_cms =(unsigned char)sentence[28] + (unsigned char)sentence[29]*256 + (unsigned char)sentence[30]*65536 + (unsigned char)sentence[31]*16777216;
		ECEFVY_cms =(unsigned char)sentence[32] + (unsigned char)sentence[33]*256 + (unsigned char)sentence[34]*65536 + (unsigned char)sentence[35]*16777216;
		ECEFVZ_cms =(unsigned char)sentence[36] + (unsigned char)sentence[37]*256 + (unsigned char)sentence[38]*65536 + (unsigned char)sentence[39]*16777216;		
		SAcc_cms =  (unsigned char)sentence[40] + (unsigned char)sentence[41]*256 + (unsigned char)sentence[42]*65536 + (unsigned char)sentence[43]*16777216;
		PDOP =      (unsigned char)sentence[44] +  (unsigned char)sentence[45]*256;				// scale 0.01
		numSV =     (unsigned char)sentence[47];

		signedcheck(Frac_ns);
		signedcheck(week);
		signedcheck(ECEF_X_cm);
		signedcheck(ECEF_Y_cm);
		signedcheck(ECEF_Z_cm);
		signedcheck(ECEFVX_cms);
		signedcheck(ECEFVY_cms);
		signedcheck(ECEFVZ_cms);
		
		}

	void x01x12::read(char  sentence[1024])
		{

		ITOW_ms =    (unsigned char)sentence[0]  + (unsigned char)sentence[1]*256  + (unsigned char)sentence[2]*65536  + (unsigned char)sentence[3]*16777216;
		VEL_N_cms =  (unsigned char)sentence[4]  + (unsigned char)sentence[5]*256  + (unsigned char)sentence[6]*65536  + (unsigned char)sentence[7]*16777216;
		VEL_E_cms =  (unsigned char)sentence[8]  + (unsigned char)sentence[9]*256  + (unsigned char)sentence[10]*65536 + (unsigned char)sentence[11]*16777216;
		VEL_D_cms =  (unsigned char)sentence[12] + (unsigned char)sentence[13]*256 + (unsigned char)sentence[14]*65536 + (unsigned char)sentence[15]*16777216;
		Speed_cms =  (unsigned char)sentence[16] + (unsigned char)sentence[17]*256 + (unsigned char)sentence[18]*65536 + (unsigned char)sentence[19]*16777216;
		GSpeed_cms = (unsigned char)sentence[20] + (unsigned char)sentence[21]*256 + (unsigned char)sentence[21]*65536 + (unsigned char)sentence[23]*16777216;
		Heading_deg =(unsigned char)sentence[24] + (unsigned char)sentence[25]*256 + (unsigned char)sentence[26]*65536 + (unsigned char)sentence[27]*16777216;	//1e-5
		Sacc_cms =   (unsigned char)sentence[28] + (unsigned char)sentence[29]*256 + (unsigned char)sentence[30]*65536 + (unsigned char)sentence[31]*16777216;
		CAcc_deg =   (unsigned char)sentence[32] + (unsigned char)sentence[33]*256 + (unsigned char)sentence[34]*65536 + (unsigned char)sentence[35]*16777216;								// 1e-5
		

		//signedcheck(VEL_N_cms);
		//signedcheck(VEL_E_cms);
		//signedcheck(VEL_D_cms);
		signedcheck(Heading_deg);
		}

	void x01x04::read(char  sentence[1024])
		{

		ITOW_ms =   (unsigned char)sentence[0] + (unsigned char)sentence[1]*256 + (unsigned char)sentence[2]*65536 + (unsigned char)sentence[3]*16777216;
		GDOP =      (unsigned char)sentence[4] +  (unsigned char)sentence[5]*256;				// scale 0.01
		PDOP =      (unsigned char)sentence[6] +  (unsigned char)sentence[7]*256;				// scale 0.01
		TDOP =      (unsigned char)sentence[7] +  (unsigned char)sentence[9]*256;				// scale 0.01
		VDOP =      (unsigned char)sentence[10] +  (unsigned char)sentence[11]*256;				// scale 0.01
		HDOP =      (unsigned char)sentence[12] +  (unsigned char)sentence[13]*256;				// scale 0.01
		NDOP =      (unsigned char)sentence[14] +  (unsigned char)sentence[15]*256;				// scale 0.01
		EDOP =      (unsigned char)sentence[16] +  (unsigned char)sentence[17]*256;				// scale 0.01
				
		}

        void x01x02::read(char  sentence[1024])
                {

                ITOW_ms =    (unsigned char)sentence[0]  + (unsigned char)sentence[1]*256  + (unsigned char)sentence[2]*65536  + (unsigned char)sentence[3]*16777216;
                Lon_deg =  (unsigned char)sentence[4]  + (unsigned char)sentence[5]*256  + (unsigned char)sentence[6]*65536  + (unsigned char)sentence[7]*16777216;
                Lat_deg =  (unsigned char)sentence[8]  + (unsigned char)sentence[9]*256  + (unsigned char)sentence[10]*65536 + (unsigned char)sentence[11]*16777216;
                Height_mm =  (unsigned char)sentence[12] + (unsigned char)sentence[13]*256 + (unsigned char)sentence[14]*65536 + (unsigned char)sentence[15]*16777216;
                HMSL_mm =  (unsigned char)sentence[16] + (unsigned char)sentence[17]*256 + (unsigned char)sentence[18]*65536 + (unsigned char)sentence[19]*16777216;
                HAcc_mm =(unsigned char)sentence[20] + (unsigned char)sentence[21]*256 + (unsigned char)sentence[22]*65536 + (unsigned char)sentence[23]*16777216;
                VAcc_mm =(unsigned char)sentence[24] + (unsigned char)sentence[25]*256 + (unsigned char)sentence[26]*65536 + (unsigned char)sentence[27]*16777216;	//1e-5

                //signedcheck(Lon_deg);
                //signedcheck(Lat_deg);
                //signedcheck(Height_mm);
                }

        void x01x03::read(char  sentence[1024])
                {

                ITOW_ms =    (unsigned char)sentence[0]  + (unsigned char)sentence[1]*256  + (unsigned char)sentence[2]*65536  + (unsigned char)sentence[3]*16777216;
                gpsFix =  (unsigned char)sentence[4];
                flags =  (unsigned char)sentence[5];
                diffStat =  (unsigned char)sentence[6];
                res =  (unsigned char)sentence[7];
                ttff = (unsigned char)sentence[8] + (unsigned char)sentence[9]*256 + (unsigned char)sentence[10]*65536 + (unsigned char)sentence[11]*16777216;
                msss = (unsigned char)sentence[12] + (unsigned char)sentence[13]*256 + (unsigned char)sentence[14]*65536 + (unsigned char)sentence[15]*16777216;	//1e-5

                }

	void x0Bx02::read(char  sentence[1024])
		{
		// This message contains a health bit mask, UTC time and Klobuchar parameters.
		// For more information on these parameters, please see the ICD-GPS-200 documentation.
		HEALTH =   (unsigned char)sentence[0] + (unsigned char)sentence[1]*256 + (unsigned char)sentence[2]*65536 + (unsigned char)sentence[3]*16777216;

		//R8
		Mestemp[0] = sentence[4];
		Mestemp[1] = sentence[5];
		Mestemp[2] = sentence[6];
		Mestemp[3] = sentence[7];
		Mestemp[4] = sentence[8];
		Mestemp[5] = sentence[9];
		Mestemp[6] = sentence[10];
		Mestemp[7] = sentence[11];
		Mestemp[8] = '\0';
		double* pf1 = reinterpret_cast<double*>(Mestemp);
		UTC_A1 = *pf1;

		Mestemp[0] = sentence[12];
		Mestemp[1] = sentence[13];
		Mestemp[2] = sentence[14];
		Mestemp[3] = sentence[15];
		Mestemp[4] = sentence[16];
		Mestemp[5] = sentence[17];
		Mestemp[6] = sentence[18];
		Mestemp[7] = sentence[19];
		Mestemp[8] = '\0';
		double* pf2 = reinterpret_cast<double*>(Mestemp);
		UTC_A0 = *pf2;		
	
		UTC_TOT  = (unsigned char)sentence[20] + (unsigned char)sentence[21]*256 + (unsigned char)sentence[22]*65536 + (unsigned char)sentence[23]*16777216;
		signedcheck(UTC_TOT);
		//I2
		UTC_WNT =   (unsigned char)sentence[24] +  (unsigned char)sentence[25]*256;
		signedcheck(UTC_WNT);
		UTC_LS =    (unsigned char)sentence[26] +  (unsigned char)sentence[27]*256;
		signedcheck(UTC_LS);
		UTC_WNF =   (unsigned char)sentence[28] +  (unsigned char)sentence[29]*256;
		signedcheck(UTC_WNF);
		UTC_DN =    (unsigned char)sentence[30] +  (unsigned char)sentence[31]*256;
		signedcheck(UTC_DN);
		UTC_LSF =   (unsigned char)sentence[32] +  (unsigned char)sentence[33]*256;
		signedcheck(UTC_LSF);
		UTC_Spare = (unsigned char)sentence[34] +  (unsigned char)sentence[35]*256;
		

		//R4
		Mestemp[0] = sentence[36];
		Mestemp[1] = sentence[37];
		Mestemp[2] = sentence[38];
		Mestemp[3] = sentence[39];
		Mestemp[4] = '\0';
		float* pf3 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_A0 = *pf3;

		Mestemp[0] = sentence[40];
		Mestemp[1] = sentence[41];
		Mestemp[2] = sentence[42];
		Mestemp[3] = sentence[43];
		Mestemp[4] = '\0';
		float* pf4 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_A1 = *pf4;

		Mestemp[0] = sentence[44];
		Mestemp[1] = sentence[45];
		Mestemp[2] = sentence[46];
		Mestemp[3] = sentence[47];
		Mestemp[4] = '\0';
		float* pf5 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_A2 = *pf5;

		Mestemp[0] = sentence[48];
		Mestemp[1] = sentence[49];
		Mestemp[2] = sentence[50];
		Mestemp[3] = sentence[51];
		Mestemp[4] = '\0';
		float* pf6 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_A3 = *pf6;

		Mestemp[0] = sentence[52];
		Mestemp[1] = sentence[53];
		Mestemp[2] = sentence[54];
		Mestemp[3] = sentence[55];
		Mestemp[4] = '\0';
		float* pf7 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_B0 = *pf7;

		Mestemp[0] = sentence[56];
		Mestemp[1] = sentence[57];
		Mestemp[2] = sentence[58];
		Mestemp[3] = sentence[59];
		Mestemp[4] = '\0';
		float* pf8 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_B1 = *pf8;

		Mestemp[0] = sentence[60];
		Mestemp[1] = sentence[61];
		Mestemp[2] = sentence[62];
		Mestemp[3] = sentence[63];
		Mestemp[4] = '\0';
		float* pf9 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_B2 = *pf9;

		Mestemp[0] = sentence[64];
		Mestemp[1] = sentence[65];
		Mestemp[2] = sentence[66];
		Mestemp[3] = sentence[67];
		Mestemp[4] = '\0';
		float* pf10 = reinterpret_cast<float*>(Mestemp); 							
		KLOB_B3 = *pf10;

		//U4
		FLAGS = (unsigned char)sentence[68] + (unsigned char)sentence[69]*256 + (unsigned char)sentence[70]*65536 + (unsigned char)sentence[71]*16777216;
		}
	
	void signedcheck(long& inout)
	{
	if (inout<0) inout = abs(inout) - 2147483647;
	}

	void signedcheck(int& inout)
	{
	if (inout<0) inout = abs(inout) - 32767;
	}

	void signedcheck(char& inout)
	{
	if (inout<0) inout = abs(inout) - 127;
	}





}
