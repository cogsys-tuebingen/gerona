#ifndef UBLOX_MSG_H
#define UBLOX_MSG_H

// ********************************************************
// This file contains message definitions of UBlox protocol
// ********************************************************
#include <string>
#include <iostream>

using namespace std;

namespace GPS
{
	class x02x10{
	//private:

	public:		
		x02x10();
		~x02x10();


		unsigned char NSV;						
		int Week;
		long ITOW_ms;

		unsigned char SV[20];
		double CPMes[20];
		double PRMes[20];
		float DOMes[20];
		unsigned char Mestemp[9];

		char MesQI[20];
		char CNO[20];
		unsigned char LLI[20];

		int count;
		void read(char [1024]);

		double Rmi[20];
		double PSR[20];
		double PSR_delta[20];
	};


	inline x02x10::x02x10()
	{    
	}

	inline x02x10::~x02x10()
	{	   
	}



	class x02x20{
	//private:

	public:		
		x02x20();
		~x02x20();

		int NumSV;
		int NumVis ;
		int Week;
		long ITOW_ms;

		int SVID[50];
		int SVFlag[50];
		int Azim[50];
		int Elev[50];
		int Age[50];


		int count;
		void read(char [1024]);

		

	};


	inline x02x20::x02x20()
	{    
	}

	inline x02x20::~x02x20()
	{	   
	}



	class x02x31{
	//private:

	public:		
		x02x31();
		~x02x31();

		
		unsigned long SVID;
		unsigned long HOW;

		double week, toc_s, Clockdriftrate_sds2, Clockdrift_sds, Clockbias_s;				
		double IODE1, CRS_m, deltan_semi_circlesdsee, M0_semicircles, Cuc_rad;
		double e_, Cus_rad, A_sqr_m, toe_s, Cic_rad;					
		double Omega0_semicircles, Cis_rad, io_semicircles, Crc_m, omega_semicircles;		
		double omegadot_semicirclesds, IODE2, idot_semicirclesds;

		void read(char [1024]);	
		bool SatPos(double,double[3]);


	};


	inline x02x31::x02x31()
	{  
		
	}

	inline x02x31::~x02x31()
	{	   
	}




	class x01x06{
	//private:

	public:		
		x01x06();
		~x01x06();

		unsigned long ITOW_ms;
		long Frac_ns;
		int week;
		unsigned char GPSfix;
		unsigned char Flags;
		long ECEF_X_cm;
		long ECEF_Y_cm;
		long ECEF_Z_cm;
		unsigned long Pacc_cm;
		long ECEFVX_cms;
		long ECEFVY_cms;
		long ECEFVZ_cms;
		unsigned long SAcc_cms;
		unsigned int PDOP;				// scale 0.01
		unsigned char numSV;
		
		void read(char [1024]);

	};


	inline x01x06::x01x06()
	{    
	}

	inline x01x06::~x01x06()
	{	   
	}



	class x01x12{
	//private:

	public:		
		x01x12();
		~x01x12();


		unsigned long ITOW_ms;
		long VEL_N_cms;
		long VEL_E_cms;
		long VEL_D_cms;
		unsigned long Speed_cms;
		unsigned long GSpeed_cms;
		long Heading_deg;			//1e-5
		unsigned long Sacc_cms;
		unsigned long CAcc_deg;		// 1e-5

		
		void read(char [1024]);

	};


	inline x01x12::x01x12()
	{    
	}

	inline x01x12::~x01x12()
	{	   
	}



		class x01x04{
	//private:

	public:		
		x01x04();
		~x01x04();


		unsigned long ITOW_ms;
		unsigned int GDOP;
		unsigned int PDOP;
		unsigned int TDOP;
		unsigned int VDOP;
		unsigned int HDOP;
		unsigned int NDOP;
		unsigned int EDOP;
		
		void read(char [1024]);

	};


	inline x01x04::x01x04()
	{    
	}

	inline x01x04::~x01x04()
	{	   
	}


        class x01x02{
        //private:

        public:
                x01x02();
                ~x01x02();


                unsigned long ITOW_ms;
                long Lon_deg;
                long Lat_deg;
                long Height_mm;
                unsigned long HMSL_mm;
                unsigned long GSpeed_cms;
                unsigned long HAcc_mm;
                unsigned long VAcc_mm;		// 1e-5


                void read(char [1024]);

        };


        inline x01x02::x01x02()
        {
        }

        inline x01x02::~x01x02()
        {
        }

        class x01x03{
        //private:

        public:
                x01x03();
                ~x01x03();


                unsigned long ITOW_ms;
                unsigned char gpsFix;
                unsigned flags : 1;
                unsigned diffStat : 1;
                unsigned char res;
                unsigned long ttff;
                unsigned long msss;


                void read(char [1024]);

        };


        inline x01x03::x01x03()
        {
        }

        inline x01x03::~x01x03()
        {
        }


	class x0Bx02{
	//private:

	public:		
		x0Bx02();
		~x0Bx02();
		
		unsigned long HEALTH;
		double UTC_A1;
		double UTC_A0;
		long UTC_TOT;
		int UTC_WNT;
		int UTC_LS;
		int UTC_WNF;
		int UTC_DN;
		int UTC_LSF;
		int UTC_Spare;

		float KLOB_A0;
		float KLOB_A1;
		float KLOB_A2;
		float KLOB_A3;
		float KLOB_B0;
		float KLOB_B1;
		float KLOB_B2;
		float KLOB_B3;

		unsigned long FLAGS;

		unsigned char Mestemp[9];
		void read(char [1024]);
	};


	inline x0Bx02::x0Bx02()
	{    
	}

	inline x0Bx02::~x0Bx02()
	{	   
	}






	void signedcheck(long&);
	void signedcheck(int&);
	void signedcheck(char&);
	void signedcheck2(double& inout, double size, double scale);
}

#endif
