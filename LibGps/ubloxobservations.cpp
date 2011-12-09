#include <iomanip>
#include "ubloxobservations.h"

namespace GPS
{
	bool UBXEphemeridenObservation::satPos(double gpsTime, double resultECEF_m[])
	{
		return message.SatPos(gpsTime, resultECEF_m);
	}


	double UBXEphemeridenObservation::getClockError_s(double gpsTime_s)
	{
		double delta_t_s = gpsTime_s - message.toe_s;
		return message.Clockdrift_sds * delta_t_s + message.Clockbias_s;
	}


	void UBXEphemeridenObservation::writeLogHeader(ostream& os)
	{
		os<<"% GPS Ephemeriden"<<endl;
		os<<"% Format:"<<endl;
		os<<"% Time[ms] UTC[s] SVID HOW week toc_s Clockdriftrate[sds2] Clockdrift[sds] Clockbias[s] "
			  <<"IODE1 CRS[m] deltan[semi_circlesdsee] M0[semicircles] Cuc[rad] "
			  <<"e Cus[rad] A[sqr_m] toe[s] Cic[rad] "
			  <<"Omega0[semicircles] Cis[rad] io[semicircles] Crc[m] omega[semicircles] "
			  <<"omegadot[semicirclesds] IODE2 idot[semicirclesds]" <<endl;
	}
	ostream& operator<<(ostream& os, const UBXEphemeridenObservation& obs)
	{
		os<<setprecision(18);
		os<<obs.ts.sec<<" "<<obs.ts.msec<<" "<<obs.message.SVID<<" "<<obs.message.HOW<<" "<<obs.message.week<<" "<<obs.message.toc_s<<" "
		  <<obs.message.Clockdriftrate_sds2<<" "<<obs.message.Clockdrift_sds<<" "<<obs.message.Clockbias_s<<" "<<obs.message.IODE1<<" "<<obs.message.CRS_m<<" "
		  <<obs.message.deltan_semi_circlesdsee<<" "<<obs.message.M0_semicircles<<" "<<obs.message.Cuc_rad<<" "<<obs.message.e_<<" "<<obs.message.Cus_rad<<" "
		  <<obs.message.A_sqr_m<<" "<<obs.message.toe_s<<" "<<obs.message.Cic_rad<<" "<<obs.message.Omega0_semicircles<<" "<<obs.message.Cis_rad<<" "
		  <<obs.message.io_semicircles<<" "<<obs.message.Crc_m<<" "<<obs.message.omega_semicircles<<" "<<obs.message.omegadot_semicirclesds<<" "<<obs.message.IODE2<<" "
		  <<obs.message.idot_semicirclesds;
		return os;
	}
	istream& operator>>(istream& is, UBXEphemeridenObservation& obs)
	{
		is>>obs.ts.sec>>obs.ts.msec>>obs.message.SVID>>obs.message.HOW>>obs.message.week>>obs.message.toc_s
			  >>obs.message.Clockdriftrate_sds2>>obs.message.Clockdrift_sds>>obs.message.Clockbias_s>>obs.message.IODE1>>obs.message.CRS_m	
			  >>obs.message.deltan_semi_circlesdsee>>obs.message.M0_semicircles>>obs.message.Cuc_rad>>obs.message.e_>>obs.message.Cus_rad
			  >>obs.message.A_sqr_m>>obs.message.toe_s>>obs.message.Cic_rad>>obs.message.Omega0_semicircles>>obs.message.Cis_rad
			  >>obs.message.io_semicircles>>obs.message.Crc_m>>obs.message.omega_semicircles>>obs.message.omegadot_semicirclesds>>obs.message.IODE2
			  >>obs.message.idot_semicirclesds;	
		return is;
	}

	int UBXSatelliteConstellationObservation::getElevationForSVID(int SVID)
	{
		int svNum = getNumForSVID(SVID);
		return message.Elev[svNum];
	}

	int UBXSatelliteConstellationObservation::getNumForSVID(int SVID)
	{
		for(int svNum = 0 ; svNum < getNumSVs() ; svNum++)
		{
			if(getSVID(svNum)==SVID)
			{
				return svNum;
			}
		}
		return -1;
	}

	void UBXSatelliteConstellationObservation::writeLogHeader(ostream& os)
	{
		os<<"% Satellite Constellation file"<<endl;
		os<<"% Format:"<<endl;
		os<<"% Time[ms] UTC[s] NumSV NumVis Week ITOW [ms] (SV_ID Flag AZIM [DEG] ELEV [DEG] AGE[days])1 ...NumSV"<<endl;
		os<<"% See Ublox documentation of 0x02x20 message for details"<<endl;
	}

	ostream& operator<<(ostream& os, const UBXSatelliteConstellationObservation& obs)
	{
		os<<obs.ts.sec<<" "<<obs.ts.msec<<" "<<obs.message.NumSV<<" "<<obs.message.NumVis<<" "<<obs.message.Week<<" "<<obs.message.ITOW_ms;
		for(int iSV = 0 ; iSV<obs.message.NumSV ; iSV++)
		{
			os<<" "<<(int)obs.message.SVID[iSV]<<" "<<(int)obs.message.SVFlag[iSV]
			<<" "<<obs.message.Azim[iSV]<<" "<<(int)obs.message.Elev[iSV]<<" "<<(int)obs.message.Age[iSV];
		}
		return os;
	}
	istream& operator>>(istream& is, UBXSatelliteConstellationObservation& obs)
	{
		is>>obs.ts.sec>>obs.ts.msec>>obs.message.NumSV>>obs.message.NumVis>>obs.message.Week>>obs.message.ITOW_ms;
		for(int iSV = 0 ; iSV<obs.message.NumSV ; iSV++)
		{
			is>>obs.message.SVID[iSV]>>obs.message.SVFlag[iSV]
				>>obs.message.Azim[iSV]>>obs.message.Elev[iSV]>>obs.message.Age[iSV];
		}
		return is;
	}

	double UBXRawObservation::getSendTime_s(int svNum)
	{
		double sendTime = (double)message.ITOW_ms/1000  - message.PRMes[svNum]/299792458;
		return sendTime;
	}

	int UBXRawObservation::getNumForSVID(int svID)
	{
		for(int svNum = 0 ; svNum<getNumSVs() ; svNum++)
		{
			if(getSVID(svNum)==svID)
			{
				return svNum;
			}
		}
		return -1;
	}
		
	int UBXRawObservation::getSignalStrength(int svNum)
	{
		return message.CNO[svNum];
	}

	void UBXRawObservation::writeLogHeader(ostream& os)
	{
		os<<"% UBX RAW Data"<<endl;
		os<<"% Format:"<<endl;
		os<<"% Time[ms] UTC[s] ITOW[ms] Week NumSV"			
			<< " (SV CPMes[cycles] PRMes[m] DOMes[Hz] MesQI[1-10] CNO[dbHz] LLI)1..NumSV"<<endl;
	}
	ostream& operator<<(ostream& os, const UBXRawObservation& obs)
	{
		os<<setprecision(18);
		os<<obs.ts.sec<<" "<<obs.ts.msec<<" "<<obs.message.ITOW_ms<<" "<< obs.message.Week <<" "<< (int)obs.message.NSV;
		for(int iSV = 0 ; iSV<obs.message.NSV ; iSV++)
		{
			os<<" "<<(int)obs.message.SV[iSV]<<" "
				<<obs.message.CPMes[iSV]<<" "
				<<obs.message.PRMes[iSV]<<" "
				<<obs.message.DOMes[iSV]<<" "
				<<(int)obs.message.MesQI[iSV]<<" "
				<<(int)obs.message.CNO[iSV]<<" "
				<<(int)obs.message.LLI[iSV];
			
		}
		for(int iDummy = 0 ; iDummy<19-obs.message.NSV ; iDummy++)
		{
			os<<" -1 0 0 0 0 0 0";
		}
		return os;
	}
	istream& operator>>(istream& is, UBXRawObservation& obs)
	{
		int tmp;	
		is>>obs.ts.sec>>obs.ts.msec>>obs.message.ITOW_ms>>obs.message.Week;
		is>>tmp;
		obs.message.NSV = (unsigned char)tmp;
		for(int iSV = 0 ; iSV<19 ; iSV++)
		{
			is>>tmp;
			obs.message.SV[iSV] = (unsigned char)tmp;
			is>>obs.message.CPMes[iSV]>>obs.message.PRMes[iSV]>>obs.message.DOMes[iSV];
			is>>tmp;
			obs.message.MesQI[iSV] = (char)tmp;
			is>>tmp;
			obs.message.CNO[iSV] = (char)tmp;
			is>>tmp;
			obs.message.LLI[iSV] = (unsigned char)tmp;
		}
		return is;
	}

	void UBXNavigationSolutionObservation::writeLogHeader(ostream& os)
	{
		os<<"% Homebrewn Navigation Solution"<<endl;
		os<<"% Time[ms] UTC[s] nSVs gk_R [m] gk_H [m] gk_alt[m] delta_gk_r[m] delta_gk_h[m] delta_gk_alt[m] sigmaR [m] sigmaH [m] sigmaAlt [m] ecef_x[m] ecef_y[m] ecef_z[m] delta_trans_gk_r[m] delta_trans_gk_h[m] receiverTimeOffset[m]"<<endl;
	}

	ostream& operator<<(ostream& os, const UBXNavigationSolutionObservation& obs)
	{
		os<<setprecision(18);
		os<<obs.ts.sec<<" "<<obs.ts.msec<<" "<<obs.nSVs<<" "<<obs.gk_m[0]<<" "<<obs.gk_m[1]<<" "<<obs.gk_m[2]<<" "<<
			obs.delta_gk_m[0]<<" "<<obs.delta_gk_m[1]<<" "<<obs.delta_gk_m[2]<<" "<<
			obs.sigma_gk_m[0]<<" "<<obs.sigma_gk_m[1]<<" "<<obs.sigma_gk_m[2]<<" "<<
			obs.ecef_m[0]<<" "<<obs.ecef_m[1]<<" "<<obs.ecef_m[2]<<" "<<obs.delta_trans_gk_m[0]<<" "<<obs.delta_trans_gk_m[1]
			<<" "<<obs.receiverTimeOffset;
		return os;
	}
	istream& operator>>(istream& is, UBXNavigationSolutionObservation& obs)
	{
		is>>obs.ts.sec>>obs.ts.msec>>obs.nSVs>>obs.gk_m[0]>>obs.gk_m[1]>>obs.gk_m[2]>>
			obs.delta_gk_m[0]>>obs.delta_gk_m[0]>>obs.delta_gk_m[0]>>
			obs.sigma_gk_m[0]>>obs.sigma_gk_m[1]>>obs.sigma_gk_m[2]>>
			obs.ecef_m[0]>>obs.ecef_m[1]>>obs.ecef_m[2]>>obs.delta_trans_gk_m[0]>>obs.delta_trans_gk_m[1]>>obs.receiverTimeOffset;
		return is;
	}
}
