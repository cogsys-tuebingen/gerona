#ifndef UBLOX_OBSERVATIONS_H
#define UBLOX_OBSERVATIONS_H

#include <iomanip>
#include "ubloxmsg.h"
#include "types.h"
#include "gps.h"



namespace GPS
{

  
  
  class UBXEphemeridenObservation :
        public Observation
  {
      x02x31 message;
    public:
      UBXEphemeridenObservation() {}
      UBXEphemeridenObservation ( TimeStamp _ts, x02x31 _message ) : Observation ( _ts ),message ( _message ) {}
      int getSVID() {
        return message.SVID;
      }
      bool satPos ( double, double[] );
      double getClockError_s ( double gpsTime_s );
      unsigned long getHOW() {
        return message.HOW;
      }
      static void writeLogHeader ( ostream& os );
      friend ostream& operator<< ( ostream& os, const UBXEphemeridenObservation& );
      friend istream& operator>> ( istream& is, UBXEphemeridenObservation& );
  };

  class UBXSatelliteConstellationObservation:
        public Observation
  {
      x02x20 message;
    public:
      UBXSatelliteConstellationObservation() {}
      UBXSatelliteConstellationObservation ( TimeStamp _ts, x02x20 _message ) : Observation ( _ts ),message ( _message ) {}
      UBXSatelliteConstellationObservation *clone() {
        UBXSatelliteConstellationObservation *o = new UBXSatelliteConstellationObservation ( getTimeStamp(),message );
        return o;
      }

      int getElevationForSVID ( int SVID );
      int getNumForSVID ( int SVID );
      int getNumSVs() {
        return message.NumSV;
      }
      int getSVID ( int svNum ) {
        return message.SVID[svNum];
      }
      static void writeLogHeader ( ostream& os );
      friend ostream& operator<< ( ostream& os, const UBXSatelliteConstellationObservation& );
      friend istream& operator>> ( istream& is, UBXSatelliteConstellationObservation& );

  };

  class UBXRawObservation:
        public Observation
  {

    public:
      x02x10 message;
      string receiverID; // ID of receiver
      UBXRawObservation() {}
      UBXRawObservation ( TimeStamp _ts, x02x10 _message, string _receiverID ) : Observation ( _ts ),message ( _message ), receiverID ( _receiverID ) {}

      UBXRawObservation *clone() {
        UBXRawObservation *o = new UBXRawObservation ( getTimeStamp(),message,receiverID );
        return o;
      }

      int getNumForSVID ( int svID );
      int getSignalStrength ( int svNum );
      int getNumSVs() {
        return message.NSV;
      }
      int getSVID ( int svNum ) {
        return message.SV[svNum];
      }
      double getSendTime_s ( int svNum );
      double getPSR_m ( int svNum ) {
        return message.PRMes[svNum];
      }
      void setPSR_m ( int svNum, double psr_m ) {
        message.PRMes[svNum] = psr_m;
      }
      double getCP ( int svNum ) {
        return message.CPMes[svNum];
      }
      static void writeLogHeader ( ostream& os );
      friend ostream& operator<< ( ostream& os, const UBXRawObservation& );
      friend istream& operator>> ( istream& is, UBXRawObservation& );

  };

  class UBXNavigationSolutionObservation:
        public Observation
  {
    public:
      double gk_m[3];
      double sigma_gk_m[3];
      double delta_gk_m[3];
      double delta_trans_gk_m[2];
      double ecef_m[3];

      double receiverTimeOffset;

      int nSVs;
      UBXNavigationSolutionObservation() {}
      UBXNavigationSolutionObservation ( TimeStamp _ts ) : Observation ( _ts ) {}

      static void writeLogHeader ( ostream& os );
      friend ostream& operator<< ( ostream& os, const UBXNavigationSolutionObservation& );
      friend istream& operator>> ( istream& is, UBXNavigationSolutionObservation& );
  };



  struct UBXObservation : Observation {
    UBXObservation() : Observation() {}
    UBXObservation ( TimeStamp _ts ) : Observation ( _ts ) {}
    unsigned char fix;
    double ITOW_ms;
    double	XGK;
    double	YGK;
    double	ZGK;
    double	ECEF_X;
    double	ECEF_Y;
    double	ECEF_Z;
    double heading_rad;
    unsigned long speed_cms;

    long VEL_N_cms;
    long VEL_E_cms;
    long VEL_D_cms;
    unsigned long GSpeed_cms;
    unsigned long Sacc_cms;
    unsigned long CAcc_deg;		// 1e-5

    unsigned int GDOP;
    unsigned int PDOP;
    unsigned int TDOP;
    unsigned int VDOP;
    unsigned int HDOP;
    unsigned int NDOP;
    unsigned int EDOP;



    //float eph,epv;

    void print() {
      cout<<" Fix: " << ( int ) fix << endl
      <<" X:" << XGK << " Y:" << YGK << " Z:" << ZGK << endl
      <<" Heading: "<<heading_rad<<endl;
    }
    static void writeLogHeader ( ostream& os ) {
      os<<"% GPS log"<<endl;
      os<<"% Format:"<<endl;
      os<<"% Time[sec] Time[msec] ITOW[ms] fix XGK YGK ZGK ECEF_X ECEF_Y ECEF_Z[m] heading[rad] speed[cm/s]"
      << "VEL_N[cms] VEL_E[cms] VEL_D[cms] GSpeed[cms] Sacc[cms] CAcc[deg e-5]"
      << "GDOP[e-2] PDOP[e-2] TDOP[e-2] VDOP[e-2] HDOP[e-2] NDOP[e-2] EDOP[e-2]" << endl;
    }
    friend ostream& operator<< ( ostream& os, const UBXObservation& r ) {
      os<<setprecision ( 18 );
      os<<r.ts.sec << " " << r.ts.msec << " " <<r.ITOW_ms<<" "<< ( int ) r.fix<<" "<<r.XGK<<" "<<r.YGK<<" "<<r.ZGK<<" "<<r.ECEF_X<<" "<<r.ECEF_Y<<" "<<r.ECEF_Z<<" "
      <<r.heading_rad<<" "<<r.speed_cms<<" "<<r.VEL_N_cms<<" "<<r.VEL_E_cms<<" "
      <<r.VEL_D_cms<<" "<<r.GSpeed_cms<<" "<<r.Sacc_cms<<" "<<r.CAcc_deg<<" "
      <<r.GDOP<<" "<<r.PDOP<<" "<<r.TDOP<<" "<<r.VDOP<<" "<<r.HDOP<<" "<<r.NDOP<<" "<<r.EDOP;
      return os;
    }
    friend istream& operator>> ( istream& is, UBXObservation& r ) {
      is>>r.ts.sec >> r.ts.msec >>r.ITOW_ms>>r.fix>>r.XGK>>r.YGK>>r.ZGK>>r.ECEF_X>>r.ECEF_Y>>r.ECEF_Z
      >>r.heading_rad>>r.speed_cms>>r.VEL_N_cms>>r.VEL_E_cms
      >>r.VEL_D_cms>>r.GSpeed_cms>>r.Sacc_cms>>r.CAcc_deg
      >>r.GDOP>>r.PDOP>>r.TDOP>>r.VDOP>>r.HDOP>>r.NDOP>>r.EDOP;
      return is;
    }
  };

  struct UBXIonosphereParametersObservation  : Observation {
    UBXIonosphereParametersObservation() : Observation() {}
    UBXIonosphereParametersObservation ( TimeStamp _ts ) : Observation ( _ts ) {}

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

    void print() {
      cout<<" HEALTH: " << HEALTH << endl;
    }
    static void writeLogHeader ( ostream& os ) {
      os<<"% GPS Health, UTC and ionosphere parameters log"<<endl;
      os<<"% Format:"<<endl;
      os<<"% Time[ms] UTC[s] HEALTH UTC_A1 UTC_A0 UTC_TOT UTC_WNT UTC_LS UTC_WNF UTC_DN UTC_LSF UTC_Spare "
      << "KLOB_A0 KLOB_A1 KLOB_A2 KLOB_A3 KLOB_B0 KLOB_B1 KLOB_B2 KLOB_B3 FLAGS"<< endl;
    }
    friend ostream& operator<< ( ostream& os, const UBXIonosphereParametersObservation& r ) {
      os<<setprecision ( 18 );
      os<<r.ts.sec<<" "<<r.ts.msec<<" "<<r.HEALTH<<" "<<r.UTC_A1<<" "<<r.UTC_A0<<" "<<r.UTC_TOT<<" "<<r.UTC_WNT<<" "<<r.UTC_LS<<" "<<r.UTC_WNF<<" "
      <<r.UTC_DN<<" "<<r.UTC_LSF<<" "<<r.UTC_Spare<<" "<<r.KLOB_A0<<" "
      <<r.KLOB_A1<<" "<<r.KLOB_A2<<" "<<r.KLOB_A3<<" "<<r.KLOB_B0<<" "
      <<r.KLOB_B1<<" "<<r.KLOB_B2<<" "<<r.KLOB_B3<<" "<<r.FLAGS;
      return os;
    }
    friend istream& operator>> ( istream& is, UBXIonosphereParametersObservation& r ) {
      is>>r.ts.sec >> r.ts.msec>>r.HEALTH>>r.UTC_A1>>r.UTC_A0>>r.UTC_TOT>>r.UTC_WNT>>r.UTC_LS>>r.UTC_WNF
      >>r.UTC_DN>>r.UTC_LSF>>r.UTC_Spare>>r.KLOB_A0
      >>r.KLOB_A1>>r.KLOB_A2>>r.KLOB_A3>>r.KLOB_B0
      >>r.KLOB_B1>>r.KLOB_B2>>r.KLOB_B3>>r.FLAGS;
      return is;
    }
  };
}

#endif
