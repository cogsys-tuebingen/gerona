#ifndef GPS_H
#define GPS_H
#include <iomanip>
#include "types.h"
namespace GPS
{
  
class Observation {
  public:
    Observation () {}
    Observation (TimeStamp _ts) : ts(_ts) {}
    ~Observation () {};
    TimeStamp getTimeStamp() {return ts;}
  protected:
    TimeStamp ts;
};

struct GPS_Record : Observation
{
  GPS_Record() : Observation(){}
  GPS_Record(TimeStamp _ts) : Observation(_ts){}
  unsigned char fix;
  double ITOW_ms;
  double  XGK;     
  double  YGK;      
  double  ZGK;
  double  ECEF_X;
  double  ECEF_Y;
  double  ECEF_Z;
  double heading_rad;
  unsigned long speed_cms;
    
  long VEL_N_cms;
  long VEL_E_cms;
  long VEL_D_cms;
  unsigned long GSpeed_cms;
  unsigned long Sacc_cms;
  unsigned long CAcc_deg;   // 1e-5

  unsigned int GDOP;
  unsigned int PDOP;
  unsigned int TDOP;
  unsigned int VDOP;
  unsigned int HDOP;
  unsigned int NDOP;
  unsigned int EDOP;



    //float eph,epv;

  void print()
  {
    cout<<" Fix: " << (int) fix << endl
        <<" X:" << XGK << " Y:" << YGK << " Z:" << ZGK << endl
        <<" Heading: "<<heading_rad<<endl;
  }
  static void writeLogHeader(ostream& os)
  {
    os<<"% GPS log"<<endl;
    os<<"% Format:"<<endl;
    os<<"% Time[ms] UTC[s] ITOW[ms] usedSV XGK YGK ZGK ECEF_X ECEF_Y ECEF_Z[m] heading[rad] speed[cm/s]"
        << "VEL_N[cms] VEL_E[cms] VEL_D[cms] GSpeed[cms] Sacc[cms] CAcc[deg e-5]"
        << "GDOP[e-2] PDOP[e-2] TDOP[e-2] VDOP[e-2] HDOP[e-2] NDOP[e-2] EDOP[e-2]" << endl;
  }
  friend ostream& operator<<(ostream& os, const GPS_Record& r)
  {
    os<<setprecision(18);
    os<<r.ts.sec << " " << r.ts.msec <<" "<<r.ITOW_ms<<" "<<(int) r.fix<<" "<<r.XGK<<" "<<r.YGK<<" "<<r.ZGK<<" "<<r.ECEF_X<<" "<<r.ECEF_Y<<" "<<r.ECEF_Z<<" "
        <<r.heading_rad<<" "<<r.speed_cms<<" "<<r.VEL_N_cms<<" "<<r.VEL_E_cms<<" "
        <<r.VEL_D_cms<<" "<<r.GSpeed_cms<<" "<<r.Sacc_cms<<" "<<r.CAcc_deg<<" "
        <<r.GDOP<<" "<<r.PDOP<<" "<<r.TDOP<<" "<<r.VDOP<<" "<<r.HDOP<<" "<<r.NDOP<<" "<<r.EDOP;
    return os;
  }
  friend istream& operator>>(istream& is, GPS_Record& r)
  {
    is>>r.ts.sec >> r.ts.msec>>r.ITOW_ms>>r.fix>>r.XGK
        >>r.YGK>>r.ZGK>>r.ECEF_X>>r.ECEF_Y>>r.ECEF_Z
        >>r.heading_rad>>r.speed_cms>>r.VEL_N_cms>>r.VEL_E_cms
        >>r.VEL_D_cms>>r.GSpeed_cms>>r.Sacc_cms>>r.CAcc_deg
        >>r.GDOP>>r.PDOP>>r.TDOP>>r.VDOP>>r.HDOP>>r.NDOP>>r.EDOP;
    return is;
  }
};


enum {
    GLOG_SAT,
    GLOG_POS,
    GLOG_RCM,
    GLOG_NMEA,
    GLOG_END,
};

}
#endif
