/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/8/2012
   @file   RamaxxLog.cpp

*/ 

#include "RamaxxLog.h"

void configureRamaxxLog(LogCollector &logger)
{
  logger.enableMatlab();
  // "%time slamx slamy slamtheta odox odoy odotheta targetdeltafront targetdeltarear halldeltafront halldeltarear servofront servorear ododist odoticksleft odoticksright"<<std::endl;
  logger.addColumn(LOG_ODOX,LOG_ODOX,false);
  logger.addColumn(LOG_ODOY,LOG_ODOY,false);
  logger.addColumn(LOG_ODOTHETA,LOG_ODOTHETA,false);
  logger.addColumn(LOG_TARGETDELTAFRONT,LOG_TARGETDELTAFRONT, false);
  logger.addColumn(LOG_TARGETDELTAREAR,LOG_TARGETDELTAREAR, false);
  logger.addColumn(LOG_TARGETSPEED,LOG_TARGETSPEED, false);
  logger.addColumn(LOG_SERVOFRONT,LOG_SERVOFRONT,false);
  logger.addColumn(LOG_SERVOREAR,LOG_SERVOREAR,false);
  logger.addColumn(LOG_SERVOSPEED,LOG_SERVOSPEED,false);
  logger.addColumn(LOG_HALLDELTAFRONT,LOG_HALLDELTAFRONT, false);
  logger.addColumn(LOG_HALLDELTAREAR,LOG_HALLDELTAREAR, false);
  logger.addColumn(LOG_SPEED,LOG_SPEED,false);
  logger.addColumn(LOG_SLAMX,LOG_SLAMX,false);
  logger.addColumn(LOG_SLAMY,LOG_SLAMY,false);
  logger.addColumn(LOG_SLAMTHETA,LOG_SLAMTHETA,false);
  logger.addColumn(LOG_PNITHETA,LOG_PNITHETA,false);
  logger.addColumn(LOG_ODODISTLEFT,LOG_ODODISTLEFT,false);
  logger.addColumn(LOG_ODODISTRIGHT,LOG_ODODISTRIGHT,false);
  logger.addColumn(LOG_ODOTICKSLEFT,LOG_ODOTICKSLEFT,false);
  logger.addColumn(LOG_ODOTICKSRIGHT,LOG_ODOTICKSRIGHT,false);
  logger.addColumn(LOG_HALLFRONT1,LOG_HALLFRONT1,false);
  logger.addColumn(LOG_HALLFRONT2,LOG_HALLFRONT2,false);
  logger.addColumn(LOG_HALLREAR1,LOG_HALLREAR1,false);
  logger.addColumn(LOG_HALLREAR2,LOG_HALLREAR2,false);
  logger.addColumn(LOG_GPS_N);
  logger.addColumn(LOG_GPS_E);
  logger.addColumn(LOG_GPS_ALT);
  logger.addColumn(LOG_GPS_FIX);
  logger.addColumn(LOG_GPS_SATS);
  logger.addColumn(LOG_IMUQ0);
  logger.addColumn(LOG_IMUQ1);
  logger.addColumn(LOG_IMUQ2);
  logger.addColumn(LOG_IMUQ3);
}
