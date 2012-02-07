/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/8/2012
   @file   RamaxxLog.h


   ramaxx specific log with defined order of columns for matlab export

*/ 

#ifndef RAMAXXLOG_H
#define RAMAXXLOG_H
#include "LogCollector.h"
/* Data field names for internal logging system */
const string LOG_SLAMX=           "slamx";
const string LOG_SLAMY=           "slamy";
const string LOG_SLAMTHETA=       "slamtheta";
const string LOG_ODOX=            "odox";
const string LOG_ODOY=            "odoy";
const string LOG_ODOTHETA=        "odotheta";
const string LOG_PLAINODOX=       "plainodox";
const string LOG_PLAINODOY=       "plainodoy";
const string LOG_PLAINODOTHETA=   "plainodotheta";
const string LOG_HALLFRONT1=      "hallfront1";
const string LOG_HALLFRONT2=      "hallfront2";
const string LOG_HALLREAR1=       "hallrear1";
const string LOG_HALLREAR2=       "hallrear2";
const string LOG_ODOTICKSLEFT=    "odoticksleft";
const string LOG_ODOTICKSRIGHT=   "odoticksright";
const string LOG_ODODIST=         "ododist";
const string LOG_ODODISTRIGHT=    "ododistright";
const string LOG_ODODISTLEFT=     "ododistleft";
const string LOG_SERVOFRONT =     "servofront";
const string LOG_SERVOREAR =      "servorear";
const string LOG_SERVOSPEED =     "servospeed";
const string LOG_SPEED =          "speed";
const string LOG_TARGETSPEED =    "targetspeed";
const string LOG_TARGETDELTAFRONT="targetdeltafront";
const string LOG_TARGETDELTAREAR= "targetdeltarear";
const string LOG_HALLDELTAFRONT=  "halldeltafront";
const string LOG_HALLDELTAREAR=   "halldeltarear";
const string LOG_PNITHETA=        "pnitheta";


void configureRamaxxLog(LogCollector& logger);

#endif // RAMAXXLOG_H
