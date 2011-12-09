#ifndef TYPES_H
#define TYPES_H
#include <map>
#include <list>
#include <vector>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <Global.h>
using namespace std;

typedef unsigned char byte;

typedef vector<int> IntVector;
typedef list<int> IntList;
typedef list<string> StringList;
typedef map<int,string> IntStringMap;

typedef unsigned long ulong;
typedef unsigned int uint32;

enum {
//  EOK = 0,  // also in Global.h
  EHW = -1, // hardware error
//  ENOINIT = -2, // not initialised ; also in Global.h
//  ESYSTEM = -3, // error in system function ; also in Global.h
  EFILENOTFOUND = -6,
  EFAILED = -7,
  EENDOFWORLD = -666666
};

struct TimeStamp
{
  TimeStamp() {
    update();
  }
  ulong sec;
  uint32 msec;
  void update () {
    struct timeval now;
    gettimeofday(&now,0);
    sec = now.tv_sec;
    msec= now.tv_usec/1000;    
  }
};


#define BEEP "\a"


#endif


