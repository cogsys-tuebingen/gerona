/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/2/2012
   @file   path4ws_sim.cpp
   
*/ 


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <unistd.h>

#include "path4ws_simulator.h"

using namespace std;

const char* WINDOW_NAME="4ws Path Visualization";

int main(int argc, char *argv[])
{

  cvStartWindowThread();

  cvNamedWindow(WINDOW_NAME);

  Path4wsSimulator path4ws_sim;

  while (cvGetWindowHandle(WINDOW_NAME)!=NULL) {
    usleep(100000);
    path4ws_sim.step();
    cvShowImage(WINDOW_NAME,path4ws_sim.getImage());
  }

}
