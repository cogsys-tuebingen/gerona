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
#include <pcl/visualization/pcl_visualizer.h>

#include "path4ws_simulator.h"

using namespace std;

const char* WINDOW_NAME="4ws Path Visualization";


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
      *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown()) {
    if (event.getKeySym () == "q") {
      exit(0);
    }
  }
}




int main(int argc, char *argv[])
{
  int vargc=3;
  //[Clipping Range / Focal Point / Position / ViewUp / Distance / Window Size / Window Pos
  char *vargv[3];
  vargv[0]="visualizer";
  vargv[1]="-cam";
  vargv[2]="0.0,1000.0/0.2,0.0,1.5/-5.0,0,3.0/0.35,0.0,1.0/800,600/0,0";
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (argc,argv,
                                                                                                      "3D Viewer");
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->updateCamera();


  Path4wsSimulator path4ws_sim;
  path4ws_sim.step();
  viewer->addPointCloud(path4ws_sim.getCloud());

  while (true) {
    sleep(1);
  }


}
