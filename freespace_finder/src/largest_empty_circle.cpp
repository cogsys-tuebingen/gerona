/*
 * largest_empty_circle.cpp
 *
 *  Created on: Sep 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

/*
 * This program can be used to test the algorithm without ROS.
 * To compile this:
 * - uncomment the last line in CMakeLists.txt
 * - uncomment the opencv2 package in manifest.xml
 */

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <time.h>

#include <vector>
#include <sstream>
#include <string>

#include "CircleFinder.h"

std::vector<Point> points;

// image size
int min_x = 0;
int max_x = 640;
int min_y = 0;
int max_y = 480;

// draw brush size
int spread = 30;

// minimum distance between two points
int min_dist = 10;
// maximum line segment to be drawn
int max_len = 1000;

bool recalc = true;
bool show_voronoi = false;
bool randomize = false;

#define MODE_NAIIVE 0
#define MODE_MAP 1
#define MODE_MAP_UNIQUE 2
#define MODE_COUNT 3

int mode = MODE_NAIIVE;
int mode_count = MODE_COUNT;

double sum[MODE_COUNT];
int sum_count[MODE_COUNT];


void reset_stats(){
  for(int m=0; m<MODE_COUNT; ++m){
    sum[m] = 0;
    sum_count[m] = 0;
  }
}


void random_point_cloud(int x, int y)
{
  Point p;
  for(int i=0; i<ceil(spread/4); ++i) {
    int dx = rand() % spread - spread/2;
    int dy = rand() % spread - spread/2;

    p.x = x + dx + min_x;
    p.y = y + dy + min_y;

    points.push_back(p);
  }
  /*
  int step = 5;
  for(int dx=-spread; dx<spread; dx+=step) {
    for(int dy=-spread; dy<spread; dy+=step) {
      p.x = x + dx + min_x;
      p.y = y + dy + min_y;

      points.push_back(p);
    }
  }*/

  reset_stats();
}

void do_randomize(int nr)
{
  for(;nr>0;nr--){
    int x = min_x + (rand() % (max_x - min_x));
    int y = min_y + (rand() % (max_y - min_y));
    random_point_cloud(x, y);
  }
}

// drawing callback for testing
void mouseCB( int event, int x, int y, int flags, void* param ){
  switch( event ){
  case CV_EVENT_LBUTTONUP: {

    if(spread == 0) {
      Point p;
      p.x = x + min_x;
      p.y = y + min_y;
      points.push_back(p);

    } else {
      random_point_cloud(x, y);
    }
    recalc = true;
  }
  break;
  }
}

int main(int argc,char **argv) {
  srand ( time(NULL) );

  // font for debug information
  CvFont font;
  double text_size = 0.4;
  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, text_size, text_size, 0, 1);

  // drawing colors
  CvScalar backgroundColor = cvScalarAll(200);
  CvScalar pointColor = cvScalar(0, 50, 200);
  CvScalar lineColor = cvScalar(200, 20, 20);
  CvScalar hideColor = cvScalarAll(180);

  // drawing dimensions
  CvSize dimensions = cvSize(max_x - min_x, max_y - min_y);

  // line parameters
  int pointRadius = 1;
  int lineThickness = 1;

  // init modes
  reset_stats();

  IplImage * debugImage = cvCreateImage(dimensions, IPL_DEPTH_8U, 3);
  bool stop = false;

  cvNamedWindow("voronoi");
  cvSetMouseCallback("voronoi", &mouseCB);

  ROS_WARN("usage:");
  ROS_WARN("W/S: increase/decrease the minimum distance between points");
  ROS_WARN("I/K: increase/decrease the maximum line length to be drawn");
  ROS_WARN("T/G: increase/decrease the draw brush size");
  ROS_WARN("C: clear the canvas");
  ROS_WARN("R: insert random points");
  ROS_WARN("M: change search method");

  CircleFinder finder(min_x, max_x, min_y, max_y, min_dist);

  std::vector<Point>::iterator point_it;
  std::vector<GraphEdge>::iterator edge_it;

  while(!stop) {
    if(randomize){
      do_randomize(50);
      randomize = false;
    }

    if(recalc){
      recalc = false;

      // clear the image
      cvRectangle(debugImage, cvPoint(0,0), cvPoint(dimensions.width, dimensions.height), backgroundColor, CV_FILLED);

      // draw the points
      for(point_it = points.begin(); point_it != points.end(); ++point_it){
        const Point &t = (*point_it);
        int x = t.x - min_x;
        int y = t.y - min_y;

        cvCircle(debugImage, cvPoint(x, y), pointRadius, pointColor, CV_FILLED, CV_AA);
      }

      bool use_naiive = (mode == MODE_NAIIVE);
      bool use_unique = (mode == MODE_MAP_UNIQUE);

      finder.run(points, use_naiive, use_unique);

      // if possible, do voronoi generation
      if(finder.valid()) {
        std::vector<Point> voronoi_points = finder.get_voronoi_points();
        std::vector<GraphEdge> voronoi_edges = finder.get_voronoi_edges();

        if(show_voronoi) {
          for(edge_it = voronoi_edges.begin(); edge_it != voronoi_edges.end(); ++edge_it){
            GraphEdge e = (*edge_it);
            cvLine(debugImage, cvPoint(e.x1, e.y1), cvPoint(e.x2, e.y2), hideColor, lineThickness, CV_AA);
            cvCircle(debugImage, cvPoint(e.sites[0].x, e.sites[0].y), pointRadius+2, cvScalar(0,255,0), 1, CV_AA);
            cvCircle(debugImage, cvPoint(e.sites[1].x, e.sites[1].y), pointRadius+2, cvScalar(0,255,0), 0.5, CV_AA);
          }
          for(point_it = voronoi_points.begin(); point_it != voronoi_points.end(); ++point_it){
            Point p = (*point_it);
            cvCircle(debugImage, cvPoint(p.x, p.y), 2, lineColor, CV_FILLED);
          }
        }

        double radius = finder.get_radius();
        Point center = finder.get_center();
        double time = finder.get_time_needed();

        sum[mode] += time;
        sum_count[mode] ++;

        if(use_naiive){
          std::cout << "naiive method:" << std::endl;
        } else {
          if(use_unique){
            std::cout << "map unique method:" << std::endl;
          } else {
            std::cout << "map method:" << std::endl;
          }
        }
        printf("freespace search on %d nodes took %.4fms, average %.4fms", points.size(), time, sum[mode] / sum_count[mode]);
        std::cout << std::endl;

        // draw free circle
        if( radius > 0 ) {
          cvCircle(debugImage,
                   cvPoint(center.x, center.y),
                   6, cvScalar(100,255,100), CV_FILLED);
          cvCircle(debugImage,
                   cvPoint(center.x, center.y),
                   radius, lineColor, 1, CV_AA);
        }
      }

      // display debug information
      std::stringstream ss;
      ss << "min dist: " << min_dist;
      cvPutText(debugImage, ss.str().c_str(), cvPoint(dimensions.width - 200, dimensions.height - 50), &font, lineColor);
      ss.str(std::string());

      ss << "max len: " << max_len;
      cvPutText(debugImage, ss.str().c_str(), cvPoint(dimensions.width - 200, dimensions.height - 30), &font, lineColor);
      ss.str(std::string());

      ss << "spread: " << spread;
      cvPutText(debugImage, ss.str().c_str(), cvPoint(dimensions.width - 200, dimensions.height - 10), &font, lineColor);
      ss.str(std::string());
    }

    // display the image
    cvShowImage("voronoi", debugImage);

    int key = cvWaitKey(13);

    if(key == 27)
      stop = true;

    else {
      // check for parameter adjustments
      switch((char) key) {
      case 'w':
      case 'W':
        min_dist++;
        recalc = true;
        break;

      case 's':
      case 'S':
        min_dist = std::max(0, min_dist - 1);
        recalc = true;
        break;

      case 'i':
      case 'I':
        max_len += 1;
        recalc = true;
        break;

      case 'k':
      case 'K':
        max_len = std::max(0, max_len - 10);
        recalc = true;
        break;

      case 'c':
      case 'C':
        points.clear();
        recalc = true;
        break;

      case 'r':
      case 'R':
        randomize = true;
        recalc = true;
        break;

      case 'v':
      case 'V':
        show_voronoi = !show_voronoi;
        recalc = true;
        break;

      case 't':
      case 'T':
        spread++;
        recalc = true;
        break;

      case 'g':
      case 'G':
        spread = std::max(0, spread - 1);
        recalc = true;
        break;

      case 'q':
      case 'Q':
        stop = true;
        break;

      case 'm':
      case 'M':
        mode = (mode + 1) % mode_count;
        recalc = true;
        break;
      }
    }
  }

  return 0;
}
