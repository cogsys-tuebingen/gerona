/*
 * CurveTest.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

/**
 * This program can be used to test the RS-Curves.
 * It creates a window where one can use the cursor to place the start pose.
 * Using the keys 'a' and 's' one can rotate the start position
 * Using the keys 'j' and 'k' one can rotate the goal position
 * Using the keys '+' and '-' one can increase or decrease the curve radius
 * Using the key 'q' the program can be closed
 * Using the key 'p' a snapshot is taken into a new window
 */

#include <iostream>

#include "Curve.h"
#include "CurveGenerator.h"
#include "CurveRenderer.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

/**
 * Window dimension
 */
#define w 600
#define h 500

/**
 * Display some obstacles?
 */
#define obstacles true

/**
 * Frames per second
 */
#define fps 30.0

IplImage *img;
Pose2d start, goal;

void mouseHandler(int event, int x, int y, int flags, void* param) {
  start.x = x;
  start.y = h-y;
}

int main(int argc, char *argv[]) {
  cvNamedWindow("Reeds Shepp Curve Test");

  cvSetMouseCallback("Reeds Shepp Curve Test", mouseHandler);

  start.x = 10;
  start.y = 10;

  goal.x = w / 2;
  goal.y = h / 2;

  img = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

  IplImage* orig_map = cvCreateImage(cvSize(img->width, img->height),
                                     img->depth, 1);
  cvSet(orig_map, cvScalarAll(0));

  if(obstacles) {
    // Obstacles
    cvRectangle(orig_map, cvPoint(80, 80), cvPoint(120, 400), cvScalarAll(255),
                CV_FILLED, CV_AA, 0);

    cvRectangle(orig_map, cvPoint(300, 80), cvPoint(320, 100), cvScalarAll(255),
                CV_FILLED, CV_AA, 0);

    cvRectangle(orig_map, cvPoint(340, 120), cvPoint(360, 140), cvScalarAll(255),
                CV_FILLED, CV_AA, 0);

    // "cage"
    cvRectangle(orig_map, cvPoint(300, 220), cvPoint(340, 240), cvScalarAll(255),
                CV_FILLED, CV_AA, 0);
    cvRectangle(orig_map, cvPoint(300, 260), cvPoint(340, 280), cvScalarAll(255),
                CV_FILLED, CV_AA, 0);
  }

  ReedsShepp::CurveRenderer renderer(img);

  ReedsShepp::CurveGenerator generator;

  //////// CSC

  generator.parse("LSL");
  generator.parse("LSR");
  generator.parse("RSL");
  generator.parse("RSR");

  generator.parse("|LSL");
  generator.parse("|LSR");
  generator.parse("|RSL");
  generator.parse("|RSR");

  generator.parse("|L|SL");
  generator.parse("|L|SR");
  generator.parse("|R|SL");
  generator.parse("|R|SR");

  generator.parse("LS|L");
  generator.parse("LS|R");
  generator.parse("RS|L");
  generator.parse("RS|R");

  generator.parse("L|SL");
  generator.parse("L|SR");
  generator.parse("R|SL");
  generator.parse("R|SR");

  generator.parse("L|S|L");
  generator.parse("L|S|R");
  generator.parse("R|S|L");
  generator.parse("R|S|R");

  generator.parse("|LS|L");
  generator.parse("|LS|R");
  generator.parse("|RS|L");
  generator.parse("|RS|R");

  generator.parse("|L|S|L");
  generator.parse("|L|S|R");
  generator.parse("|R|S|L");
  generator.parse("|R|S|R");

  ///// CCC

  //    L(a) L(b)      = L(a+b)
  //    L(a) L(b) L(c) = L(a+b+c)

  generator.parse("LRL");
  generator.parse("RLR");

  // C|C|C
  generator.parse("L|R|L");
  generator.parse("R|L|R");
  generator.parse("|L|R|L");
  generator.parse("|R|L|R");

  // CC|C
  generator.parse("LR|L");
  generator.parse("RL|R");
  generator.parse("|LR|L");
  generator.parse("|RL|R");

  // C|CC
  generator.parse("L|RL");
  generator.parse("R|LR");
  generator.parse("|L|RL");
  generator.parse("|R|LR");

  generator.parse("LR(b)|L(b)R");
  generator.parse("|LR(b)|L(b)R");
  generator.parse("RL(b)|R(b)L");
  generator.parse("|RL(b)|R(b)L");

  generator.parse("L|R(b)L(b)|R");
  generator.parse("R|L(b)R(b)|L");
  generator.parse("|L|R(b)L(b)|R");
  generator.parse("|R|L(b)R(b)|L");

  //// ABOVE THIS LINE, IT WORKS

  // TODO: ALLOW SPECIAL QUARTER CIRCLE COMBINATIONS!!!

  MapInfo map_info;
  map_info.height = orig_map->height;
  map_info.width = orig_map->width;
  map_info.origin = Point2d(0, 0);
  map_info.resolution = 0.02f;
  map_info.threshold_max = 10;
  map_info.threshold_min = 0;

  float radius = 1.0f /*meter*/;
  float max_dist = 0.25f /*meter*/;

  generator.set_circle_radius(radius);
  generator.set_max_waypoint_distance(max_dist);

  generator.set_cost_backwards(2.0);
  generator.set_cost_curve(1.3);

  for(int y=0; y<orig_map->height; y++){
    for(int x=0; x<orig_map->width; x++){
      map_info.data.push_back(cvGet2D(orig_map, y, x).val[0]);
    }
  }

  while (true) {
    cvSet(img, cvScalarAll(0));

    ReedsShepp::Curve * c = generator.find_path(start, goal, &map_info);

    renderer.draw(c);
    renderer.display_overlay(c);

    cvShowImage("Reeds Shepp Curve Test", img);

    int key;
    key = cvWaitKey(1000 / fps);

    float dt = 0.1;
    switch ((char) key) {
    case 'a':
      start.theta -= dt;
      break;
    case 's':
      start.theta += dt;
      break;
    case 'j':
      goal.theta -= dt;
      break;
    case 'k':
      goal.theta += dt;
      break;

    case 'p':
      renderer.snapshot_in_new_window(c);
      break;

    case '+':
      radius = std::min(10.0, radius + 0.1);
      generator.set_circle_radius(radius);
      break;
    case '-':
      radius = std::max(0.0001, radius - 0.1);
      generator.set_circle_radius(radius);
      break;

    case '*':
      max_dist = std::min(2.0, max_dist + 0.01);
      generator.set_max_waypoint_distance(max_dist);
      break;
    case '_':
      max_dist = std::max(0.0001, max_dist - 0.01);
      generator.set_max_waypoint_distance(max_dist);
      break;

    case 'q':
      exit(0);

    case -1:
      break;

    default:
      std::cout << "key " << key << " is not bound" << std::endl;
    }

    delete c;

  }
  cvReleaseImage(&img);

  cvDestroyWindow("Reeds Shepp Curve Test");
}
