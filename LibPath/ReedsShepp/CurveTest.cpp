/*
 * CurveTest.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
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

/// COMPONENT
#include "Curve.h"
#include "CurveGenerator.h"
#include "CurveRenderer.h"

/// PROJECT
#include "../common/SimpleGridMap2d.h"

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>

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

using namespace lib_path;

cv::Mat img;
Pose2d start, goal;
int trace = -1;
bool ignore = false;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    start.x = x;
    start.y = h-y;
}

int main(int argc, char* argv[])
{
    cv::namedWindow("Reeds Shepp Curve Test");

    cv::setMouseCallback("Reeds Shepp Curve Test", mouseHandler);

    start.x = 10;
    start.y = 10;

    goal.x = w / 2;
    goal.y = h / 2;

    cv::Mat orig_map(h, w, CV_8UC1, cv::Scalar::all(0));

    if(obstacles) {
        // Obstacles
        cv::rectangle(orig_map, cv::Point(80, 80), cv::Point(120, 400), cv::Scalar::all(255),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(300, 80), cv::Point(320, 100), cv::Scalar::all(255),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(340, 120), cv::Point(360, 140), cv::Scalar::all(255),
                      CV_FILLED, CV_AA, 0);

        // "cage"
        cv::rectangle(orig_map, cv::Point(300, 220), cv::Point(340, 240), cv::Scalar::all(255),
                      CV_FILLED, CV_AA, 0);
        cv::rectangle(orig_map, cv::Point(300, 260), cv::Point(340, 280), cv::Scalar::all(255),
                      CV_FILLED, CV_AA, 0);
    }

    cv::cvtColor(orig_map, img, CV_GRAY2BGR);

    lib_path::CurveRenderer renderer(img);

    lib_path::CurveGenerator generator;

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
    //    L(a) | L(b)      = L(a-b)

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

    while(cvGetWindowHandle("Reeds Shepp Curve Test") != NULL) {
        SimpleGridMap2d map_info(orig_map.cols, orig_map.rows, 0.02f);
        map_info.setOrigin(Point2d(0, 0));
        map_info.setLowerThreshold(20);
        map_info.setUpperThreshold(50);

        float radius = 1.0f /*meter*/;
        float max_dist = 0.25f /*meter*/;

        generator.set_circle_radius(radius);
        generator.set_max_waypoint_distance(max_dist);

        generator.set_cost_backwards(2.0);
        generator.set_cost_curve(1.3);

        for(int y=0; y<orig_map.rows; y++) {
            for(int x=0; x<orig_map.cols; x++) {
                map_info.setValue(x, y, orig_map.at<char>(y, x));
            }
        }
//        cv::cvtColor(orig_map, img, CV_GRAY2BGR);

        generator.set_trace(trace);
        lib_path::Curve* curve = generator.find_path(start, goal, &map_info, ignore);

        // Test copy constructor
        lib_path::Curve c(*curve);

        delete curve;


        if(trace == -1) {
            if(!curve->is_valid()) {
                lib_path::Curve* curve_no_obstacles = generator.find_path(start, goal, &map_info, true);
                renderer.draw_map(&map_info);
                renderer.draw(curve_no_obstacles, cv::Scalar(30, 30, 200, 100));
                delete curve_no_obstacles;

            } else {
                renderer.draw_map(&map_info);
                renderer.draw(&c);
            }
        } else {
            renderer.draw_map(&map_info);
        }

        renderer.display_overlay(&c);


        if(cvGetWindowHandle("Reeds Shepp Curve Test") != NULL) {
            cv::imshow("Reeds Shepp Curve Test", img);
        }

        int key = 0;
        if(cvGetWindowHandle("Reeds Shepp Curve Test") != NULL) {
            key = cv::waitKey(1000 / fps);
        }

        float dt = 0.1;
        switch((char) key) {
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
            renderer.snapshot_in_new_window(&c);
            break;

        case 't':
            if(trace == -1) trace = 40;
            else trace = -1;
            generator.set_trace(trace);
            break;

        case 'i':
            ignore = !ignore;
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

    }
}
