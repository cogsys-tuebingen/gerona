/*
 * evaluator.cpp
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "evaluator.h"

/// COMPONENT
#include "PathRenderer.hpp"

/// SYSTEM
#include <boost/bind.hpp>

using namespace lib_path;

Evaluator::Evaluator(int w, int h)
    : map_info(w, h, 0.02f), w(w), h(h)
{
    obstacles = true;

    window = "Path Algorithm Evaluator";

    cv::namedWindow(window);

    initMap();

    start.x = 20;
    start.y = 20;

    goal.x = w / 2;
    goal.y = h / 2;
    goal.theta = M_PI / 2;

    img = cv::Mat(map_info.getHeight(), map_info.getWidth(), CV_8UC3, cv::Scalar::all(127));
}

void Evaluator::initMap()
{

    map_info.setOrigin(Point2d(0, 0));
    map_info.setResolution(1);
    map_info.setLowerThreshold(20);
    map_info.setUpperThreshold(50);

    cv::Mat orig_map(h, w, CV_8UC1, cv::Scalar::all(255));

    if(obstacles) {
        // Obstacles
        cv::rectangle(orig_map, cv::Point(80, 80), cv::Point(120, 400), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(0, 0), cv::Point(10, 10), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(300, 80), cv::Point(320, 100), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(340, 120), cv::Point(360, 140), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        // "cage"
        cv::rectangle(orig_map, cv::Point(300, 220), cv::Point(340, 240), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
        cv::rectangle(orig_map, cv::Point(300, 260), cv::Point(340, 280), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
    }

    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            uint8_t val = orig_map.at<uint8_t>(y, x);

            if(val == 0){
                /// OBSTACLE
                map_info.setValue(x, y, 255);
                assert(!map_info.isFree(x, y));
            } else {
                map_info.setValue(x, y, 0);
                assert(map_info.isFree(x, y));
            }
        }
    }
}

void Evaluator::intermission()
{
    PathRenderer<> renderer(map_info, img);
    renderer.renderMap();
    renderer.draw_arrow(start, cv::Scalar::all(-1));
    renderer.draw_arrow(goal,  cv::Scalar::all(-1));
    searchAlgorithm.visualize(img);

    cv::imshow(window.c_str(), img);
    int key = cv::waitKey(100) & 0xFF;

    if(key == 27 || cvGetWindowHandle(window.c_str()) == NULL){
        exit(0);
    }
}

void Evaluator::run()
{
    searchAlgorithm.setMap(&map_info);

    Path path = searchAlgorithm.findPath(start, goal, boost::bind(&Evaluator::intermission, this));

    PathRenderer<> renderer(map_info, img);
    renderer.renderMap();
    renderer.draw_arrow(start, cv::Scalar::all(-1));
    renderer.draw_arrow(goal,  cv::Scalar::all(-1));
    searchAlgorithm.visualize(img);
    renderer.render(path);

    while(cvGetWindowHandle(window.c_str()) != NULL) {
        if(cvGetWindowHandle(window.c_str()) != NULL) {
            cv::imshow(window.c_str(), img);
        }

        if(cvGetWindowHandle(window.c_str()) != NULL) {
            int key = cv::waitKey(100) & 0xFF;

            if(key == 27){
                break;
            }
        }
    }
}

int main(int argc, char* argv[])
{
    Evaluator eval(600, 400);
    eval.run();
}
