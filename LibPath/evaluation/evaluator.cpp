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

/// PROJECT
#include <utils/LibUtil/Stopwatch.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace lib_path;

Evaluator::Evaluator(int w, int h)
    : map_info(w, h, 0.02f), w(w), h(h)
{
    obstacles = true;

    window = "Path Algorithm Evaluator";

    cv::namedWindow(window);

    if(w > 200) {
        initHighResMap();
    } else {
        initLowResMap();
    }

    img = cv::Mat(map_info.getHeight() * SCALE, map_info.getWidth() * SCALE, CV_8UC3, cv::Scalar::all(127));
}

void Evaluator::initHighResMap()
{
    start.x = 20;
    start.y = 200;
    start.theta = M_PI / 2;

    //goal.x = w / 2 + 20;
    goal.x = w - 20;
    goal.y = h / 2 + 40;
    goal.theta = 0;//M_PI;

    map_info.setOrigin(Point2d(0, 0));
    map_info.setResolution(1);
    map_info.setLowerThreshold(20);
    map_info.setUpperThreshold(50);

    cv::Mat orig_map(h, w, CV_8UC1, cv::Scalar::all(255));

    if(obstacles) {
        // Obstacles
        cv::rectangle(orig_map, cv::Point(180, 80), cv::Point(120, 400), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(280, 80), cv::Point(220, 400), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(0, 0), cv::Point(10, 10), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(300, 80), cv::Point(320, 100), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(340, 120), cv::Point(360, 140), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        // "cage"
        cv::rectangle(orig_map, cv::Point(300, 200), cv::Point(340, 220), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
        cv::rectangle(orig_map, cv::Point(300, 260), cv::Point(340, 280), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
    }

    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            uint8_t val = orig_map.at<uint8_t>(y, x);

            if(val == 0) {
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


void Evaluator::initLowResMap()
{
    start.x = 2;
    start.y = 20;
    start.theta = 0;//M_PI / 2;

    goal.x = w - 4;
    goal.y = h - 4;
    goal.theta = M_PI / 2;

    map_info.setOrigin(Point2d(0, 0));
    map_info.setResolution(SCALE);
    map_info.setLowerThreshold(20);
    map_info.setUpperThreshold(50);

    cv::Mat orig_map(h, w, CV_8UC1, cv::Scalar::all(255));

    if(obstacles) {
        // Obstacles
        cv::rectangle(orig_map, cv::Point(18, 8), cv::Point(14, 40), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(0, 0), cv::Point(1, 1), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(30, 8), cv::Point(32, 10), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        cv::rectangle(orig_map, cv::Point(34, 12), cv::Point(36, 14), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);

        // "cage"
        cv::rectangle(orig_map, cv::Point(30, 22), cv::Point(34, 24), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
        cv::rectangle(orig_map, cv::Point(30, 26), cv::Point(34, 28), cv::Scalar::all(0),
                      CV_FILLED, CV_AA, 0);
    }

    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            uint8_t val = orig_map.at<uint8_t>(y, x);

            if(val == 0) {
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

void Evaluator::draw(cv::Scalar color)
{
    PathRenderer<SCALE, SearchAlgorithm::NodeT, SearchAlgorithm::PathT> renderer(map_info, img);
    renderer.renderMap();
    renderer.draw_arrow(start, color);
    renderer.draw_arrow(goal,  color);
    searchAlgorithm.visualize(img);

    std::stringstream ss;
    ss << "expansions: " << searchAlgorithm.noExpansions();
    cv::putText(img, ss.str(), cv::Point(40, 40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(255), 3, CV_AA);
    cv::putText(img, ss.str(), cv::Point(40, 40), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar::all(0), 1, CV_AA);

    cv::imshow(window.c_str(), img);
    int key = cv::waitKey(20);

    if(key == 27 || cvGetWindowHandle(window.c_str()) == NULL) {
        exit(0);
    }
}

void Evaluator::run()
{
    searchAlgorithm.setMap(&map_info);

    Stopwatch watch;

    SearchAlgorithm::PathT path = searchAlgorithm.findPath(start, goal, boost::bind(&Evaluator::draw, this, cv::Scalar::all(0)));

    std::cout << "path search took " << watch.usElapsed() / 1e3 << "ms" << std::endl;

    draw(cv::Scalar(0,0,255));

    PathRenderer<SCALE, SearchAlgorithm::NodeT, SearchAlgorithm::PathT> renderer(map_info, img);
    renderer.render(path);

    while(cvGetWindowHandle(window.c_str()) != NULL) {
        if(cvGetWindowHandle(window.c_str()) != NULL) {
            cv::imshow(window.c_str(), img);
        }

        if(cvGetWindowHandle(window.c_str()) != NULL) {
            int key = cv::waitKey(100) & 0xFF;

            if(key == 27) {
                break;
            }
        }
    }
}

int main(int argc, char* argv[])
{
    Evaluator eval(600, 400);
    //Evaluator eval(60, 40);
    eval.run();
}
