/*
 * evaluator.h
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef EVALUATOR_H
#define EVALUATOR_H

/// PROJECT
#include "../common/SimpleGridMap2d.h"
#include "../generic/BreadthFirstSearch.hpp"

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>

namespace lib_path {

class Evaluator
{
public:
    Evaluator(int w, int h);

    void run();

    void render(Path path);
private:
    void initMap();
    void intermission();

private:
    SimpleGridMap2d map_info;
    AStarSearch_Debug<50, Pose2d, GridMap2d, 8> searchAlgorithm;

    cv::Mat img;
    Pose2d start, goal;

    int w;
    int h;
    bool obstacles;

    std::string window;
};
}

#endif // EVALUATOR_H
