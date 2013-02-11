/*
 * evaluator.h
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef EVALUATOR_H
#define EVALUATOR_H

/// COMPONENT
#include "MapRenderer.hpp"

/// PROJECT
#include "../common/SimpleGridMap2d.h"
#include "../generic/BreadthFirstSearch.hpp"

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>

namespace lib_path {

class Evaluator
{
    static const int SCALE = 2;

    struct EvalSubParameter {
        enum { SCALE = Evaluator::SCALE};
        typedef int Connector;
    };

//    typedef AStar2dSearch_Debug<8000, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, DirectNeighborhood<8,5> > AStar;
    typedef AStarSearch_Debug<0, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, NonHolonomicNeighborhood<250, 80> > AStar;

public:
    Evaluator(int w, int h);

    void run();

    void render(Path path);
private:
    void initHighResMap();
    void initLowResMap();
    void draw(cv::Scalar color);

private:
    SimpleGridMap2d map_info;
    AStar searchAlgorithm;

    cv::Mat img;
    Pose2d start, goal;

    int w;
    int h;
    bool obstacles;

    std::string window;
};
}

#endif // EVALUATOR_H
