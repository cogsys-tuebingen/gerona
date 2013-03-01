/*
 * evaluator.h
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef EVALUATOR_H
#define EVALUATOR_H

/// COMPONENT
#include "PathRenderer.hpp"
#include "MapRenderer.hpp"

/// PROJECT
#include "../common/SimpleGridMap2d.h"
#include "../generic/Algorithms.hpp"

/// SYSTEM
#include <iostream>
#include <opencv2/opencv.hpp>

namespace lib_path
{

class Evaluator
{
    static const int SCALE = 2;

    struct EvalSubParameter {
        enum { SCALE = Evaluator::SCALE};
    };

    typedef DirectNeighborhood<8,5> DNeighbor;
    typedef NonHolonomicNeighborhood<200, 120> NHNeighbor;

    typedef BreadthFirstSearch_Debug<0, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, DNeighbor> BFS;
    typedef BreadthFirstStateSearch_Debug<10000, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, NHNeighbor> BFS3d;

    typedef AStar2dTaxiSearch_Debug<80, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, DNeighbor> AStarTaxi;
    typedef AStar2dSearch_Debug<80, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, DNeighbor> AStar;
    typedef AStar2dInfSearch_Debug<80, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, DNeighbor> AStarMax;


    typedef AStarSearch_Debug<10000, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, NHNeighbor > AStarNH;
    typedef AStarHybridHeuristicsSearch_Debug<10000, EvalSubParameter, MapRenderer, Pose2d, GridMap2d, NHNeighbor> AStarNHHH;

    typedef AStarNH SearchAlgorithm;

    typedef PathRenderer<SCALE, SearchAlgorithm::NodeT, SearchAlgorithm::PathT, SearchAlgorithm::Heuristic> Renderer;

public:
    Evaluator(int w, int h, double resolution);

    void run();

    void render(const Path& path);

    void setFocus(int x, int y);

private:
    void initHighResMap();
    void initLowResMap();

    template <class H>
    void init(generic::Int2Type<false>){
    }

    template <class H>
    void init(generic::Int2Type<true>){
        H::init("heuristic_holo_no_obst.txt");
    }

    void draw(cv::Scalar color, bool use_wait = true);
    bool handleKey(int key);

private:
    SimpleGridMap2d map_info;
    SearchAlgorithm searchAlgorithm;

    Renderer* active_renderer;

    cv::Mat img;
    Pose2d start, goal;

    int w;
    int h;
    int focus_x;
    int focus_y;
    double res;
    bool obstacles;

    std::string window;
};
}

#endif // EVALUATOR_H
