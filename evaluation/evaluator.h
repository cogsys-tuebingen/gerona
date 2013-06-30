/*
 * evaluator.h
 *
 *  Created on: Feb 01, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef EVALUATOR_H
#define EVALUATOR_H

/// COMPONENT
#include "Renderer.hpp"

/// PROJECT
#include "../common/SimpleGridMap2d.h"
#include "../generic/Algorithms.hpp"
#include "../generic/DistanceTransformation.hpp"
#include "../generic/ReedsSheppExpansion.hpp"
#include <utils/LibUtil/VideoWriter.hpp>

/// SYSTEM
#include <iostream>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

namespace lib_path
{

/**
 * @brief The DirectNeighborhood struct is used to select a holonomic grid neighborhood
 */
template <int n = 8, int distance = 1>
struct BFSNeighborhood
        : public DirectNeighborhoodBase<n, distance, BFSNeighborhood<n, distance> >
{
    typedef BFSNeighborhood<n, distance> Self;
    typedef DirectNeighborhoodBase<n, distance, Self> Parent;
    using Parent::DISTANCE;

    static int dx(int x,int i) {
        return DirectNeighborhood<n,distance>::dx(x,i);
    }
    static int dy(int y,int i) {
        return DirectNeighborhood<n,distance>::dy(y,i);
    }
    static double delta(int i) {
        return 1;
    }
};

// IGNORE END ORIENTATION
template <int n, int distance>
struct NonHolonomicNeighborhoodNoEndOrientation :
        public NonHolonomicNeighborhood<n, distance> {
    typedef NonHolonomicNeighborhood<n, distance> Parent;

    using Parent::distance_step_pixel;

    template <class NodeType>
    static bool isNearEnough(NodeType* goal, NodeType* reference) {
        return std::abs(goal->x - reference->x) <= distance_step_pixel / 2 &&
                std::abs(goal->y - reference->y) <= distance_step_pixel / 2;
    }
};

namespace search_algorithms {

static const int N = 20;
static const int M = 20 * N;
static const int INIT_STEPS = 150;

typedef BFSNeighborhood<8,1> BFSNeighbor;
typedef DirectNeighborhood<8,1> DNeighbor;
typedef NonHolonomicNeighborhood<20, 75> NHNeighbor;
typedef NonHolonomicNeighborhoodNoEndOrientation<20, 75> NHNeighborNoEndOrientation;

typedef DijkstraSearch_Debug<N,INIT_STEPS, DNeighbor> Dijkstra;
typedef DijkstraStateSearch_Debug<M,INIT_STEPS, NHNeighbor> Dijkstra4d;

typedef AStar2dTaxiSearch_Debug<N,INIT_STEPS, DNeighbor> AStarTaxi;
typedef AStar2dSearch_Debug<N,INIT_STEPS, DNeighbor> AStar;
typedef AStar2dInfSearch_Debug<N,INIT_STEPS, DNeighbor> AStarMax;

typedef AStarSearch_Debug<M,INIT_STEPS, NHNeighbor/*, ReedsSheppExpansion<250>*/ > AStarNH;
typedef AStarOverEstimateSearch_Debug<M,INIT_STEPS, NHNeighbor/*, ReedsSheppExpansion<250>*/ > AStarNHOverEstimate;
typedef AStarHybridHeuristicsSearch_Debug<M,INIT_STEPS, NHNeighbor/*, ReedsSheppExpansion<250>*/ > AStarNHHH;

DEFINE_CONCRETE_ALGORITHM(BreadthFirst,
                          Pose2d, GridMap2d, BFSNeighbor, NoExpansion, NoHeuristic, GridMapManager, PriorityQueueManager)
typedef BreadthFirstSearch_Debug<N, INIT_STEPS> BFS;

DEFINE_CONCRETE_ALGORITHM(AStarSearchNoOrientation,
                          Pose2d, GridMap2d, NHNeighborNoEndOrientation, NoExpansion, HeuristicL2, DirectionalStateSpaceManager, PriorityQueueManager)
typedef AStarSearchNoOrientationSearch_Debug<M,INIT_STEPS> AStarNHNoEndOrientation;

typedef DistanceTransformationSearch<GenericParameter<Pose2d,NoHeuristic,GridMap2d,DirectNeighborhood<8,1>,NoExpansion, QueueManager,GridMapManager,5> > DTA;
}



struct MouseCallback {
    virtual void mouseCallback(int event, int x, int y, int flags) = 0;
};


template <class Search>
class Evaluator : public MouseCallback
{
public:
    static const int SCALE = 8;

    typedef MapRenderer<SCALE, Search> SearchAlgorithm;
    typedef typename SearchAlgorithm::Heuristic Heuristic;
    typedef typename SearchAlgorithm::PathT PathT;

public:
    Evaluator(int w, int h, double resolution, int skip_images = 0);
    ~Evaluator();

    void render(const string &file);
    bool run(bool skip_init = false, bool loop = true);

    void setFocus(int x, int y);

    void mouseCallback(int event, int x, int y, int flags);

    void drawThread();

private:

    enum KEY_EVENT {
        KEY_EVENT_SKIP, KEY_EVENT_EXIT, KEY_EVENT_ANY, KEY_EVENT_NONE
    };

    enum DRAG {
        DRAG_LEFT, DRAG_RIGHT, DRAG_NONE
    };
    enum DRAG_MODE {
        SET_GOAL, SET_START, SET_FOCUS, SET_OBSTACLE, MAKE_MAZE, MAKE_FILLING_CURE
    };

private:
    void initDefaultMap();
    void invertMap(bool save_video_frames = true);
    void screenshot();
    void clearMap(bool save_video_frames = true);
    void makeMaze(int width);
    void makeSpaceFillingCurve(int width);
    void generateMap(bool save_video_frames = true);
    void saveMap();
    bool loadMap();

    template <class H>
    void init(generic::Int2Type<false>){
    }

    template <class H>
    void init(generic::Int2Type<true>){
        //H::H2T::init(HeuristicNonHolonomicNoObstacles::Parameter("heuristic_holo_no_obst.txt"));
    }

    KEY_EVENT wait();
    void drawInParallel();
    void draw(bool use_wait = true);
    void renderStartAndGoal();
    bool paintLoop(typename SearchAlgorithm::PathT *path = NULL);
    KEY_EVENT keyCallback(int key);

private:
    SimpleGridMap2d map_info;
    SearchAlgorithm algo;
    PathT * lastest_path_;

    std::string file_;
    bool show_info_;

    cv::Scalar start_color;
    cv::Scalar goal_color;

    cv::Mat img;
    cv::Mat orig_map;
    Pose2d start, goal;

    bool complete_repaint_;
    bool refresh_requested_;
    int x_;
    int y_;

    DRAG drag;
    DRAG_MODE drag_mode;

    int w;
    int h;
    cv::Point drag_start;
    cv::Rect selection;
    cv::Point2d circle_center;
    double circle_radius;
    double res;
    bool obstacles;

    boost::mutex mut;
    boost::recursive_mutex mut_paint;
    boost::condition_variable draw_cond;
    bool draw_cond_state;

    boost::thread worker;
    std::string window;

    ExternalVideoWriter video_;
    bool first_or_last_frame;
    bool save_video_frames;
    bool force_no_wait;
    int image_;

    int skip_images;
    int skipped;
};
}

#endif // EVALUATOR_H
