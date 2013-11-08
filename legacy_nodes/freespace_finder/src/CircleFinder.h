/*
 * CircleFinder.h
 *
 *  Created on: Sep 19, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef CIRCLEFINDER_H
#define CIRCLEFINDER_H

#include "VoronoiDiagramGenerator.h"
#include "Timer.h"

#include <vector>

class CircleFinder
{
public:
    CircleFinder(int min_x, int max_x, int min_y, int max_y, int min_dist);

    void run(std::vector<Point> &obstacles, bool use_naiive=false, bool use_unique=false);
    std::vector<GraphEdge> get_voronoi_edges();
    std::vector<Point> get_voronoi_points();
    bool valid();

    Point get_center();
    double get_radius();
    double get_time_needed();

private:
    void run_naiive(std::vector<Point> &obstacles);
    void run_map(std::vector<Point> &obstacles, bool use_unique);

    Timer profiler;

    int m_min_x;
    int m_max_x;
    int m_min_y;
    int m_max_y;
    int m_min_dist;

    Point m_center;
    double m_radius;
    double m_time;

    std::vector<Point> m_obstacles;
    std::vector<GraphEdge> m_voronoi_edges;
    std::vector<Point> m_voronoi_nodes;
};

#endif // CIRCLEFINDER_H
