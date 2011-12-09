/*****************************************************************************/
/*                                                                           */
/*  Header: gmst.hpp                                                         */
/*                                                                           */
/*  Accompanies STANN Version 0.71 B                                         */
/*  Dec 07, 2009                                                             */
/*                                                                           */
/*  Copyright 2007, 2008                                                     */
/*  Michael Connor and Piyush Kumar                                          */
/*  Florida State University                                                 */
/*  Tallahassee FL, 32306-4532                                               */
/*                                                                           */
/*****************************************************************************/

#ifndef __STANN_GMST__
#define __STANN_GMST__

#include <algorithm>
#include <vector>

#include <filter_kruskal.hpp>
#include <int_partition.hpp>
#include <pwspd.hpp>
#include <rand.hpp>
#include <zorder_lt.hpp>

/*! \file
  \brief Contains wrapper function for Geometric Minimum Spanning tree implementation
*/

/*! Geometric minimum spanning tree function
 \brief Finds the geometric minimum spanning tree of a point set
 \param points set of reviver::dpoints
 \param mst set of stl::pair<,> which stores the output mst
*/
template<typename Point>
void gmst(std::vector<Point> &points, std::vector<std::pair<typename std::vector<Point>::size_type,  typename std::vector<Point>::size_type> > &mst)
{
  zorder_lt<Point> lt;
  std::vector<WSP<Point> > wsp;
  int_partition UF;
  int num_threads=1;
  
#ifdef _OPENMP
  num_threads = omp_get_max_threads();
#endif
  sort(points.begin(), points.end(), lt);
  pwspd<Point> A(points, 1.0);
  A.run(wsp, num_threads);
  UF.initialize(points.size());
  FilterKruskal<Point> B(A, UF, mst);
  B.run(wsp.begin(), wsp.end(), 0);
}

template<typename Point>
class edgePred
{
  typedef typename std::vector<Point>::size_type size_type;
  typedef std::pair<size_type, size_type> Edge;
public:
  std::vector<Point> &points;
  
  edgePred(std::vector<Point> &p): points(p)
  {
  };

  bool operator()(const Edge a, const Edge b)
  {
    double da, db;
    
    da = points[a.first].sqr_dist(points[a.second]);
    db = points[b.first].sqr_dist(points[b.second]);
    return (da < db);
  }
};

/*! Brute force GMST function
  \brief Finds the GMST of a point set using a brute force implementation of Kruskal's algorithm.
  \param points set of reviver::dpoints
  \param mst set of stl::pair<,> which stores the output mst
*/
template<typename Point>
void bfgmst(std::vector<Point> &points, std::vector<std::pair<typename std::vector<Point>::size_type,  typename std::vector<Point>::size_type> > &mst)
{
  typedef typename std::vector<Point>::size_type size_type;
  typedef std::pair<size_type, size_type> Edge;

  std::vector<Edge> edges;

  for(int i=0;i < (int) points.size();++i)
    {
      for(int j=0;j < (int) points.size();++j)
	{
	  if(i!=j)
	    edges.push_back(Edge(i,j));
	}
    }
  edgePred<Point> pred(points);
  sort(edges.begin(), edges.end(), pred);

  int_partition UF;
  UF.initialize(points.size());

  for(int i=0;i < (int) edges.size();++i)
    {
      if(UF.find(edges[i].first) != UF.find(edges[i].second))
	{
	  mst.push_back(edges[i]);
	  UF.union_blocks(edges[i].first, edges[i].second);
	}
    }
}
#endif
