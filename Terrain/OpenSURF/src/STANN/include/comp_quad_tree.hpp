/*****************************************************************************/
/*                                                                           */
/*  Header: comp_quad_tree.hpp                                               */
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

#ifndef __STANN_COMP_QUAD_TREE__
#define __STANN_COMP_QUAD_TREE__

#include <limits>
#include <vector>

#include <zorder_lt.hpp>

/*! \file
  \brief Comtains implementation of a z-order based compressed quadtree
*/

template<typename Point>
class comp_quad_tree;

/*! Compressed quadtree node class
  \brief Node class for a compressed quadtree
  
  This class contains the functionality needed by nodes in a compressed quadtree
*/
template<typename Point>
class comp_quad_tree_node
{
  typedef typename std::vector<comp_quad_tree_node<Point> >::size_type size_type;
  typedef typename std::vector<Point>::iterator PItr;
  
public:
  /*! Constructor
    \brief Default constructor
  */
  comp_quad_tree_node()
  {
    left   = NULL;
    right  = NULL;
  };
  /*! Constructor
    \brief paramaterized constructor
    \param len Side length of the node
    \param l pointer to left child of the node
    \param r pointer to right child of the node
    \param cl upper corner of the node
    \param cu lower corner of the node
    \param lw lowest point (in z-order) contained in the node
    \param hg highest point (in z-order) contained in the node
  */
  comp_quad_tree_node(double len, 
		      comp_quad_tree_node<Point>* l, comp_quad_tree_node<Point>* r, 
		      Point cl, Point cu, 
		      PItr lw, PItr hg)
  {
    left     = l;
    right    = r;
    corner_l = cl;
    corner_u = cu;
    low      = lw;
    hig      = hg;
    length   = len;
  };
  /*! Assignment operator
    \brief Copies the contents of a node
    \param x Node to be copied
    \return Returns reference to the node
  */
  comp_quad_tree_node<Point>& operator=(const comp_quad_tree_node<Point> &x)
  {
    left     = x.left;
    right    = x.right;
    corner_l = x.corner_l;
    corner_u = x.corner_u;
    low      = x.low;
    hig      = x.hig;
    length   = x.length;
    return *this;
  };

  /*! length
    \brief side length of the node
  */
  double length;
  /*! left
    \brief left child of the node
  */
  comp_quad_tree_node<Point> *left;
  /*! right
    \brief right child of the node
  */
  comp_quad_tree_node<Point> *right;
  /*! low
    \brief random access iterator pointing to smallest point in z-order contained in node
  */
  PItr low;
  /*! hig
    \brief random access iterator pointing to largest point in z-order contained in node
  */
  PItr hig;
  /*! corner_l
    \brief lower corner of the node.  (All points in the node fall after the lower corner in z-order).
  */
  Point corner_l;
  /*! corner_u
    \brief upper corner of the node.  (All points in the node fall before the upper corner in z-order).
  */
  Point corner_u;
private:
};

/*! dist_sq_between_quad_boxes
  \brief Computes the square distance between two compressed quadtree nodes
  \param right First compressed quadtree node
  \param left Second compressed quadtree node
  \return the minimum square distance between the two nodes
*/
template <typename Point>
double dist_sq_between_quad_boxes(const comp_quad_tree_node<Point>* right, const comp_quad_tree_node<Point>* left)
{
  double dist=0;
  for(unsigned int j=0;j < Point::__DIM;++j)
    {
      if(right->corner_u[j] < left->corner_l[j])
	dist += (left->corner_l[j]-right->corner_u[j])*(left->corner_l[j]-right->corner_u[j]);
      else if(left->corner_u[j] < right->corner_l[j])
	dist += (right->corner_l[j]-left->corner_u[j])*(right->corner_l[j]-left->corner_u[j]);
    }
  return dist;
}

/*! max_dist_sq_between_quad_boxes
  \brief Computes the maximum square distance between two compressed quadtree nodes
  \param right First compressed quadtree node
  \param left Second compressed quadtree node
  \return The maximum square distance between the two nodes.
*/
template <typename Point>
double max_dist_sq_between_quad_boxes(const comp_quad_tree_node<Point> *right, const comp_quad_tree_node<Point> *left)
{
  double dist=0;
  for(int j=0;j < Point::__DIM;++j)
    {
      if((*right).corner_u[j] < (*left).corner_l[j])
	dist += ((*left).corner_u[j]-(*right).corner_l[j])*((*left).corner_u[j]-(*right).corner_l[j]);
      else if((*left).corner_u[j] < (*right).corner_l[j])
	dist += ((*right).corner_u[j]-(*left).corner_l[j])*((*right).corner_u[j]-(*left).corner_l[j]);
    }
  return dist;
}

/*! Compressed Quadtree class
  \brief Builds a compressed quadtree on a vector of z-order sorted 
  points

  This class contains the functionality needed to build a compressed 
  quadtree containing n-1 internal nodes and n leaf nodes on a z-order 
  sorted point set of size n.
*/
template<typename Point>
class comp_quad_tree
{
  typedef int size_type;
  typedef typename Point::__NumType PType;
  typedef comp_quad_tree_node<Point>* NPtr;

public:

  /*! Constructor
    \brief Creates the compressed quadtree
    The constructor creates the tree on a z-order sorted point set.  
    Once completed, every node will know it's left and right child, 
    it's side length, and the z-order range of points contained in it.  
    Construction takes O(n) time.
    \param P a vector of points (must be sorted in z-order)
  */
  comp_quad_tree(std::vector<Point> &P) : Points(P)
  {
    init();
  };
  /*! Size
    \brief Returns the number of nodes in the tree
    \return The size of the tree
  */
  typename std::vector<comp_quad_tree_node<Point> >::size_type size()
  {
    return tree.size();
  };

  /*! Bracket operator
    \brief Accesses element at index
    Accessing beyond the end of the storage vector will cause the program 
    to terminate in error
    \param k Index of element accessed in underlying vector
    \return Reference to the accessed element
  */
  comp_quad_tree_node<Point>& operator[](typename std::vector<comp_quad_tree_node<Point> >::size_type k)
  {
    if(k > tree.size())
      {
	std::cerr << "Error: reference past end of tree(" << k << ")" << std::endl;
	exit(1);
      }
    return tree[k];
  };

  /*! tree
    \brief Underlying storage for the compressed quadtree
    The nodes of the tree are stored relative to their z-order.  The first n-1 nodes are 
    internal nodes defined by the minimum quadtree box enclosing two points in z-order.  Therefore,
    the first element is the quadtree box defined by point 0 and point 1 in z-order.  The last n nodes are leaves defined by a single point, also in z-order
  */
  std::vector<comp_quad_tree_node<Point> > tree;
  /*! root
    \brief the root node of the tree
    The root node is the largest quadtree box in the tree.  All other nodes in the tree have the root for an ancestor
  */
  size_type root;

private:
  

  void init() 
  {
    tree.resize((Points.size()*2));
    Point max,min;
    for(unsigned int i=0;i < Point::__DIM;++i)
      {
	max[i] = (std::numeric_limits<typename Point::__NumType>::max)();
	min[i] = -(std::numeric_limits<typename Point::__NumType>::max)();
      }

    tree[Points.size()] = comp_quad_tree_node<Point>(0,NULL,NULL,
						     Points[0], Points[0], 
						     Points.begin(),Points.begin()+1);
    
    tree[0] = comp_quad_tree_node<Point>((std::numeric_limits<
					  typename Point::__NumType>::max)(),
					 NULL,&tree[Points.size()], 
					 max, min,Points.begin(),
					 Points.end());
    root=0;
    k=0;
    for(typename std::vector<Point>::size_type i=1;i < Points.size();++i)
      {
	double length;
	Point lcorner, ucorner;
	length = lt.quad_box_length(Points[i-1], Points[i]);
	length = length*length*Point::__DIM;
	lt.min_quad_box(Points[i-1], Points[i], lcorner, ucorner);

	k=(size_type) (i-1);
	while((length > tree[k].length) && (k>0))
	  {
	    k=k-1;
	  }
	tree[i+Points.size()] = comp_quad_tree_node<Point>(0, 
							   NULL,NULL,
							   Points[i], Points[i], 
							   Points.begin()+i,Points.begin()+(i+1));
	
	tree[i] = comp_quad_tree_node<Point>(length, tree[k].right,
					     &tree[i+Points.size()],
					     lcorner, ucorner, 
					     Points.begin()+k,tree[k].hig);
	
	NPtr j = &tree[k];
	while(j->right->right !=NULL)
	  {
	    j = j->right;
	    j->hig = Points.begin()+i;
	  }
	tree[k].right = &tree[i];
      }
  };
  size_type k;
  std::vector<Point> &Points;
  zorder_lt<Point> lt;
  
};
#endif
