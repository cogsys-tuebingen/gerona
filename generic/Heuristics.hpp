/*
 * Heuristics.hpp
 *
 *  Created on: 2 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

/// PROJECT
#include <utils/LibUtil/MathHelper.h>
#include <utils/LibGeneric/Utils.hpp>

/// SYSTEM
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <fstream>

namespace lib_path
{

/**
 * @brief The HeuristicNode struct augments a class by adding more fields for managing heuristics
 */
template <class PointT>
struct HeuristicNode : public Node<PointT> {
    using Node<PointT>::distance;

    typedef PointT PointType;
    typedef HeuristicNode<PointT> NodeType;

    virtual double getTotalCost() const {
        return distance + h;
    }

    template <class AnyPoint>
    static void init(HeuristicNode<PointT> &memory, const AnyPoint& p) {
        Node<PointT>::init(memory, p);
        memory.h = 0;
    }

    template <typename V>
    static void init(HeuristicNode<PointT> &memory, V x, V y) {
        Node<PointT>::init(memory, x, y);
        memory.h = 0;
    }

    double h;
};

/**
 * @brief The HeuristicMapTraits class has type information for Maps
 */
template <class H>
struct HeuristicMapTraits {
private:
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small test4init(typeof(&Class::init));
    template <typename Class> static Big test4init(...);

    template <bool, class HH>
    struct InitIf {
        template <class Param>
        static void init(const Param& param){}
    };

    template <class HH>
    struct InitIf<true, HH> {
        template <class Param>
        static void init(const Param& param){
            HH::init(param);
        }
    };
public:
    enum { HeuristicUsesInitFunction = sizeof(test4init<H>(0)) == sizeof(Small) };

    typedef InitIf<HeuristicUsesInitFunction, H> Init;
};


/**
 * @brief The NoHeuristic struct represents no used heuristic
 */
struct NoHeuristic {
    template <class PointT>
    struct NodeHolder {
        typedef Node<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(const A*, const B*, double res) {
        /// will be optimized out
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        /// will be optimized out
    }
};

/**
 * @brief The HeuristicL2 struct represents the L2 norm (euclidean)
 */
struct HeuristicL2 {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        current->h = res * hypot(current->x - goal->x, current->y - goal->y);
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        /// will be optimized out
    }
};

/**
 * @brief The HeuristicL2OverEstimate struct represents the L2 norm (euclidean)
 */
struct HeuristicL2OverEstimate {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        current->h = res * std::pow(hypot(current->x - goal->x, current->y - goal->y), 1.2);
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        /// will be optimized out
    }
};

/**
 * @brief The HeuristicL1 struct represents the L1 norm (manhattan)
 */
struct HeuristicL1 {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        current->h = res * (std::abs(current->x - goal->x) + std::abs(current->y - goal->y));
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        /// will be optimized out
    }
};

/**
 * @brief The HeuristicLInf struct represents Loo norm (max distance)
 */
struct HeuristicLInf {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        current->h = res * std::max(std::abs(current->x - goal->x), std::abs(current->y - goal->y));
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        /// will be optimized out
    }
};

/**
 * @brief The HeuristicNonholonomicObstacles struct represents a norm that respects non-holonomic constraits
 * @note WORK IN PROGRESS, NOT YET FUNCTIONAL
 */
struct HeuristicHolonomicObstacles {
#define cost__(c,r)  cost[((int) r) * w + ((int) c)]

    struct Parameter {
        Parameter()
        {}
    };

    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        if(cost != NULL) {
            current->h = cost__(current->x, current->y);
            if(current->h != std::numeric_limits<double>::max()) {
                current->h *= res;
            }
        } else {
            current->h = 0;
        }
    }

    template <class Map, class NodeType>
    static void setMap(const Map* map, const NodeType& goal) {
        if(cost != NULL) {
            delete [] cost;
        }

        w = map->getWidth();
        h = map->getHeight();

        cost = new double[w * h];

        double HORIZ = 1.0;
        double DIAG = std::sqrt(2) * HORIZ;

        for(int row = 0; row < h; ++row) {
            for(int col = 0; col < w; ++col) {
                if(map->isFree(col, row)) {
                    cost__(col, row) = std::numeric_limits<double>::max();
                } else {
                    cost__(col, row) = INFINITY;
                }
            }
        }

        cost__(goal.x, goal.y) = 0;

        int iteration = 0;

        bool change = true;
        while(change) {
            ++iteration;
            change = false;

            for(int row = 1; row < h-1; ++row) {
                for(int col = 1; col < w-1; ++col) {
                    double& d = cost__(col, row);
                    bool free = d != INFINITY;

                    if(free) {
                        double d1 = cost__(col-1, row  ) + HORIZ;
                        double d2 = cost__(col-1, row-1) + DIAG;
                        double d3 = cost__(col,   row-1) + HORIZ;
                        double d4 = cost__(col+1, row-1) + DIAG;

                        double dmin = std::min(d1, std::min(d2, std::min(d3, d4)));

                        if(dmin < d) {
                            d = dmin;
                            change = true;
                        }
                    }
                }
            }

            for(int row = h-2; row >= 1; --row) {
                for(int col = w-2; col >= 1; --col) {
                    double& d = cost__(col, row);
                    bool free = d != INFINITY;

                    if(free) {
                        double d1 = cost__(col+1, row  ) + HORIZ;
                        double d2 = cost__(col-1, row+1) + DIAG;
                        double d3 = cost__(col,   row+1) + HORIZ;
                        double d4 = cost__(col+1, row+1) + DIAG;

                        double dmin = std::min(d1, std::min(d2, std::min(d3, d4)));

                        if(dmin < d) {
                            d = dmin;
                            change = true;
                        }
                    }
                }
            }
        }
        std::cout << "initializing heuristic took " << iteration << " iteratiions" << std::endl;
    }
    static void init(const Parameter& param) {
        std::cout << "init called" << std::endl;
    }

private:
    static double * cost;
    static unsigned w;
    static unsigned h;
#undef cost__
};
double* HeuristicHolonomicObstacles::cost = NULL;
unsigned HeuristicHolonomicObstacles::w = 0;
unsigned HeuristicHolonomicObstacles::h = 0;

/**
 * @brief The HeuristicNonHolonomicNoObstacles struct represents a norm that respects non-holonomic constraits
 * @note WORK IN PROGRESS, NOT YET FUNCTIONAL
 */
struct HeuristicNonHolonomicNoObstacles {
    struct Parameter {
        Parameter(const std::string& file)
            : file_(file)
        {}

        std::string file_;
    };

    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        instance().computeImp(current, goal, res);
    }

    static HeuristicNonHolonomicNoObstacles& instance() {
        static HeuristicNonHolonomicNoObstacles instance;
        return instance;
    }


    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        instance().resolution_map = map->getResolution();
        instance().updateResolutionFactor();
    }
    static void init(const Parameter& param) {
        instance().param = param;
        instance().read();
    }

private:
    HeuristicNonHolonomicNoObstacles()
        : param("heuristic_holo_no_obst.txt")
    {
        read();
    }

    template <class A, class B>
    void computeImp(A* current, B* goal, double res) {
        assert(angles != 0);

        double x_grid = current->x;
        double y_grid = current->y;
        double t = current->theta;

        // rotate both nodes around goal so that goal->theta = 0
        /// translation so that goal is at (0,0)
        x_grid -= goal->x;
        y_grid -= goal->y;
//        y_grid -= goal->center_y;

        /// rotation so that goal is at (0,0,0)
        double gt = -goal->theta;
        double x_grid_rot,y_grid_rot;

        x_grid_rot = x_grid * std::cos(gt) - y_grid * std::sin(gt);
        y_grid_rot = x_grid * std::sin(gt) + y_grid * std::cos(gt);

        int x_precomp = x_grid_rot / resolution_factor + dimension / 2;
        int y_precomp = y_grid_rot / resolution_factor + dimension / 2;

        t = MathHelper::AngleClamp(t+gt);
        assert(-M_PI <= t);
        assert(t < M_PI);

        // access lut with new currents state
        const double epsilon = 0.001;

        if(t < 0) {
            t += 2*M_PI;
        }
        int a = std::floor(t / (2 * M_PI + epsilon) * (angles-1));
        assert(0 <= a);
        assert(a < angles);

        if(x_precomp < 0 || x_precomp >= dimension) {
//            std::cout << "x=" << x_grid_rot * resolution_map << " is out of range +-" << (dimension / 2 * resolution_precomp) << std::endl;
            current->h = 0;
            return;
        }
        if(y_precomp < 0 || y_precomp >= dimension) {
//            std::cout << "y=" << y_grid_rot * resolution_map << " is out of range +-" << (dimension / 2 * resolution_precomp) << std::endl;
            current->h = 0;
            return;
        }

        int idx = (dimension - y_precomp) * dimension + x_precomp;

//        assert(std::abs(current->h - hypot(current->x-goal->x, current->y-goal->y)) < 0.1);

        current->h = res * costs[a].second[idx] * resolution_factor;
    }

    void updateResolutionFactor() {
        resolution_factor = resolution_precomp / resolution_map;
    }

    void read() {
        std::ifstream ifs(param.file_.c_str());

        std::cout << "reading: " << param.file_ << std::endl;
        assert(ifs.is_open());

        ifs >> dimension;
        ifs >> angles;
        ifs >> resolution_precomp;

        updateResolutionFactor();

        unsigned D = dimension * dimension;

        for(int a = 0; a < angles; ++a) {
            std::pair<double, std::vector<double> > layer(0, std::vector<double>(D));

            ifs >> layer.first;

            for(unsigned i = 0; i < D; ++i) {
                ifs >> layer.second[i];
            }

            costs.push_back(layer);
        }
    }

public:
    Parameter param;

private:
    int dimension;
    int angles;
    double resolution_map;
    double resolution_precomp;
    double resolution_factor;
    std::vector<std::pair<double, std::vector<double> > > costs;
};



/**
 * @brief The MaxHeuristic struct combines two heuristics and uses the higher value
 * @note if both H1 and H2 are admissible, then so is max(H1, H2)
 */
template <class H1, class H2>
struct MaxHeuristic {
    typedef H1 H1T;
    typedef H2 H2T;

    template <class PointT>
    struct NodeHolder {
        typedef typename H1::template NodeHolder<PointT> H1NH;
        typedef typename H1NH::NodeType T1;

        typedef typename H2::template NodeHolder<PointT> H2NH;
        typedef typename H2NH::NodeType T2;

        BOOST_STATIC_ASSERT((boost::is_same<T1, T2>::value));

        typedef T1 NodeType;
    };

    template <class A, class B>
    static void compute(A* current, const B* goal, double res) {
        H1::compute(current, goal, res);
        double h1 = current->h;

        H2::compute(current, goal, res);

        current->h = std::max(current->h, h1);
    }

    template <class Map, class NodeT>
    static void setMap(const Map* map, const NodeT& goal) {
        H1::setMap(map, goal);
        H2::setMap(map, goal);
    }
};


}

#endif // HEURISTICS_HPP
