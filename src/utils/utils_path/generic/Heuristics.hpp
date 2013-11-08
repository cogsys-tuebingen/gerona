/*
 * Heuristics.hpp
 *
 *  Created on: 2 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

/// PROJECT
#include <utils_general/MathHelper.h>

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

    static void init(HeuristicNode<PointT> &memory, int x, int y) {
        Node<PointT>::init(memory, x, y);
        memory.h = 0;
    }

    double h;
};



/**
 * @brief The NodeTraits class has type information for Nodes
 */
template <typename Any>
class NodeTraits
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small test(typeof(&Class::h)) ;
    template <typename Class> static Big test(...);


public:
    enum { HasHeuristicField = sizeof(test<Any>(0)) == sizeof(Small) };
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

    template <typename Class> static Small test(typeof(&Class::setMapResolution)) ;
    template <typename Class> static Big test(...);

    template <bool, class HH>
    struct If {
        static void init(const std::string& param){}
        static void setMapRes(double res){}
    };

    template <class HH>
    struct If<true, HH> {
        static void init(const std::string& param){
            HH::init(param);
        }
        static void setMapRes(double res){
            HH::setMapResolution(res);
        }
    };
public:
    enum { HeuristicUsesMapResolution = sizeof(test<H>(0)) == sizeof(Small) };
    typedef If<HeuristicUsesMapResolution, H> Do;
};


/**
 * @brief The NoHeuristic struct represents no used heuristic
 */
struct NoHeuristic {
    template <class PointT>
    struct NodeHolder {
        typedef Node<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(const NodeType*, const NodeType*) {
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

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = hypot(current->x - goal->x, current->y - goal->y);
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

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = std::abs(current->x - goal->x) + std::abs(current->y - goal->y);
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

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = std::max(std::abs(current->x - goal->x), std::abs(current->y - goal->y));
    }
};

/**
 * @brief The HeuristicHolonomicNoObstacles struct represents a norm that respects non-holonomic constraits
 * @note WORK IN PROGRESS, NOT YET FUNCTIONAL
 */
struct HeuristicHolonomicNoObstacles {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        instance().computeImp(current, goal);
    }

    static HeuristicHolonomicNoObstacles& instance() {
        static HeuristicHolonomicNoObstacles instance;
        return instance;
    }

    static void setMapResolution(double res) {
        instance().resolution_map = res;
        instance().updateResolutionFactor();
    }
    static void init(const std::string& param) {
        instance().file_ = param;
        instance().read(param);
    }

private:
    HeuristicHolonomicNoObstacles() {
        file_ = "heuristic_holo_no_obst.txt";
    }

    template <class NodeType>
    void computeImp(NodeType* current, NodeType* goal) {
        double x_grid = current->center_x;
        double y_grid = current->center_y;
        double t = current->theta;

        // rotate both nodes around goal so that goal->theta = 0
        /// translation so that goal is at (0,0)
        x_grid -= goal->center_x;
        y_grid -= goal->center_y;

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
        int a = std::floor(t / (2 * M_PI + epsilon) * angles);
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

        current->h = costs[a].second[idx];//* resolution_factor;
    }

    void updateResolutionFactor() {
        resolution_factor = resolution_precomp / resolution_map;
    }

    void read(const std::string& file) {
        std::ifstream ifs(file.c_str());

        std::cout << "reading: " << file << std::endl;
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

private:
    std::string file_;
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

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        H1::compute(current, goal);
        double h1 = current->h;

        H2::compute(current, goal);

        current->h = std::max(current->h, h1);
    }

    static void setMapResolution(double res) {
        HeuristicMapTraits<H1>::Do::setMapRes(res);
        HeuristicMapTraits<H1>::Do::setMapRes(res);
    }

    static void init(const std::string& param) {
        HeuristicMapTraits<H1>::Do::init(param);
        HeuristicMapTraits<H2>::Do::init(param);
    }

};


}

#endif // HEURISTICS_HPP
