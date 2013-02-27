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

/// SYSTEM
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <fstream>

namespace lib_path
{

template <class PointT>
struct HeuristicNode : public Node<PointT> {
    using Node<PointT>::distance;

    typedef PointT PointType;
    typedef HeuristicNode<PointT> NodeType;

    virtual double getTotalCost() const {
        return distance + h;
    }

    static void init(HeuristicNode<PointT> &memory, int x, int y) {
        Node<PointT>::init(memory, x, y);
        memory.h = 0;
    }

    double h;
};



template <typename Any>
class HasHeuristicField
{
    typedef char Small;
    class Big
    {
        char dummy[2];
    };

    template <typename Class> static Small test(typeof(&Class::h)) ;
    template <typename Class> static Big test(...);


public:
    enum { value = sizeof(test<Any>(0)) == sizeof(Small) };
};




struct NoHeuristic {
    template <class PointT>
    struct NodeHolder {
        typedef Node<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(const NodeType*, const NodeType*) {
    }
    static void setMapResolution(double res) {}
    static void init(const std::string& param) {}
};

struct HeuristicDistToGoal {
    template <class PointT>
    struct NodeHolder {
        typedef HeuristicNode<PointT> NodeType;
    };

    template <class NodeType>
    static void compute(NodeType* current, NodeType* goal) {
        current->h = hypot(current->x - goal->x, current->y - goal->y);
    }
    static void setMapResolution(double res) {}
    static void init(const std::string& param) {}
};

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

        if(current->h > h1) {
//            std::cout << "using heuristic 2" << current->h << " > " << h1 << std::endl;
        }

        current->h = std::max(current->h, h1);
    }
    static void setMapResolution(double res) {
        H1::setMapResolution(res);
        H2::setMapResolution(res);
    }


    static void init(const std::string& param) {
        H1::init(param);
        H2::init(param);
    }
};


}

#endif // HEURISTICS_HPP
