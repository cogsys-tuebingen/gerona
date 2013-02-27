/*
 * KMeans.hpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef KMEANS_H
#define KMEANS_H

/// PROJECT
#include "Cluster.hpp"
#include "Vector.hpp"

#include <utils/LibGeneric/Intermission.hpp>

/// SYSTEM
#include <assert.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <boost/function.hpp>

namespace lib_clustering {

/**
 * @brief The ClusteringParameter struct eases the handling of KMeansImps parameters
 */
template <unsigned Dimensions,
         template <class, class, typename> class InitializationMethod,
         template <unsigned> class Distance,
         template <class, typename> class DataType,
         typename InputDataType,
         typename UserDataType,
         typename IndexDataType,
         typename WeightDataType>
struct ClusteringParameter
{
    enum { Dimension = Dimensions };

    typedef IndexDataType IndexDataTypeT;
    typedef WeightDataType WeightDataTypeT;

    typedef InputDataType InputDataTypeT;
    typedef UserDataType UserDataTypeT;

    typedef VectorImp<Dimension, IndexDataTypeT, WeightDataTypeT, UserDataTypeT> VectorT;
    typedef VectorT CentroidT;
    typedef Cluster<VectorT, CentroidT, Distance<Dimension> > ClusterT;
    typedef DataType<ClusterT, InputDataType> DataT;
    typedef Distance<Dimension> DistanceT;

    typedef InitializationMethod<DataT, ClusterT, InputDataType> InitializationMethodPolicy;
};

/**
 * @brief The KMeansImp struct implements k-means clustering
 */
template <class ClusteringParameter>
struct ClusteringAlgorithm : public ClusteringParameter::InitializationMethodPolicy {
    enum { Dimension = ClusteringParameter::Dimension };

    typedef typename ClusteringParameter::VectorT VectorT;
    typedef typename ClusteringParameter::ClusterT ClusterT;
    typedef typename ClusteringParameter::DataT DataT;
    typedef typename ClusteringParameter::IndexDataTypeT IndexDataTypeT;
    typedef typename ClusteringParameter::WeightDataTypeT WeightDataTypeT;
    typedef typename ClusteringParameter::InputDataTypeT InputDataTypeT;
    typedef typename ClusteringParameter::DistanceT Distance;

    typedef std::pair<IndexDataTypeT, IndexDataTypeT> LimitPair;
    typedef std::vector<LimitPair> LimitPairList;

    typedef typename ClusteringParameter::InitializationMethodPolicy InitializationMethodPolicy;

    using InitializationMethodPolicy::data;
    using InitializationMethodPolicy::clusters;
    using InitializationMethodPolicy::expectation;
    using InitializationMethodPolicy::init;

    /**
     * @brief KMeansImp Constructor
     * @param K no of clusters
     */
    ClusteringAlgorithm(int K)
        : K(K) {
        assert(K > 0);
    }

    /**
     * @brief setK
     * @param K
     */
    void setK(int K) {
        assert(K > 0);
        this->K = K;
    }

    /**
     * @brief find computes the clusters
     * @param input all datapoints
     * @param limits pair of limit values for each dimension
     * @param result output vector of found clusters
     * @param seed [optional] seed for the RNG
     */
    void find(const InputDataTypeT& input,
              const LimitPairList& limits,
              std::vector<ClusterT>& result,
              long seed = 1337) {

        seed_ = seed;
        findImp<generic::NoIntermission>(input, limits, result, boost::function<void()>());
    }



    /**
     * @brief find computes the clusters
     * @param input all datapoints
     * @param limits pair of limit values for each dimension
     * @param result output vector of found clusters
     * @param intermission callback that is called between each iteration
     * @param seed [optional] seed for the RNG
     */
    void find(const InputDataTypeT& input,
              const LimitPairList& limits,
              std::vector<ClusterT>& result,
              boost::function<void()> intermission,
              long seed = 1337) {

        seed_ = seed;
        findImp<generic::CallbackIntermission<1> >(input, limits, result, intermission);
    }

private:
    template <class Intermission>
    void findImp(const InputDataTypeT& input,
                 const LimitPairList& limits,
                 std::vector<ClusterT>& result,
                 boost::function<void()> intermission) {

        DataT::prepare(input, limits);

        if(data.size() <= 1){
            return;
        }

        initialize(input, limits);

        int maxIter = 50;
        for(int i = 0; i < maxIter; ++i) {
            Intermission::callAndCopy(intermission, clusters, result);

            expectation();
            if(maximization() == ClusterT::NO_CHANGE) {
                break;
            }
        }

        result.clear();
        result = clusters;
    }

    typename ClusterT::MoveResult maximization() {
        typename ClusterT::MoveResult result = ClusterT::NO_CHANGE;

        for(typename std::vector<ClusterT>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
            if(cluster->recompute() == ClusterT::CHANGE) {
                result = ClusterT::CHANGE;
            }
        }

        return result;
    }

    void initialize(const InputDataTypeT& input, const LimitPairList& limits) {
        srand(seed_);

        init(K, limits);
    }

private:
    unsigned K;
    long seed_;
};

}

#endif // KMEANS_H
