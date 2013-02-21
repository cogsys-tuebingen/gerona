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
 * @brief The KMeansParameter struct eases the handling of KMeansImps parameters
 */
template <unsigned Dimensions,
         template <class, class, typename> class InitializationMethod,
         template <unsigned> class Distance,
         template <class, typename> class DataType,
         typename InputDataType,
         typename UserDataType,
         typename IndexDataType,
         typename WeightDataType>
struct KMeansParameter
{
    enum { Dimension = Dimensions };

    typedef IndexDataType IndexDataTypeT;
    typedef WeightDataType WeightDataTypeT;

    typedef InputDataType InputDataTypeT;
    typedef UserDataType UserDataTypeT;

    typedef VectorImp<Dimension, IndexDataTypeT, WeightDataTypeT, UserDataTypeT> VectorT;
    typedef Cluster<VectorT, Distance<Dimension> > ClusterT;
    typedef DataType<ClusterT, InputDataType> DataT;
    typedef Distance<Dimension> DistanceT;

    typedef InitializationMethod<DataT, ClusterT, InputDataType> InitializationMethodPolicy;
};

/**
 * @brief The KMeansImp struct implements k-means clustering
 */
template <class KMeansParameter>
struct ClusteringAlgorithm : public KMeansParameter::InitializationMethodPolicy {
    enum { Dimension = KMeansParameter::Dimension };

    typedef typename KMeansParameter::VectorT VectorT;
    typedef typename KMeansParameter::ClusterT ClusterT;
    typedef typename KMeansParameter::DataT DataT;
    typedef typename KMeansParameter::IndexDataTypeT IndexDataTypeT;
    typedef typename KMeansParameter::WeightDataTypeT WeightDataTypeT;
    typedef typename KMeansParameter::InputDataTypeT InputDataTypeT;
    typedef typename KMeansParameter::DistanceT Distance;

    typedef std::pair<IndexDataTypeT, IndexDataTypeT> LimitPair;
    typedef std::vector<LimitPair> LimitPairList;

    typedef typename KMeansParameter::InitializationMethodPolicy InitializationMethodPolicy;
    using InitializationMethodPolicy::data;
    using InitializationMethodPolicy::clusters;

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

            InitializationMethodPolicy::assignVectorsToNearestCluster();
            if(recomputeCentroids() == ClusterT::NO_CHANGE) {
                break;
            }
        }

        result.clear();
        std::copy(clusters.begin(), clusters.end(), std::back_inserter(result));

//        result = clusters;
    }

    typename ClusterT::MoveResult recomputeCentroids() {
        typename ClusterT::MoveResult result = ClusterT::NO_CHANGE;

        for(typename std::vector<ClusterT>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
            if(cluster->recomputeCentroid() == ClusterT::CHANGE) {
                result = ClusterT::CHANGE;
            }
        }

        return result;
    }

    void initialize(const InputDataTypeT& input, const LimitPairList& limits) {
        srand(seed_);

        InitializationMethodPolicy::init(K, limits);
    }

private:
    unsigned K;
    long seed_;
};

}

#endif // KMEANS_H
