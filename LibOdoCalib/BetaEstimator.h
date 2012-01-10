#ifndef BETAESTIMATOR_H
#define BETAESTIMATOR_H
#include "Global.h"


/**
  estimates the driven angle beta (= angle between direction of movement and orientation)
  from consecutive poses


  */
class BetaEstimator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
    default ctor

    */
  BetaEstimator();
  /**
    @param min_dist minimum distance between first and last pose required for valid beta-estimation
    @param threshold  ransac param
    @param quota ransac parameter
    */
    BetaEstimator(double min_dist,double threshold, double quota);

    void reset ();
    void update (const Vector3d& p);
    //double calcBetaAngle ();
    bool calcLsBetaAngle (int delta, int direction, double& beta);
    bool calcLsBetaRansac (int delta, int direction, double& beta);
    void setMinDist (double min_dist) {min_dist_ = min_dist;}
private:
    /**
      return points from last covering distance dist
      */
    bool getPointsToDist (double dist,Vector2dVec& ps, int theta_delta,double& theta);

    /**
      ransac the best line fitting points of the given point set
      @param threshold thresold distance for outliers
      @param quota return false if set is smaller than quota*length of points
      @param conset resulting set
      */
    bool getConsensusSet(const Vector2dVec& points, double threshold, double quota,Vector2dVec& conset );
    double min_dist_;
    double threshold_;
    double quota_;
    Vector3dList points_;
    DList dists_;
};

#endif // BETAESTIMATOR_H
