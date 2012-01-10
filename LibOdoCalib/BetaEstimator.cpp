#include <iostream>
#define EIGEN2_SUPPORT
#include <Eigen/LeastSquares>
#include <Eigen/Dense>
#include <cmath>
#include "BetaEstimator.h"
#include "Misc.h"
#include "Calibration.h"
using namespace std;

BetaEstimator::BetaEstimator()
  :min_dist_(0.3),threshold_(0.05),quota_(0.6)
{

}

BetaEstimator::BetaEstimator(double min_dist,double threshold, double quota)
    :min_dist_(min_dist), threshold_(threshold),quota_(quota)
{

}


void BetaEstimator::update (const Vector3d& p)
{
    if (points_.size()>0) {
        double d=(p.head<2>()-points_.back().head<2>()).norm();
        dists_.push_back(d);
    }
    points_.push_back(p);
}


void BetaEstimator::reset()
{
  points_.clear();
  dists_.clear();
}


bool BetaEstimator::getPointsToDist(double min_dist, Vector2dVec &ps, int theta_delta,double& theta)
{
    // count how many points to go back to reach min_dist
    DList::reverse_iterator ritd=dists_.rbegin();
    double dist_sum=0.0;
    int cnt=0;
    while (ritd!=dists_.rend()&& !(dist_sum>=min_dist_ && cnt>=3)) {
        dist_sum+=*ritd;
        ++ritd;
        ++cnt;
    }
    if (dist_sum<min_dist_ || cnt<3) {
        return false;
    }
    ps.resize(cnt);

    Vector3dList::reverse_iterator ritp=points_.rbegin();
    theta=(*ritp)(2);
    for (int i=cnt-1;i>=0;--i) {
        ps[i]=ritp->head<2>();
        ++ritp;
        if (i==cnt-1-theta_delta) {
            theta=(*ritp)(2);
        }
    }
    if (cnt-1<theta_delta) {
        theta=(*ritp)(2);
    }
    return true;

}

bool BetaEstimator::calcLsBetaRansac(int theta_delta, int direction, double &beta)
{
    Vector2dVec points,conset;
    double theta;
    bool success=getPointsToDist(min_dist_,points,theta_delta,theta);
    if (!success) return false;
    success=getConsensusSet(points,threshold_,quota_,conset);
    if (!success) return false;
    // pointers on conset
    int n =conset.size();
    vector<Vector2d*> ptrs(n);
    for (int i = 0;i<n;++i) {
        ptrs[i]=&(conset[i]);
    }
    Vector2d res;
    Vector2d BA;
    int dep=1;
    linearRegression(n,&(ptrs[0]),&res,dep);
    if (res(0)>1000) {
        dep = 0;
        linearRegression(n,&(ptrs[0]),&res,dep);
        BA<<res(0),1;
    } else {
        BA<<1,res(0);
    }
    Vector2d A=conset.front();
    Vector2d B = conset.back();
    double d=BA.norm();
    if (d<1e-6) {
        std::cout << "error in ls ransac" << endl;
        return false;
    }
    Vector2d dir;
    dir << cos(theta),sin(theta);
    Vector2d CA = dir*d;
    double cosbeta = BA.dot(CA)/BA.squaredNorm();
    Vector2d C=A+d*dir;
    Matrix3d T;
    T << A,B,C,1,1,1;
    double h=T.determinant()/d;
    beta = acos(cosbeta);
    if (direction==POS_FRONT) {
        if (h>0) {
            beta= -1.0*beta;
        } else {
            beta= 1.0*beta;
        }
    } else {
        //if (direction==MODE_BACKWARD) beta=beta-M_PI;
        beta=Misc::normalizeAngle(beta);
        if (h>0) {
            beta= 1.0*beta;
        } else {
            beta=-1.0*beta;
        }
    }
    if (isnan(beta)) {
        std::cout << "beta is nan d="<<d<<" BA="<<BA <<" CA="<<CA<<std::endl;
        return false;
    }
    return true;
}


bool BetaEstimator::calcLsBetaAngle (int theta_delta, int direction, double& beta)
{
    Vector2dVec ps;
    double theta;
    bool success=getPointsToDist(min_dist_,ps,theta_delta,theta);
    if (!success) return false;
    if (ps.size()<3) {
      std::cout << "too few points for beta estimation n="<<ps.size()<< std::endl;

      return false;
    }
    vector<Vector2d*> ptrs(ps.size());
    for (unsigned int i = 0;i<ps.size();++i) {
        ptrs[i]=&(ps[i]);
    }
    Vector2d res;
    Vector2d BA;
    int dep=1;
    linearRegression(ps.size(),&(ptrs[0]),&res,dep);
    if (res(0)>1000) {
        dep = 0;
        linearRegression(ps.size(),&(ptrs[0]),&res,dep);
        BA<<res(0),1;
    } else {
        BA<<1,res(0);
    }
    Vector2d A=ps.front();
    Vector2d B = ps.back();
    double d=BA.norm();
    Vector2d dir;
    dir << cos(theta),sin(theta);
    Vector2d CA = dir*d;
    double cosbeta = BA.dot(CA)/BA.squaredNorm();
    Vector2d C=A+d*dir;
    Matrix3d T;
    T << A,B,C,1,1,1;
    double h=T.determinant()/d;
    beta = acos(cosbeta);
    if (direction==POS_FRONT) {
        if (h>0) {
            beta= -1.0*beta;
        } else {
            beta= 1.0*beta;
        }
    } else {
        //if (direction==MODE_BACKWARD) beta=beta-M_PI;
        beta=Misc::normalizeAngle(beta);
        if (h>0) {
            beta= 1.0*beta;
        } else {
            beta=-1.0*beta;
        }
    }
    if (isnan(beta)) {
        std::cout << "beta is nan d="<<d<<" BA="<<BA <<" CA="<<CA<<std::endl;
        for (unsigned int i=0;i<ps.size();++i) {
          std::cout << ps[i]<<endl;
        }
        return false;
    }

    return true;
}

bool BetaEstimator::getConsensusSet(const Vector2dVec& points, double threshold, double quota,Vector2dVec& conset )
{
    IVector set,cset;
    DVector dists,cdists;
    int n = points.size();
    if (n<4) {
        return false;
    }
    set.reserve(n);
    cset.reserve(n);
    dists.reserve(n);
    cdists.reserve(n);
    conset.reserve(n);

    for (int it=0;it<n;++it) {
        cset.clear();
        cdists.clear();
        // select two random different indices
        int i1=rand()%n;
        int i2=rand()%n;
        while (i2==i1) {
            i2=rand()%n;
        }
        // the line is Q1Q2
        Vector2d Q1=points[i1];
        Vector2d Q2=points[i2];
        for (int i=0;i<n;++n) {
            if (i!=i1 && i!=i2) {
                // check point P
                Vector2d P=points[i];
                // calc distance P to line Q1Q2
                // http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
                double d=fabs((Q2.x()-Q1.x())*(Q1.y()-P.y())-(Q1.x()-P.x())*(Q2.y()-Q1.y()))
                        /(Q2-Q1).norm();
                if (d<threshold) {
                    // save good point index in cset
                    cset.push_back(i);
                    cdists.push_back(d);
                }
            }  else {
                // Q1 and Q2 are part of the set
                cset.push_back(i);
            }
        }
        // set larger than previous best set?
        if (cset.size()>set.size()) {
            // copy it
            set=cset;
            dists=cdists;
        }
        if (cset.size()>=n-1) {
            // n-1 size conset is good enough
            break;
        }
    }
    if (set.size()<quota*n) {
        return false;
    }
    for (unsigned int i=0;i<set.size();++i) {
        conset.push_back(points[set[i]]);
    }
    return true;
}
