#ifndef STATSESTIMATOR_H
#define STATSESTIMATOR_H
#include <limits>
#include <cmath>
/**
  online estimator for statistical values of a
  random variable
  see Incremental calculation of weighted mean and variance, finch, 2009
  @author bohlmann
  @date early 21st century
  */
template <class T>
class StatsEstimator {
public:
    StatsEstimator ()
        :n_(0),sum_(T(0)),mu_(T(0)),s_(T(0)),max_(std::numeric_limits<T>::min()),
        min_(std::numeric_limits<T>::max()) {;}

    StatsEstimator (const StatsEstimator<T>& src) {
        *this = src;
    }

    ~StatsEstimator () {;}// nothing to do

    StatsEstimator<T>& operator=(const StatsEstimator<T>& src) {
        if (this==&src) return *this;
        this->n_=src.n_;
        this->min_=src.min_;
        this->max_=src.max_;
        this->mu_=src.mu_;
        this->s_=src.s_;
        this->sum_=src.sum_;
        return *this;
    }

    /**
      update statistics estimations with a new value of x
      */
    void update (T x) {
        // update min/max
        if (x>max_) max_=x;
        if (x<min_) min_=x;
        // update sum
        sum_ += x;
        // update count
        ++n_;
        // calculate new mean
        T mu_prev=mu_;
        mu_ = mu_prev+(x-mu_prev)/n_;
        // update S
        s_ = s_+(x-mu_prev)*(x-mu_);
    }

    /**
      @return mean of variable
      */
    T getMean() const {return mu_;}

    /**
      @return standard deviation
      */
    T getStd() const {return (n_>=2)?sqrt(s_/(n_-1)):0;}

    /**
      @return max
      */
    T getMax () const {return max_;}

    /**
      @return min
      */
    T getMin () const {return min_;}

    /**
        reset values
      */
    void reset() {n_=T(0);mu_=T(0);s_=T(0);max_=std::numeric_limits<T>::min(),
                min_=std::numeric_limits<T>::max();}
private:
    T n_;
    T sum_;
    T mu_;
    T s_;
    T max_,min_;


};

#endif // STATSESTIMATOR_H
