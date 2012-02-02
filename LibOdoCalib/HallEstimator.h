#ifndef HALLESTIMATOR_H
#define HALLESTIMATOR_H
#include <list>
#include "StatsEstimator.h"
#include "LogCollector.h"
struct HallDataSet
{
  HallDataSet(double beta, vector<StatsEstimator<float> > stats)
    :target_beta_(beta),hall_stats_(stats) {}
  double target_beta_;
  //double mean_beta_;
  vector<StatsEstimator<float> > hall_stats_;


};

class HallEstimator
{
public:
  HallEstimator();
  ~HallEstimator();
  void addHallVoltage(int idx,double val);
  void finishRun(double beta, int count);
  void startResultLog(const string& fname);
  bool isStarted() {return started_;}
  void analyze();
private:
  void configureLog();
  vector<StatsEstimator<float> > current_stats_;

  LogCollector logger_;
  std::list<HallDataSet*> hall_runs_;
  bool started_;
  int run_counter_;
};

#endif // HALLESTIMATOR_H
