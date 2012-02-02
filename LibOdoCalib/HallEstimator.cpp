#include "HallEstimator.h"

const string& LOG_RUNNO="runno";
const string& LOG_BETA_TARGET="betatarget";
const string& LOG_FRONT_MEAN="frontmean";
const string& LOG_REAR_MEAN="rearmean";
const string& LOG_FRONT_STD="frontstd";
const string& LOG_REAR_STD="rearstd";
const string& LOG_CTRL_COUNT="ctrl_count";
const string& LOG_BETA_ERR_MEAN="betaerrmean";
const string& LOG_YAW_ERR_MEAN="yawerrmean";
const string& LOG_CALIB_LEFT="calibleft";
const string& LOG_CALIB_RIGHT="calibright";
const string& LOG_HALL1_MEAN="hall1_mean";
const string& LOG_HALL1_STD="hall1_std";
const string& LOG_HALL2_MEAN="hall2_mean";
const string& LOG_HALL2_STD="hall2_std";
const string& LOG_HALL3_MEAN="hall3_mean";
const string& LOG_HALL3_STD="hall3_std";
const string& LOG_HALL4_MEAN="hall4_mean";
const string& LOG_HALL4_STD="hall4_std";

const int HALL_SENSOR_NUMBER=4;
HallEstimator::HallEstimator()
{
  current_stats_.resize(HALL_SENSOR_NUMBER);
  started_=false;
  configureLog();

}


HallEstimator::~HallEstimator()
{
  for (std::list<HallDataSet*>::iterator it=hall_runs_.begin();it!=hall_runs_.end();++it) {
    delete *it;
  }

}


void HallEstimator::startResultLog(const string &fname)
{
  run_counter_=0;
  logger_.enable(fname,true);
  started_=true;

}


void HallEstimator::configureLog()
{
  //% runNo radTarget front_mean rear_mean front_std rear_std ctrl_count
  // rad_err_mean yaw_rate_err_mean calib_left calib_right hall1_mean hall1_std hall2_mean hall2_std
  logger_.addColumn(LOG_RUNNO,"Number of test run",false);
  logger_.addColumn(LOG_BETA_TARGET,LOG_BETA_TARGET,false);
  logger_.addColumn(LOG_FRONT_MEAN);
  logger_.addColumn(LOG_REAR_MEAN);
  logger_.addColumn(LOG_FRONT_STD);
  logger_.addColumn(LOG_REAR_STD);
  logger_.addColumn(LOG_CTRL_COUNT);
  logger_.addColumn(LOG_BETA_ERR_MEAN);
  logger_.addColumn(LOG_YAW_ERR_MEAN);
  logger_.addColumn(LOG_CALIB_LEFT);
  logger_.addColumn(LOG_CALIB_RIGHT);
  logger_.addColumn(LOG_CALIB_LEFT);
  logger_.addColumn(LOG_HALL1_MEAN);
  logger_.addColumn(LOG_HALL1_STD);
  logger_.addColumn(LOG_HALL2_MEAN);
  logger_.addColumn(LOG_HALL2_STD);
  logger_.addColumn(LOG_HALL3_MEAN);
  logger_.addColumn(LOG_HALL3_STD);
  logger_.addColumn(LOG_HALL4_MEAN);
  logger_.addColumn(LOG_HALL4_STD);

}


void HallEstimator::addHallVoltage(int idx, double val)
{
  if (idx<0 ||idx>=HALL_SENSOR_NUMBER) {
    std::cout << "error halestimator"<<std::endl;
    return;
  }
  current_stats_[idx].update(val);
}


void HallEstimator::finishRun(double beta, int count)
{
  logger_.addValue(LOG_RUNNO,run_counter_);
  logger_.addValue(LOG_BETA_TARGET,beta);
  logger_.addValue(LOG_HALL1_MEAN,current_stats_[0].getMean());
  logger_.addValue(LOG_HALL2_MEAN,current_stats_[1].getMean());
  logger_.addValue(LOG_HALL3_MEAN,current_stats_[2].getMean());
  logger_.addValue(LOG_HALL4_MEAN,current_stats_[3].getMean());
  logger_.addValue(LOG_HALL1_STD,current_stats_[0].getStd());
  logger_.addValue(LOG_HALL2_STD,current_stats_[1].getStd());
  logger_.addValue(LOG_HALL3_STD,current_stats_[2].getStd());
  logger_.addValue(LOG_HALL4_STD,current_stats_[3].getStd());

  logger_.writeLogLine();
  HallDataSet *set = new HallDataSet(beta,current_stats_);

  hall_runs_.push_back(set);
  for (int i=0;i<HALL_SENSOR_NUMBER;++i) {
    current_stats_[i].reset();
  }
  ++run_counter_;
}


void HallEstimator::analyze()
{
  if (isStarted()) {
    logger_.disable();
    started_=false;
  }
}
