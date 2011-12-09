/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/15/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#include "TrajectoryMission.h"
#include "ConfigFileReader.h"
#include "CalibBotInterface.h"
#include "MoveDistAction.h"
#include "MoveTimeAction.h"
#include "StandSteerAction.h"

TrajectoryMission::TrajectoryMission(ConfigFileReader *config, const string &config_key,
                                     CalibBotInterface *proxy)
  :proxy_(proxy)
{
  configure(config,config_key);
}


TrajectoryMission::~TrajectoryMission ()
{
  // nothing to do
}


bool TrajectoryMission::execute()
{
  ConfigFileReader results;
  int result_line=0;
  string log_fname=result_fname_+".log";
  log_stream_.open(log_fname.c_str());
  for (list<RobotAction *>::iterator it=action_list_.begin();it!=action_list_.end();++it) {
    bool success = (*it)->execute();
    if (!success) {
      cout << "failed to complete robot action"<<endl;
      return false;
    }
    // retrieve results
    char buffer[256];
    snprintf(buffer,255,"trajectory%03d",result_line);
    (*it)->getResults(buffer,results);
    ++result_line;
  }
  results.Save(result_fname_+".conf");
  results.SaveMatlab(result_fname_+"_mat.log","trajectory%03d::%s");
  log_stream_.close();
  return true;
}


void TrajectoryMission::configure(ConfigFileReader *config, const string &config_key)
{
  // create and configure robot actions
  char buffer[101];
  for (int i=0;i<=99;++i) {
    snprintf(buffer,100,"::action%02d",i);
    string action_key=config_key+buffer;
    if (config->KeyExists(action_key,false)) {
      string type=config->GetString(action_key+"::type","moveDist");
      RobotAction *action = 0;
      if (type.compare("moveDist")==0) {
        action = new MoveDistAction(config,action_key,proxy_);
      } else if (type.compare("moveTime")==0) {
        action = new MoveTimeAction(config,action_key,proxy_);
      } else if (type.compare("standSteer")==0) {
        action = new StandSteerAction(config,action_key,proxy_);
      } else {
        cout << "unknown action type "<< type << " in key "<<action_key<< endl;
        continue;
      }
      action->setLog(&log_stream_,&mission_timer_);
      action_list_.push_back(action);
    }
  }
  // read in result file name
  result_fname_ = config->GetString(config_key+"::resultFile",config_key+"result");
}

