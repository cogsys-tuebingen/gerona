#include <path_follower/utils/parameters.h>
#include <iostream>
#include <fstream>

using namespace std;

// this has to be defined here to make the compiler happy...
std::vector<Parameters*> Parameters::instances_;


void Parameters::print()
{
    for (vector<ParamInfo>::const_iterator it = params_.begin(); it != params_.end(); ++it) {
        ROS_INFO_STREAM(it->name << " [" << it->default_value << "]:\t" << it->description);
    }
}

void Parameters::printToFile(const std::string &filename)
{
    ofstream file;
    file.open(filename.c_str(), ios::out | ios::app);

    if (!file.is_open()) {
        ROS_ERROR("Can't open file %s for writing.", filename.c_str());
        return;
    }

    //TODO: first check, if file is empty and print table header if yes.

    //file << "| Name | Default | Description |" << endl;
    for (vector<ParamInfo>::const_iterator it = params_.begin(); it != params_.end(); ++it) {
        file << it->name << "\t| " << it->default_value << "\t| " << it->description << endl;
    }

    file.close();
}

Parameters::Parameters(const std::string& ns, Parameters* parent)
    : parent_(parent)
{
    if(parent_) {
        ns_ = parent_->ns_;
    }
    ns_ += ns;
    if(ns_ != "~") {
        ns_ += '/';
    }

    registerInstance(this);
}

Parameters::~Parameters()
{
    unregisterInstance(this);
}

void Parameters::registerInstance(Parameters *ptr)
{
    instances_.push_back(ptr);
}

void Parameters::unregisterInstance(Parameters *ptr)
{
    instances_.erase(std::remove(instances_.begin(), instances_.end(), ptr), instances_.end());
}

void Parameters::printAllInstances()
{
    for (vector<Parameters*>::const_reverse_iterator it = instances_.rbegin(); it != instances_.rend(); ++it) {
        (*it)->print();
    }
}

void Parameters::printToFileAllInstances(const string &filename)
{
    for (vector<Parameters*>::const_reverse_iterator it = instances_.rbegin(); it != instances_.rend(); ++it) {
        (*it)->printToFile(filename);
    }
}
