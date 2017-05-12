#include <path_follower/utils/parameters.h>
#include <iostream>

using namespace std;

std::vector<Parameters*> Parameters::instances_;

Parameters::Parameters()
    : parent_(nullptr),
      ns_("~")
{
    registerInstance(this);
}
Parameters::Parameters(const std::string& ns)
    : parent_(nullptr)
{
    if(ns.empty()) {
        ns_ = "~";

    } else {
        if(ns[0] != '~') {
            ns_ = "~" + ns;
        } else {
            ns_ = ns;
        }
        if(ns_ != "~") {
            ns_ += '/';
        }
    }

    registerInstance(this);
}
Parameters::Parameters(const std::string& ns, const Parameters* parent)
    : parent_(parent)
{
    ns_ = parent_->ns_ + ns;
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

void Parameters::visitParameters(std::function<void(const ParamInfo&)> visitor)
{
    for (vector<Parameters*>::const_reverse_iterator it = instances_.rbegin(); it != instances_.rend(); ++it) {
        Parameters* p = *it;
        for(ParamInfo& info : p->params_) {
            visitor(info);
        }
    }
}


template <>
std::string type2name<double>()
{
    return "double";
}
template <>
std::string type2name<float>()
{
    return "double";
}

template <>
std::string type2name<int>()
{
    return "int";
}
template <>
std::string type2name<bool>()
{
    return "bool";
}

template <>
std::string type2name<std::string>()
{
    return "string";
}

template <>
std::string type2name<const char*>()
{
    return "string";
}
