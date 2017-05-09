/// HEADER
#include <path_follower/factory/abstract_factory.h>

#include <algorithm>

AbstractFactory::~AbstractFactory()
{

}

std::string AbstractFactory::toLower(const std::string& s)
{
    std::string s_low = s;
    std::transform(s_low.begin(), s_low.end(), s_low.begin(), ::tolower);
    return s_low;
}
