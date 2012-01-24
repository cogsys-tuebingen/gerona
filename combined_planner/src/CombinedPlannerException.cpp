/**
 * @file CombinePlannerException.cpp
 * @date Jan 2012
 * @author marks
 */

// Project
#include "CombinedPlannerException.h"

using namespace combined_planner;

CombinedPlannerException::CombinedPlannerException( const std::string &msg )
    : msg_( msg )
{ /* Nothing to do */ }

CombinedPlannerException::~CombinedPlannerException() throw()
{ /* Nothing to do */ }

const char* CombinedPlannerException::what() const throw()
{
    return msg_.c_str();
}
