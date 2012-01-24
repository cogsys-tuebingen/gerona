/**
 * @file CombinePlannerException.h
 * @date Jan 2012
 * @author marks
 */

#ifndef COMBINEDPLANNEREXCEPTION_H
#define COMBINEDPLANNEREXCEPTION_H

// C/C++
#include <string>
#include <exception>

namespace combined_planner {

/**
 * @class CombinedPlannerException
 * @brief Used to handle planner errors.
 */
class CombinedPlannerException : public std::exception
{
public:
    /**
     * @brief Create an exception.
     * @param msg Short error description.
     */
    CombinedPlannerException( const std::string& msg );

    virtual ~CombinedPlannerException() throw();

    /**
     * @brief Get the error description.
     * @return A short error description.
     */
    virtual const char* what() const throw();

private:
    /// Error description
    std::string msg_;
};

} // namespace

#endif // COMBINEDPLANNEREXCEPTION_H
