/**
 * @file PlannerExceptions.h
 * @date Feb 2012
 * @author marks
 */

#ifndef PLANNEREXCEPTIONS_H
#define PLANNEREXCEPTIONS_H

// C/C++
#include <string>
#include <exception>

/**
 * @brief Exception representing several different errors.
 */
class PlannerException : public std::exception
{
public:
    /**
     * @brief Create object.
     * @param msg Short error description.
     */
    PlannerException( const std::string msg ) : msg_( msg ) {}

    virtual ~PlannerException() throw() { /* Nothing to do */ }

    /**
     * @brief Get the error description.
     * @return A short error description.
     */
    virtual const char* what() const throw() { return msg_.c_str(); }

protected:
    /// Short error description
    std::string msg_;
};

/**
 * @brief Exception if there is no valid path.
 */
class NoPathException : public PlannerException
{
public:
    /**
     * @brief Create object.
     * @param msg Short error description.
     */
    NoPathException( const std::string msg ) : PlannerException( msg ) {}

    virtual ~NoPathException() throw() { /* Nothing to do. */ }
};

#endif // PLANNEREXCEPTIONS_H
