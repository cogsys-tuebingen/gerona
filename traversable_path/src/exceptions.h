#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <exception>

namespace traversable_path
{

/**
 * @brief Simple default exception.
 *
 * Extends std::exception and takes an optional error message as constructor argument
 */
class Exception: std::exception
{
public:
    Exception() throw(): msg_("")
    {}

    Exception(const char* msg) throw(): msg_(msg)
    {}

    ~Exception() throw()
    {}

    virtual const char* what() const throw()
    {
        return msg_;
    }

private:
     const char* msg_;
};


/**
 * @brief Exception for map transformation failures.
 */
class TransformMapException: Exception
{
public:
    TransformMapException() throw():
        Exception("Point lies outside of the map.")
    {
    }

    virtual const char* what() const throw()
    {
        return Exception::what();
    }
};

}

#endif // EXCEPTIONS_H
