#ifndef PATH_EXCEPTIONS_H
#define PATH_EXCEPTIONS_H

/// SYSTEM
#include <stdexcept>

struct EmergencyBreakException : public std::runtime_error
{
    EmergencyBreakException(const std::string& what)
        : std::runtime_error(what)
    {

    }
};

#endif // PATH_EXCEPTIONS_H
