#ifndef PATH_EXCEPTIONS_H
#define PATH_EXCEPTIONS_H

/// SYSTEM
#include <stdexcept>
#include <cstdint> // uint8_t

#include <path_msgs/FollowPathResult.h>

/**
 * @brief The EmergencyBreakException struct is thrown on unrecoverable errors and
 *        should be caught up-stream to halt the robot.
 */
struct EmergencyBreakException : public std::runtime_error
{
    EmergencyBreakException(const std::string& what,
                            uint8_t status = path_msgs::FollowPathResult::RESULT_STATUS_INTERNAL_ERROR)
        : std::runtime_error(what),
          status_code(status)
    {
    }

    const uint8_t status_code;
};

#endif // PATH_EXCEPTIONS_H
