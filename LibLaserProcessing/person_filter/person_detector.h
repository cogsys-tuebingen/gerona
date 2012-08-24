/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H

// Project
#include <utils/LibLaserProcessing/person_filter/person.h>

namespace lib_laser_processing {

/**
 * @brief Used to match detected legs to persons
 */
class PersonDetector
{
public:
    /**
     * @brief Create object and initialize with default values
     */
    PersonDetector();

    /**
     * @brief Process detected legs
     * @param legs Detected legs
     */
    void update( const std::vector<Leg>& legs );

    /**
     * @brief Return detected persons
     * @return All persons
     */
    const std::vector<Person>& getPersons() const {
        return persons_;
    }

private:

    /// All persons
    std::vector<Person> persons_;
};

} // namespace

#endif // PERSON_DETECTOR_H
