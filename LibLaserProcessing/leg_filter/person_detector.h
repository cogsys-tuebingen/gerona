#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H

// Project
#include <utils/LibLaserProcessing/leg_filter/person.h>

namespace lib_laser_processing {

class PersonDetector
{
public:
    PersonDetector();

    void update( const std::vector<Leg>& legs );

    const std::vector<Person>& getPersons() const {
        return persons_;
    }

private:
    std::vector<Person> persons_;
};

} // namespace

#endif // PERSON_DETECTOR_H
