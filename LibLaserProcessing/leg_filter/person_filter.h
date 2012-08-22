#ifndef PERSON_FILTER_H
#define PERSON_FILTER_H

// C/C++
#include <vector>

// Project
#include <utils/LibLaserProcessing/leg_filter/person.h>
#include <utils/LibLaserProcessing/leg_filter/person_probability.h>

namespace lib_laser_processing {

class PersonFilter
{
public:
    PersonFilter();

    void update( const std::vector<Person>& pers );
    const std::vector<PersonProbability>& getHypotheses() const {
        return hypo_;
    }

private:

    int getMatchingHypo( const Person& p );

    PersonProbability createNewHypo( const Person& p );

    double probability( double x, double u, double s);

    /// All hypotheses
    std::vector<PersonProbability> hypo_;
};

} // namespace

#endif // PERSON_FILTER_H
