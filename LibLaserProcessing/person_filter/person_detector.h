/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H

/// Use graph matching to optimize leg to person assignment (needs lemon graph library)?
//#define USE_LEMON_GRAPH_LIB

// C/C++
#include <list>

// Lemon graph library
#ifdef USE_LEMON_GRAPH_LIB
#include <lemon/list_graph.h>
#endif

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

#ifdef USE_LEMON_GRAPH_LIB
    void buildGraph( const std::vector<Leg>& legs, lemon::ListGraph& g );
    void printGraph( const lemon::ListGraph& g ) const;
#endif

    /// All persons
    std::vector<Person> persons_;
};

} // namespace

#endif // PERSON_DETECTOR_H
