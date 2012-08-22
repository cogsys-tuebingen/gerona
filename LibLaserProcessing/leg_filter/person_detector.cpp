#include "person_detector.h"

namespace lib_laser_processing {

PersonDetector::PersonDetector()
{
}

void PersonDetector::update(const std::vector<Leg> &legs)
{
    persons_.clear();

    std::vector<bool> processed;
    processed.resize( legs.size(), false );

    // For all detected legs
    Leg l;
    for ( size_t i = 0; i < legs.size(); ++i ) {
        if ( processed[i])
            continue;
        processed[i] = true;

        Person p;
        l = legs[i];
        p.legs.push_back( l );

        // Check for a second leg
        for ( size_t j = i + 1; j < legs.size(); ++j ) {
            if (!processed[j] && (l.pos - legs[j].pos).norm() < 0.75 ) {
                p.legs.push_back( legs[j] );
                processed[j] = true;
                break;
            }
        }

        // Estimate persons center of mass
        if ( p.legs.size() < 2 ) {
            p.pos = p.legs[0].pos + 0.2*p.legs[0].pos.normalized();
        } else {
            p.pos = 0.5*(p.legs[0].pos + p.legs[1].pos);
        }
        persons_.push_back( p );
        p.legs.clear();
    }
}

} // namespace
