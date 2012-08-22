// C/C++
#include <cmath>

// Project
#include "person_filter.h"

using namespace Eigen;
using namespace std;

namespace lib_laser_processing {

PersonFilter::PersonFilter()
{
}

void PersonFilter::update( const std::vector<Person> &pers )
{
    // For each hypo
    std::vector<PersonProbability>::iterator it = hypo_.begin();
    for ( ; it != hypo_.end(); ++it ) {
        it->cov(0,0) += 0.05;
        it->cov(1,1) += 0.05;

        if ( sqrt( it->cov(0,0)) > 1.0 || sqrt( it->cov(1,1)) > 1.0 ) {
            it = hypo_.erase( it );
            /// TODO
            it = hypo_.begin();
        }
    }

    // For each detected person
    Person p;
    for ( std::size_t i = 0; i < pers.size(); ++i ) {
        p = pers[i];

        // Get best matching existing hypothesis
        int hypo_idx = getMatchingHypo( p );

        // No hypothesis found?
        if ( hypo_idx < 0 ) {
            hypo_.push_back( createNewHypo( p ));
            continue;
        }

        // Hypothesis found. Update
        PersonProbability* h = &hypo_[hypo_idx];
        double k = h->cov(0,0)/(h->cov(0,0) + 0.4*0.4 );
        h->pos(0) = h->pos(0) + k*(p.pos(0) - h->pos(0));
        h->cov(0,0) *= (1.0 - k);
        k = h->cov(1,1)/(h->cov(1,1) + 0.4*0.4 );
        h->pos(1) = h->pos(1) + k*(p.pos(1) - h->pos(1));
        h->cov(1,1) *= (1.0 - k);
    }
}

PersonProbability PersonFilter::createNewHypo(const Person &p)
{
    PersonProbability h;
    h.pos = h.first_seen = p.pos;
    h.vel = Vector2d::Zero();
    h.state.head<2>() = h.pos;
    h.state.tail<2>() = h.vel;
    h.cov << 0.7*0.7, 0, 0, 0,
            0, 0.7*0.7, 0, 0,
            0, 0, 1.0, 0,
            0, 0, 0, 1.0;
    return h;
}

int PersonFilter::getMatchingHypo(const Person &pers)
{
    int ret_idx = -1;

    // Check all hypotheses
    double p;
    double p_best = 0.1;
    Eigen::Vector2d x;
    PersonProbability* h;
    for ( std::size_t i = 0; i < hypo_.size(); ++i ) {
        h = &hypo_[i];
        p = probability( pers.pos(0), h->pos(0), sqrt( h->cov(0,0)));
        p *= probability( pers.pos(1), h->pos(1), sqrt( h->cov(1,1)));
        if ( p > p_best ) {
            p_best = p;
            ret_idx = i;
        }
    }
    return ret_idx;
}

double PersonFilter::probability( double x, double u, double s )
{
    return exp(-0.5*pow((x - u)/s, 2 ))/(sqrt( 2.0*M_PI)*s);
}

} // namespace
