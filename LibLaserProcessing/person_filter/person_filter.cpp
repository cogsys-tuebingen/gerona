/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

// C/C++
#include <cmath>

// Eigen
#include <Eigen/LU>

// Project
#include "person_filter.h"

using namespace Eigen;
using namespace std;

namespace lib_laser_processing {

PersonFilter::PersonFilter()
{
    config_.noise
            << pow( 1.0, 2 ), 0, 0, 0,
            0, pow( 1.0, 2 ), 0, 0,
            0, 0, 1.0, 0,
            0, 0, 0, 1.0;
    config_.measurement_noise
            << pow( 0.4, 2 ), 0, 0, 0,
            0, pow( 0.4, 2 ), 0, 0,
            0, 0, pow( 2.0, 2 ), 0,
            0, 0, 0, pow( 2.0, 2 );
    config_.initial_cov
            << pow( 0.6, 2 ), 0, 0, 0,
            0, pow( 0.6, 2 ), 0, 0,
            0, 0, pow( 1.0, 2 ), 0,
            0, 0, 0, pow( 1.0, 2 );
    config_.max_position_var = pow( 0.7, 2 );
    config_.slow_down = 0.95;
}

void PersonFilter::update( const vector<Person> &pers, const double dt )
{
    // Prediction step for each hypothesis
    for ( list<Hypothesis>::iterator it = hypo_.begin(); it != hypo_.end(); ++it ) {
        // Slow down
        it->state.tail<2>() *= config_.slow_down;

        // Estimated movement
        it->state.head<2>() += dt*it->state.tail<2>();

        // Add uncertainty
        it->cov += dt*config_.noise;
    }

    // For each detected person blob
    for ( vector<Person>::const_iterator it = pers.begin(); it != pers.end(); ++it ) {
        // Get best matching existing hypothesis
        Hypothesis* h = getMatchingHypo( *it, dt );

        // No hypothesis found? Create one
        if ( !h ) {
            hypo_.push_back( createNewHypo( *it ));
            continue;
        }

        // Calculate velocity
        Vector4d m;
        m.head<2>() = it->pos;
        m.tail<2>() = (it->pos - h->state.head<2>())/dt;

        // Correction step
        Matrix4d k;
        if ( it->legs.size() > 1 )
            k = h->cov*(h->cov + config_.measurement_noise).inverse();
        else
            k = h->cov*(h->cov + 2.5*config_.measurement_noise).inverse(); /// @todo This is a hack
        h->state += k*(m - h->state);
        h->cov = (Matrix4d::Identity() - k)*h->cov;
    }

    // Delete bad hypotheses and update person probability
    list<Hypothesis>::iterator it = hypo_.begin();
    while (  it != hypo_.end()) {
        if ( it->cov(0,0) > config_.max_position_var || it->cov(1,1) > config_.max_position_var ) {
            it = hypo_.erase( it );
            continue;
        }

        /// @todo This is a hack. Use at least Bayes or something like that
        if ( it->state.tail<2>().norm() > 0.2 )
            it->person_prob += 0.005;
        else
            it->person_prob -= 0.02;

        if ((it->state.head<2>() - it->first_seen).norm() > 0.3 )
            it->person_prob += 0.01;

        if ( it->cov(0,0) < 0.1 )
            it->person_prob += 0.0;
        else
            it->person_prob -= 0.05;

        if ( it->person_prob > 1.0 )
            it->person_prob = 1.0;
        if ( it->person_prob < 0 )
            it->person_prob = 0;

        ++it;
    }
}

PersonFilter::Hypothesis PersonFilter::createNewHypo( const Person &p )
{
    Hypothesis h;
    h.state = Vector4d::Zero();
    h.state.head<2>() = p.pos;
    h.cov = config_.initial_cov;
    h.first_seen = p.pos;
    h.person_prob = 0.3;
    return h;
}

PersonFilter::Hypothesis* PersonFilter::getMatchingHypo( const Person &pers, const double dt )
{
    /// @todo It might be possible to use the estimated velocity as well
    /// @todo Update only one hypothesis per person?

    Hypothesis* ret( NULL );

    // Check all hypotheses
    double p;
    double p_best = 0.37;
    Vector4d x;
    x.head<2>() = pers.pos;
    list<Hypothesis>::iterator it = hypo_.begin();
    while ( it != hypo_.end()) {
        x.tail<2>() = (pers.pos - it->state.head<2>())/dt;
        p = positionProbability( x, it->state, it->cov );
        if ( p > p_best ) {
            p_best = p;
            ret = &(*it);
        }
        ++it;
    }
    return ret;
}

double PersonFilter::positionProbability( const Vector4d& x, const Vector4d& state, const Matrix4d& cov )
{
    /// @todo Correct scaling
    double p = -0.5*(x.head<2>() - state.head<2>()).transpose()*cov.topLeftCorner<2,2>().inverse()*(x.head<2>() - state.head<2>());
    p = exp(p)/(sqrt( pow( 2.0*M_PI, 2 )*cov.topLeftCorner<2,2>().determinant()));
    return p;
}

} // namespace
