/**
 * (c) Cognitive Systems, University of Tübingen
 *
 * @date Aug 2012
 * @author marks
 */

// C/C++
#include <map>

// Project
#include "person_detector.h"

// Lemon graph library
#ifdef USE_LEMON_GRAPH_LIB
#include <lemon/matching.h>
using namespace lemon;
#endif

using namespace std;

namespace lib_laser_processing {

PersonDetector::PersonDetector()
{
}

void PersonDetector::update( const std::vector<Leg> &legs )
{
    /**
     * @todo Use something like graph matching to Compute
     * an optimal solution. This basic version fails with many persons
     */

    persons_.clear();

#ifdef USE_LEMON_GRAPH_LIB

    ListGraph g;
    buildGraph( legs, g );
    //printGraph( g );

#else

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
            if (!processed[j] && (l.pos - legs[j].pos).norm() < 0.7 ) {
                p.legs.push_back( legs[j] );
                processed[j] = true;
                break;
            }
        }

        // Estimate persons center of mass
        if ( p.legs.size() < 2 ) {
            p.pos = p.legs[0].pos + 0.15*p.legs[0].pos.normalized();
        } else {
            p.pos = 0.5*(p.legs[0].pos + p.legs[1].pos);
        }
        persons_.push_back( p );
        p.legs.clear();
    }

#endif
}

#ifdef USE_LEMON_GRAPH_LIB
void PersonDetector::buildGraph( const std::vector<Leg> &legs, lemon::ListGraph& g )
{
    // Remove all nodes and edges
    g.clear();

    // Add a node for each leg and fill the node-leg map
    ListGraph::NodeMap<Leg> node_leg_map(g);
    for ( vector<Leg>::const_iterator i = legs.begin(); i != legs.end(); ++i )
        node_leg_map.set( g.addNode(), *i );

    // Insert edges and calculate edge weights
    ListGraph::EdgeMap<double> weights( g );
    for ( ListGraph::NodeIt i(g); i != INVALID; ++i ) {
        // Corresponding leg for current node
        Leg l = node_leg_map[i];

        ListGraph::NodeIt j(i);
        ++j;
        for ( ; j != INVALID; ++j ) {
            double d = (l.pos - node_leg_map[j].pos).norm();
            if ( d < 0.7 )
                weights.set( g.addEdge( i, j ), 0.7 - d );
        }
    }

    /// TEST ///
    MaxWeightedMatching<ListGraph, ListGraph::EdgeMap<double> > matcher( g, weights );

    //cout << "Running graph matcher" << endl;
    matcher.run();
    //cout << "Graph matcher finished" << endl;
    //cout << "Matching size: " << matcher.matchingSize() << endl;
    //cout << "Matching weight: " << matcher.matchingWeight() << endl;
    for ( ListGraph::EdgeIt i( g ); i != INVALID; ++i ) {
        if ( matcher.matching(i)) {

            Person p;
            p.legs.push_back( node_leg_map[g.u(i)]);
            p.legs.push_back( node_leg_map[g.v(i)]);
            // Estimate persons center of mass
            if ( p.legs.size() < 2 ) {
                p.pos = p.legs[0].pos + 0.15*p.legs[0].pos.normalized();
            } else {
                p.pos = 0.5*(p.legs[0].pos + p.legs[1].pos);
            }
            persons_.push_back( p );
        }
    }

    for ( ListGraph::NodeIt i( g ); i != INVALID; ++i ) {
        if ( matcher.mate(i) == INVALID ) {
            Person p;
            p.legs.push_back( node_leg_map[i]);
            // Estimate persons center of mass
            if ( p.legs.size() < 2 ) {
                p.pos = p.legs[0].pos + 0.15*p.legs[0].pos.normalized();
            } else {
                p.pos = 0.5*(p.legs[0].pos + p.legs[1].pos);
            }
            persons_.push_back( p );
        }
    }
}

void PersonDetector::printGraph( const lemon::ListGraph &g ) const
{
    cout << "Graph edges:\n";
    for ( ListGraph::EdgeIt i(g); i != INVALID; ++i )
        cout << "\t(" << g.id(g.u(i)) << "," << g.id(g.v(i)) << ")\n";
    cout << endl;
}
#endif

} // namespace
