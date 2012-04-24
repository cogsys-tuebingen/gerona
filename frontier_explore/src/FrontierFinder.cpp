
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "FrontierFinder.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

bool FrontierFinder::getFrontiers( const lib_path::GridMap2d *map,
                                   std::vector<Frontier> &frontiers )
{
    frontiers.clear();

    int w = map->getWidth();
    int h = map->getHeight();
    unsigned int size = ( w * h );

    if ( size <= 0 )
        return false;

    bool visited[size];
    memset( visited, false, size*sizeof(bool));
    std::list<unsigned int> to_visit;
    bool left, right, top, bottom;
    std::vector<FrontierPoint> frontier;
    int min_x, max_x, min_y, max_y;
    unsigned int cell;

    // For all cells
    for ( unsigned int y = 0; (int)y < h; ++y ) {
        for ( unsigned int x = 0; (int)x < w; ++x ) {
            cell = y*w + x;

            // Occupied or visited?
            if ( !map->isFree( x, y ) || visited[cell] )
                continue;

            // Cell is free and unvisited. Search frontier
            frontier.clear();
            to_visit.push_back( cell );
            min_x = min_y = size;
            max_x = max_y = -1;
            while ( !to_visit.empty()) {
                // Get current cell coordinates
                cell = to_visit.back();
                to_visit.pop_back();
                unsigned int q = cell/w;
                unsigned int p = cell - w*q;

                // Neighbor flags
                left = right = top = bottom = false;
                left = p > 0;
                right = (int)p < (w - 1);
                top = (int)q < (h - 1);
                bottom = q > 0;

                // Check neighbors and set cell visited
                unsigned int count = 0;
                Eigen::Vector2d dir( 0, 0 );
                if ( left && map->isNoInformation( p - 1, q )) {
                    ++count;
                    dir.x() += -1.0;
                }
                if ( right && map->isNoInformation( p + 1, q )) {
                    ++count;
                    dir.x() += 1.0;
                }
                if ( top && map->isNoInformation( p, q + 1 )) {
                    ++count;
                    dir.y() += 1.0;
                }
                if ( bottom && map->isNoInformation( p, q - 1 )) {
                    ++count;
                    dir.y() += -1.0;
                }
                visited[cell] = true;

                // No neighbor with no information found? Cell is not part of a frontier
                if ( count == 0 )
                    continue;

                // Add point to frontier
                frontier.push_back( FrontierPoint( p, q, dir / (double)count ));
                min_x = std::min((int)p, min_x );
                max_x = std::max((int)p, max_x );
                min_y = std::min((int)q, min_y );
                max_y = std::max((int)q, max_y );

                // Visited all open and unvisited neighbor cells
                if ( top && left && !visited[cell+w-1] && map->isFree( p-1, q+1 ))
                    to_visit.push_back( cell+w-1 );
                if ( top && !visited[cell+w] && map->isFree( p, q+1 ))
                    to_visit.push_back( cell+w );
                if ( top && right && !visited[cell+w+1] && map->isFree( p+1, q+1 ))
                    to_visit.push_back( cell+w+1 );
                if ( left && !visited[cell-1] && map->isFree( p-1, q ))
                    to_visit.push_back( cell-1 );
                if ( right && !visited[cell+1] && map->isFree( p +1, q ))
                    to_visit.push_back( cell+1 );
                if ( bottom && left && !visited[cell-w-1] && map->isFree( p-1, q-1 ))
                    to_visit.push_back( cell-w-1 );
                if ( bottom && !visited[cell-w] && map->isFree( p, q-1 ))
                    to_visit.push_back( cell-w );
                if ( bottom && right && !visited[cell-w+1] && map->isFree( p+1, q-1 ))
                    to_visit.push_back( cell-w+1 );
            }

            // Frontier found?
            if ( frontier.size() == 0 )
                continue;

            // Check frontier length
            if ( map->getResolution()*(double)std::max( max_x - min_x, max_y - min_y )
                 < min_frontier_length_ )
                continue;

            // Compute direction vector
            Eigen::Vector2d dir_sum( 0, 0 );
            Eigen::Vector2d frontier_pos;
            FrontierPoint * frontier_point;
            for ( unsigned int j = 0; j < frontier.size(); ++j ) {
                frontier_point = &frontier[j];
                dir_sum += frontier_point->dir;
            }

            // Take the middle cell as first estimation for frontier position
            /// @todo this is not the middle cell!
            unsigned int middle = frontier.size() / 2;
            map->cell2point( frontier[middle].x, frontier[middle].y, frontier_pos.x(), frontier_pos.y());

            // Optimize position?
            if ( use_pos_optimization_ ) {
                Eigen::Vector2d old_pos( frontier_pos );
                pos_optimizer_.optimize( old_pos, map, frontier_pos );
            }

            // Frontier orientation
            double frontier_yaw = 0;
            if ( dir_sum.norm() > 1E-6 )
                frontier_yaw = atan2(dir_sum.y(), dir_sum.x());

            // Create and store detected frontier
            Frontier newFrontier;
            newFrontier.pose.x() = frontier_pos.x();
            newFrontier.pose.y() = frontier_pos.y();
            newFrontier.pose(2) = frontier_yaw;
            newFrontier.size = frontier.size();
            frontiers.push_back( newFrontier );
        }
    }
    return frontiers.size() > 0;
}

} // namespace
