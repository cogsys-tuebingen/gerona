/**
 * @file GlobalPlanner.h
 * @date Jan 2012
 * @author marks
 */

#ifndef GLOBALPLANNER_H
#define GLOBALPLANNER_H

// C/C++
#include <vector>
#include <list>

// Workspace
#include <utils/LibPath/a_star/AStar.h>

namespace combined_planner {

/**
 * @class GlobalPlanner
 * @brief Uses A* to find a global path and offers methods to flatten this path.
 * The paths won't be feasible for car-like vehicles.
 */
class GlobalPlanner
{
public:
    /**
     * @brief Create object.
     * @param map The initial map. The size of this map used to initialize the
     *      the object. It's possible to use a map with a different size
     *      during runtime but it will take some computation time (memory allocation).
     */
    GlobalPlanner( lib_path::GridMap2d* map );

    /**
     * @brief Set the current map.
     * @param map The new map.
     * @attention There is no internal copy of this map! The pointer should be valid
     *       as long as this object is in use.
     */
    virtual void setMap( lib_path::GridMap2d* map );

    /**
     * @brief Try to find a path.
     * @param start Start of the path.
     * @param goal End of the path.
     * @return True if there is a path, falss otherwise.
     * @exception CombinedPlannerException If the start or the goal pose lies outside
     *      of the map.
     */
    virtual bool planPath( lib_path::Point2d start, lib_path::Point2d goal );

    /**
     * @brief Return the raw result of the latest A* search.
     * The returned path won't be flattened.
     * @param path The result will be stored in this variable.
     */
    virtual void getLatestPathRaw( std::list<lib_path::Point2d>& path ) const
        { path.assign( path_raw_.begin(), path_raw_.end()); }

    /**
     * @brief Get the latest flattened path.
     * The result will contain less points than the raw path and will be close
     * to the shortest path possible.
     * @param path The result will be written to this variable.
     */
    virtual void getLatestPath( std::list<lib_path::Point2d>& path ) const
        { path.assign( path_.begin(), path_.end()); }

    /**
     * @brief Get the latest flattened path.
     * The result will contain less points than the raw path and will be close
     * to the shortest path possible.
     * @return The latest flattened path.
     */
    virtual const std::vector<lib_path::Point2d>& getLatestPath() const
        { return path_; }

protected:

    /**
     * @brief Check if the straight line between to points contains only open cells.
     * This method is used for path flattening.
     * @brief p1 Cell-coordinates of the line start.
     * @brief p2 Cell-coordinates of the line ending.
     * @return False if there is an obstacle. True otherwise.
     */
    virtual bool isLineFree( const lib_path::waypoint_t& p1,
                             const lib_path::waypoint_t& p2 ) const;

    /**
     * @brief Flatten the given path.
     * This methos tries to remove as many points as possible from the raw path.
     * It's possible to remove a point if there are no obstacles on the stright line between
     * the previous and the next point.
     * @param raw A* path in cell-coordinates.
     * @param flattened The result will be written to this parameter. The list won't be cleared
     *      and it won't contain the start or goal position. The entries of this list
     *      are map coordinates (not cell coordinates).
     */
    virtual void flatten( const lib_path::path_t* raw, vector<lib_path::Point2d> &flattened ) const;

private:
    /// The map we are working on
    lib_path::GridMap2d* map_;

    /// A* search object
    lib_path::AStar a_star_;

    /// Start of path in map coordinates
    lib_path::Point2d start_;

    /// Goal in map coordinates
    lib_path::Point2d goal_;

    /// Latest planned path
    std::vector<lib_path::Point2d> path_;

    /// Latest planned path (not flattened etc)
    std::list<lib_path::Point2d> path_raw_;
};

} // namespace

#endif // GLOBALPLANNER_H
