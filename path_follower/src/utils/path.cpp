#include <path_follower/utils/path.h>


void Path::clear()
{
    path_.clear();
    current_sub_path = path_.begin();
    next_waypoint_idx_ = 0;
}

void Path::setPath(const std::vector<SubPath> &path)
{
    path_ = path;
    current_sub_path = path_.begin();
    next_waypoint_idx_ = 0;
}

void Path::registerNextWaypointCallback(NextWaypointCallback_t func)
{
    next_wp_callback_ = func;
    has_callback_ = true;
}

void Path::switchToNextSubPath()
{
    // only proceed, if there is a next sub path
    if (current_sub_path != path_.end()) {
        ++current_sub_path;
        next_waypoint_idx_ = 0;
        fireNextWaypointCallback();
    }
}

void Path::switchToNextWaypoint()
{
    if (!isSubPathDone()) {
        ++next_waypoint_idx_;
        fireNextWaypointCallback();
    }
}

void Path::switchToLastWaypoint()
{
    next_waypoint_idx_ = current_sub_path->size() - 1;
    fireNextWaypointCallback();
}

bool Path::empty() const
{
    return path_.empty();
}

size_t Path::subPathCount() const
{
    return path_.size();
}

bool Path::isDone() const
{
    return current_sub_path == path_.end();
}

bool Path::isSubPathDone() const
{
    return next_waypoint_idx_ >= current_sub_path->size();
}

bool Path::isLastWaypoint() const
{
    return next_waypoint_idx_ + 1 == current_sub_path->size();
}

const SubPath &Path::getCurrentSubPath() const
{
    return *current_sub_path;
}

const Waypoint &Path::getWaypoint(size_t idx) const
{
    return (*current_sub_path)[idx];
}

const Waypoint &Path::getCurrentWaypoint() const
{
    return (*current_sub_path)[next_waypoint_idx_];
}

const Waypoint &Path::getLastWaypoint() const
{
    return current_sub_path->back();
}

size_t Path::getWaypointIndex() const
{
    return next_waypoint_idx_;
}

void Path::fireNextWaypointCallback() const
{
    if (has_callback_) {
        next_wp_callback_();
    }
}
