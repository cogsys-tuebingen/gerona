#include <path_follower/utils/path.h>

//TODO: add unit test for this class
#include <path_follower/controller/robotcontroller.h>

void Path::clear()
{
    path_.clear();
    current_sub_path_ = path_.begin();
    next_waypoint_idx_ = 0;
    wp_distance_to_end_.clear();
}

void Path::setPath(const std::vector<SubPath> &path)
{
    path_ = path;
    current_sub_path_ = path_.begin();
    next_waypoint_idx_ = 0;
    computeWaypointToEndDistances();
}

void Path::registerNextWaypointCallback(NextWaypointCallback_t func)
{
    next_wp_callback_ = func;
    has_callback_ = true;
}

void Path::switchToNextSubPath()
{
    // only proceed, if there is a next sub path
    if (current_sub_path_ != path_.end()) {
        ++current_sub_path_;
        next_waypoint_idx_ = 0;
        fireNextWaypointCallback();
        computeWaypointToEndDistances();
    }
}

void Path::switchToNextWaypoint()
{
    if (!isSubPathDone()) {
        ++next_waypoint_idx_;
        fireNextWaypointCallback();
    }
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
    return current_sub_path_ == path_.end();
}

bool Path::isSubPathDone() const
{
    return next_waypoint_idx_ >= current_sub_path_->size();
}

bool Path::isLastWaypoint() const
{
    return next_waypoint_idx_ + 1 == current_sub_path_->size();
}

const SubPath &Path::getCurrentSubPath() const
{
    return *current_sub_path_;
}

const SubPath &Path::getSubPath(size_t idx) const
{
    return path_.at(idx);
}

const Waypoint &Path::getWaypoint(size_t idx) const
{
    return (*current_sub_path_)[idx];
}

const Waypoint &Path::getCurrentWaypoint() const
{
    return (*current_sub_path_)[next_waypoint_idx_];
}

const Waypoint &Path::getLastWaypoint() const
{
    return current_sub_path_->back();
}

size_t Path::getWaypointIndex() const
{
    return next_waypoint_idx_;
}

float Path::getRemainingSubPathDistance() const
{
    return wp_distance_to_end_[next_waypoint_idx_];
}

void Path::fireNextWaypointCallback() const
{
    if (has_callback_) {
        next_wp_callback_();
    }
}

void Path::computeWaypointToEndDistances()
{
    if (isDone()) {
        wp_distance_to_end_.clear();
        return;
    }

    //TODO: resize initializes every value. This is not necessary as they are overwritten anyway.
    //      Is there a more efficient way to do this?
    wp_distance_to_end_.resize(current_sub_path_->size());

    // Distance form last waypoint to end is zero (last wp == end of path)
    wp_distance_to_end_.back() = 0;
    // iterate subpath in reversed order starting with the penultimate waypoint
    for (int i = current_sub_path_->size()-2; i >= 0; --i) {
        float dist_to_next_waypoint = (*current_sub_path_)[i].distanceTo((*current_sub_path_)[i+1]);
        wp_distance_to_end_[i] = dist_to_next_waypoint + wp_distance_to_end_[i+1];
    }
}

void Path::precomputeSteerCommands(RobotController *controller)
{
    for (int i=0;i<(int)current_sub_path_->size()-1;++i) {

        controller->precomputeSteerCommand((*current_sub_path_)[i],(*current_sub_path_)[i+1]);
    }
}

std::string Path::getFrameId() const
{
    return frame_id_;
}

void Path::setFrameId(const std::string &frame_id)
{
    frame_id_ = frame_id;
}
