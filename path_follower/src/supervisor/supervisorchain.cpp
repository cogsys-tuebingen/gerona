#include <path_follower/supervisor/supervisorchain.h>

using namespace std;

void SupervisorChain::addSupervisor(Supervisor::Ptr supervisor)
{
    ROS_INFO("Use Supervisor '%s'", supervisor->getName().c_str());
    supervisors_.push_back(supervisor);
}

Supervisor::Result SupervisorChain::supervise(Supervisor::State &state)
{
    list<Supervisor::Ptr>::iterator it;
    for (it = supervisors_.begin(); it != supervisors_.end(); ++it) {
        Supervisor::Result res;
        (*it)->supervise(state, &res);

        if (!res.can_continue) {
            return res;
        }
    }

    return Supervisor::Result(); // Constructor sets to can_continue = true.
}

void SupervisorChain::notifyNewGoal()
{
    list<Supervisor::Ptr>::iterator it;
    for (it = supervisors_.begin(); it != supervisors_.end(); ++it) {
        (*it)->eventNewGoal();
    }
}

void SupervisorChain::notifyNewWaypoint()
{
    list<Supervisor::Ptr>::iterator it;
    for (it = supervisors_.begin(); it != supervisors_.end(); ++it) {
        (*it)->eventNewWaypoint();
    }
}
