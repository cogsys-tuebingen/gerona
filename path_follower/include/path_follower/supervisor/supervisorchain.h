#ifndef SUPERVISORCHAIN_H
#define SUPERVISORCHAIN_H

#include <list>
#include <path_follower/supervisor/supervisor.h>

class SupervisorChain
{
public:
    void addSupervisor(Supervisor::Ptr supervisor);

    Supervisor::Result supervise(Supervisor::State &state);

    void notifyNewGoal();
    void notifyNewWaypoint();

private:
    std::list<Supervisor::Ptr> supervisors_;
};

#endif // SUPERVISORCHAIN_H
