#ifndef CALIBRATIONMISSION_H
#define CALIBRATIONMISSION_H
#include "Stopwatch.h"

class RobotMission {
public:

    virtual ~RobotMission() {};

    /**
     * @return False if any severe error occured.
     */
    virtual bool execute() = 0;

protected:
    Stopwatch mission_timer_;
};

#endif // CALIBRATIONMISSION_H
