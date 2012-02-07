#ifndef SENSOR_H
#define SENSOR_H

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>

// Project
#include "DataListener.h"

////////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

class Sensor : public DataOwner {
public:

    Sensor();
    virtual ~Sensor();

    enum SensorState { VALID, ERROR, TIMEOUT, NOT_INITIALIZED };

    virtual void diagnosis( int timeDiff );
    std::string getName() const { return mName; }
    SensorState getSensorState() const { return mState; }

protected:
    void setSensorState( const SensorState &state );
    std::string stateToString( const SensorState &state );

    int mValidMsgCount;
    int mTimeout;
    std::string mName;

private:
    SensorState mState;
    int mDiagnosisTimeDiff;
};

#endif // SENSOR_H
