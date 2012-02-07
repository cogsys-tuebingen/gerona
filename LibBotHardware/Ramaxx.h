/*
 * @fileRamaxx.h
 *
 * @date Jul 14, 2009 early 21st century
 * @author bohlmann, marks
 */

#ifndef RAMAXX_H_
#define RAMAXX_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class Ramaxx;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <map>
#include <math.h>
#include <iostream>

// Workspace
#include <Global.h>
#include <Stopwatch.h>

// Project
#include "QuMessage.h"
#include "Odometry.h"
#include "WheelEncoder.h"
#include "SteerServo.h"
#include "SpeedController.h"
#include "Compass.h"
#include "UsbConn.h"
#include "RobotSensors.h"
#include "Adns.h"
#include "Actuators.h"
#include "PtzController.h"
#include "Robot.h"
#include "Avr32UiHandler.h"
#include "Avr32Params.h"
#include "RtzController.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class Ramaxx {
public:
    /** Available speed controller types */
    enum SpeedCtrlType { SIMPLE, PID, AVR32 };

    /**
     * Initializes the driver object (singleton pattern).
     */
    static void initialize();

    // TODO Singleton pattern unused
    /**
     * Returns the driver object (singleton pattern).
     *
     * @return
     *      The driver object.
     */
    static Ramaxx* getRamaxx() { return mRamaxx; }

    /**
     * Destructor.
     */
    virtual ~Ramaxx();

    /**
     * Open the connection to the robot.
     */
    bool open();

    /**
     * Close the connection to the robot.
     */
    int close();

    /**
     * Is the connection to the robot open?
     *
     * @return
     *      True if the connection is open.
     */
    bool isOpen() { return mUsbConn.isOpen(); }

    /**
     * Resets the robot watchdog.
     *
     * @return
     *      1 if a reset was necessary and the command succeeded, -1 if
     *      a reset was necessary but the command failed, 0 if a
     *      reset is not necessary.
     */
    int resetRobotWatchdog();


    /**
     * Turns of speed controller, i.e. for debugging purposes
     */
    void deactivateSpeedController();

    /**
     * display text on the onboard display
     * @param lineNr nr of line where text should appear
     */
    void displayText( const string& text, int lineNr = 0 );

    /**
     * Let ramaxx beep. Plug up your ears.
     */
    void beep( const int *beeps );

    /**
     * Read all messages from the robot and notify all listeners.
     *
     * @return
     *      True if no error occurred.
     */
    bool processRobotMsgs();

    /**
     * updates e.g. dead reckoning position
     */
    void update ();

    /**
     * Adjusts the calibration of the front steer servo.
     *
     * @param delta The difference between the old calibration value and the new calibration.
     */
    void adjustSteerFront( int delta );

    /**
     * Adjusts the calibration of the front steer servo.
     *
     * @param delta The difference between the old calibration value and the new calibration.
     */
    void adjustSteerBack( int delta );

    /**
     * Adjust pan servo zero position.
     */
    void adjustPan( int delta );

    /**
     * Adjust tilt servo zero position.
     */
    void adjustTilt( int delta );

    /**
     * Enqeues a message. Call sendMsgs() to send all enqueued messages to the robot.
     *
     * @param msg The message.
     */
    bool queueMsg(const QuMessage& msg ) { return mUsbConn.queueMsg( msg ); }

    /**
     * Register a message handler. The handler will receive messages from the robot.
     *
     * @param msgType Type of the message that will be relayed to the handler.
     * @param handler The message handler.
     */
    void addQuMsgHandler( U8 msgType, QuMsgHandler &handler );

    /**
     * Register a message handler. The handler will receive messages from the robot.
     *
     * @param msgType Types of the messages that will be relayed to the handler.
     * @param length Size of msgTypes.
     * @param handler The message handler.
     */
    void addQuMsgHandler( U8 msgTypes[], size_t length, QuMsgHandler &handler );

    /**
     * Register a message handler. The handler will receive all messages.
     *
     * @param handler The message handler.
     */
    void addQuMsgHandler( QuMsgHandler &handler );

//// Getter ////////////////////////////////////////////////////////////////////

    /**
     * @return Current steering mode (front, back, parallel,..steer)
     */
    Ra::RobotSteerMode getSteerMode () { return mSteerMode; }

    /**
     * Return kinematic state of robot.
     *
     * @param speed Velocity as reported by odometry [m/s]
     * @param steerFront Front steer angle [deg]
     * @param steerRear Rear steer angle [deg]
     *
     */
     void getKinematicState( float& speed, float& steerFront, float& steerRear );

     /**
        return UI display object
       */
     Avr32UiHandler *getUiHandler() {return &mAvrUi;}

    /**
     * Return all sensors.
     *
     * @return
     *      The sensors.
     */
    RobotSensors * getSensors() { return &mSensors; }

    double getTargetSpeed() {return mSpeed->getSpeed();}


    /**
     * Returns the odometry object.
     *
     * @return The odometry object.
     */
    Odometry * getOdometry()  { return mOdo; }

    /**
     * Gets the steering angle of the robot in degrees.
     */
    float getSteerDegrees();

    /**
     * Writes the current odometry data to the given variable.
     *
     * @param pos The current position will be written to this variable.
     */
    void getOdoPos( Vector3d& pos );

    /**
     * Writes the current uncorrected odometry data to the given variable.
     *
     * @param pos The current position will be written to this variable.
     */
    void getPlainOdoPos( Vector3d& pos );

    /**
     * Writes the current velocity to the given variable.
     *
     * @param vel The current velocity will be written to this parameter.
     */
    void getOdoVelocity( Vector3d &vel );

    SteerServo* getSteerFrontController() { return &mSteerFront; }
    SteerServo* getSteerRearController() { return &mSteerBack; }

    /**
     * Returns a pointer to actuators.
     */
    Actuators* getActuators();

    /**
     * returns current pan angle
     */
    float getPanRad() const;

    /**
     * returns current tilt angle
     */
    float getTiltRad() const;

    /**
     * Rear left encoder active?
     *
     * @return True if the rear left encoder is enabled.
     */
    bool hasEncoderLeft();

    /**
     * Rear right encoder active?
     *
     * @return True if the rear right encoder is enabled.
     */
    bool hasEncoderRight();

    bool hasFrontHallSensors() const {return mSteerFront.usesHallSensors();}

    bool hasRearHallSensors() const {return mSteerBack.usesHallSensors();}

    /**
     * Returns if the rear axis is steerable.
     *
     * @return True if the rear axis is steerable
     */
    bool isSteerBackAvailable() const { return mBackSteerAvail; }

    /**
     * Returns the ptz controller.
     *
     * @return The ptz controller.
     */
    PtzController* getPtzController() { return &mPtz; }

    /**
     * Returns the laser roll/tilt unit controller.
     *
     * @return Pointer to the roll/tilt controller.
     */
    RtzController* getRtzController() { return &mRtz; }

//// Setter ////////////////////////////////////////////////////////////////////

    /**
     * Set type of the speed controller. Attention: You have
     * to configure the new speed controller again!
     *
     * @param type The new speed controller type.
     */
    void setSpeedCtrlType( SpeedCtrlType type );

    /**
     * Sets the front steer servo angle.
     *
     * @param angle The new servo angle.
     */
    void setSteerFrontDegrees( float angle );

    /**
     * Sets the back steer servo angle.
     *
     * @param angle The new servo angle.
     */
    void setSteerBackDegrees( float angle );

    /**
     * Sets the steering angle of the robot.
     *
     * @param angle Steering angle in degrees (front/back/both depends on)
     */
    void setSteerDegrees( float angle );

    /**
     * Sets speed of robot.
     *
     * @param speed in m/sec
     */
    void setSpeed( float speed );

    /**
     * Pans the camera (turn sideways)
     *
     * @param angle target angle in rad, 0 is looking forward, pi/2 is 90deg left
     */
    void setPanRad( float angle );

    /**
     * Tilts the camera (turn updown)
     *
     * @param angle target angle in rad, 0 is looking forward, pi/4 is 45deg up
     */
    void setTiltRad( float angle );

    /**
     * Sets the current odometry position.
     *
     * @param pos
     *      The new position.
     */
    void setOdoPos( const Vector3d& pos );

    /**
     * Sets steering mode, i.e. front steer, back steer, parallel steer, ...
     * Its not possible to change the steer mode if the rear axis is not steerable.
     */
    void setSteerMode( const Ra::RobotSteerMode& mode );

    /**
     * Sets if the rear axis is steerable. If its not steerable, the only
     * available steer mode is "front steer".
     *
     * @param available Is the rear axis steerable?
     */
    void setBackSteerAvailable( bool available );

    /**
     * Sets the speed controller calibration data.
     *
     * @param calib The new calibrations data.
     */
    void setSpeedCtrlCalibration( const SpeedCtrlCalibration &calib );

    /**
     * Sets the ptz unit calibration data.
     *
     * @param calib The new calibration data.
     */
    void setPtzCalibration( const PtzCalibration &calib );

    /**
     * Sets the configuration of the front steer servo controller.
     *
     * @param front Front steer servo configuration.
     */
    void setFrontSteerServoConfig( const SteerServoConfig &front );

    /**
     * Sets the configuration of the rear steer servo controller.
     *
     * @param rear Rear steer servo configuration.
     */
    void setRearSteerServoConfig( const SteerServoConfig &rear );

    /**
     * Returns the current command mode.
     *
     * @return The current command mode.
     */
    Ra::RobotCmdMode getCmdMode() const { return mCmdMode; }

    /**
     * Sets the command mode.
     *
     * @param mode New command mode.
     */
    void setCmdMode( const Ra::RobotCmdMode &mode );

    /**
     * Returns settings for logfile control, as set on robot UI.
     *
     * @param[out] isLogging log enabled
     * @param[out] logSuffix suffix for logfile name
     */
    void getLogState(bool& isLogging, int& logSuffix);

    /**
     * @param isLogging true if log is running
     * @param fileNumber Current logfile number
     * @param size current size of logfile in bytes
     */
    void setLogInfo( bool isLogging, U16 fileNumber, U32 size );

private:
    /**
     * Standard constructor.
     */
    Ramaxx();

    /** The driver object (singleton pattern). */
    static Ramaxx*          mRamaxx;
    /** Object to handle the USB connection to the robot. */
    RamaxxConnection        mUsbConn;
    /** The robots actuators. */
    Actuators               mActuators;
    /** Sensors. */
    RobotSensors            mSensors;
    /** Current steering mode. */
    Ra::RobotSteerMode      mSteerMode;

    /** Represents the front steer servo. */
    FrontSteerServo              mSteerFront;
    /** Represents the back steer servo. */
    BackSteerServo              mSteerBack;
    /** Points to the object doing odometry. */
    Odometry *              mOdo;
    /** Odometry using compass and wheel encoder. */
    RamaxxOdometry          mRamaxxOdo;
    /** Represents the speed controller. */
    SpeedController *       mSpeed;
    /** Ptz unit controller. */
    PtzController           mPtz;
    /** Rtz unit controller */
    RtzController           mRtz;

    double                  mOldDistance;
    /** Time of the last robot watchdog reset. */
    timeval                 mLastWdtReset;
    /** Rear axis steerable? Defaults to false. */
    bool                    mBackSteerAvail;
    /** Used for sensor state diagnosis */
    Stopwatch               mDiagnosisTimer;
    /** Sensor diagnosis interval [ms] */
    int                     mDiagnosisInterval;

    /** Command mode */
    Ra::RobotCmdMode mCmdMode;

    /** Avr32 user interface handler */
    Avr32UiHandler mAvrUi;

    /** Avr32 parameter handler */
    Avr32Params mAvrParams;
};

#endif /* RAMAXX_H_ */
