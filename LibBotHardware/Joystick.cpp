/*
 * Joystick.cpp
 *
 *  Created on: Dec 4, 2009
 *      Author: bohlmann, marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C++
#include <iostream>

// Workspace
#include <Global.h>

// Project
#include "Joystick.h"
#include "Robot.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Enable debug output
//#define SHOW

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace Ra;

//// class JoyButton //////////////////////////////////////////////////////////

JoyButton::JoyButton( double debounce ) :
    mDebounce( debounce ), mPressCount( 0 ), mPressed(false) {
    mLastChange.tv_sec = mLastChange.tv_usec = 0;
}

void JoyButton::setSignal( bool pressed ) {
    // Still debouncing?
    timeval now;
    gettimeofday( &now, 0 );
    if ( Misc::getTimeDiff( &mLastChange, &now ) < mDebounce ) {
        return; // Still debouncing. Nothing to do.
    }

    // Set the button state & increment the keystroke counter on a falling edge
    if ( !mPressed && pressed ) {
        mPressCount++;
    }
    mLastChange = now;
    mPressed = pressed;
}

//// class JoyAxis /////////////////////////////////////////////////////////////

void JoyAxis::setSignal( int signal ) {
    mPrevValue = mValue;
    mValue = signal;
}

//// class Joystick ////////////////////////////////////////////////////////////

Joystick::Joystick() {
    mJoy = NULL;    
}

bool Joystick::initialize() {
    // Init SDL subsystem
    SDL_InitSubSystem( SDL_INIT_JOYSTICK );

    // Check number of joysticks
    int numJoysticks = SDL_NumJoysticks();
    if ( numJoysticks < 1 ) {
        ERRORPRINT( "No joystick found!" );
        return false;
    }

    // Init joystick
    mJoy = SDL_JoystickOpen( 0 );
#ifdef SHOW
    INFOPRINT1( "Joystick: Number of axes: %d", SDL_JoystickNumAxes( mJoy ));
    INFOPRINT1( "Joystick: Number of trackballs: %d", SDL_JoystickNumHats( mJoy ));
    INFOPRINT1( "Joystick: Number of buttons: %d", SDL_JoystickNumButtons( mJoy ));
#endif
    mButtons.resize( SDL_JoystickNumButtons( mJoy ) );
    mAxes.resize( SDL_JoystickNumAxes( mJoy ));
    mAxesState.resize( mAxes.size());
    memset(&mAxesState[0],0,mAxesState.size()*sizeof(float));
    mButtonsState=0;
    return true;
}

bool Joystick::update() {
    // Update joystick
    SDL_JoystickUpdate();

    // Check buttons
    for ( Uint i = 0; i < mButtons.size(); ++i ) {
       mButtons[i].setSignal( SDL_JoystickGetButton( mJoy, i ));
    }

    // Check axes
    for ( Uint i = 0; i < mAxes.size(); ++i ) {
        mAxes[i].setSignal( SDL_JoystickGetAxis( mJoy, i ));
    }

    // update joystick state
    mButtonsState = 0x0;
    unsigned int flag= 1;
    for ( Uint i = 0; i < mButtons.size(); ++i ) {
        if (mButtons[i].isPressed()) {
            mButtonsState|=flag;
            flag = flag << 1;
        }
    }

    for ( Uint i = 0; i < mAxes.size(); ++i ) {
        mAxesState[i]=((float)mAxes[i].getValue())/JOY_AXIS_MAX;
    }


    return true;
}

void Joystick::resetButtons() {
    for ( Uint i = 0; i < mButtons.size(); ++i ) {
        mButtons[i].reset();
    }
}

void Joystick::resetAxes() {
    for ( Uint i = 0; i < mAxes.size(); ++i ) {
        mAxes[i].reset();
    }
}

void Joystick::getState(unsigned int &buttons, vector<float> &axes)
{
  if (axes.size()!=mAxesState.size()) axes.resize(mAxesState.size());
  buttons=mButtonsState;
  memcpy(&axes[0],&mAxesState[0],mAxesState.size()*sizeof(float));
}

//// class JoystickCarDriver ////////////////////////////////////////////////////////

JoystickCarDriver::JoystickCarDriver( Ramaxx &ramaxx ) {
    mRamaxx = &ramaxx;
    mJoy.initialize();
    mWingman = false;
    mEnabled = false;
    mSpeedBoost = 2.0;
    mRelativePTZ = false;
    mShootingStrength = 1.0;
    mShooting = false;
}
bool JoystickCarDriver::update() {
    if ( !mJoy.good() || !mJoy.update()) {
        return false;
    }

#ifdef SHOW
    for ( Uint i = 0; i < mJoy.getNumButtons(); ++i ) {
        if ( mJoy.getButton( i ).isPressed()) {
            cout << "Joystick: Button " << i << " pressed." << endl;
        }
    }
#endif

    // Joystick enabled?
    int enablePressCount = mJoy.getButton( mWingman ? JOY_BUTTON_ENABLE_WINGMAN : JOY_BUTTON_ENABLE ).getPressCount();
    if ( !mEnabled && enablePressCount != 0 ) {
        mEnabled = true;
        //INFOPRINT( "Joystick enabled" );
    }
    if ( !mEnabled ) {
        return true; // Not enabled yet, nothing to do
    }

    // Check steer trim
    int steerAxis = mWingman ? JOY_AXIS_STEER_WINGMAN : JOY_AXIS_STEER;
    bool l1Pressed = mJoy.getButton( mWingman ? JOY_BUTTON_L1_WINGMAN : JOY_BUTTON_L1 ).isPressed();
    bool l2Pressed = mJoy.getButton( mWingman ? JOY_BUTTON_L2_WINGMAN : JOY_BUTTON_L2 ).isPressed();
    bool r1Pressed = mJoy.getButton( mWingman ? JOY_BUTTON_R1_WINGMAN : JOY_BUTTON_R1 ).isPressed();
    bool r2Pressed = mJoy.getButton( mWingman ? JOY_BUTTON_R2_WINGMAN : JOY_BUTTON_R2 ).isPressed();

    if ( r1Pressed && !l1Pressed && !r2Pressed) {
        // Modify front steer trim
        int trim = mJoy.getAxis( steerAxis ).getValue();
        if ( trim > JOY_AXIS_ZERO_THRESHOLD )
            mRamaxx->adjustSteerFront( -5 );
        else if ( trim < -JOY_AXIS_ZERO_THRESHOLD )
            mRamaxx->adjustSteerFront( 5 );
        mRamaxx->setSteerDegrees( 0 );
    }
    if ( r2Pressed && !l1Pressed && !r1Pressed) {
        // Modify back steer trim
        int trim = mJoy.getAxis( steerAxis ).getValue();
        if ( trim > JOY_AXIS_ZERO_THRESHOLD )
            mRamaxx->adjustSteerBack( -5 );
        else if ( trim < -JOY_AXIS_ZERO_THRESHOLD )
            mRamaxx->adjustSteerBack( 5 );
        mRamaxx->setSteerDegrees( 0 );
    }

    // Adjust pan/tilt trim?
    if ( l1Pressed && ( r1Pressed || r2Pressed )) {
        // TODO Implement
    }

    // TEMPORARY shoot the marker unit?
    int panAxis = mWingman ? JOY_AXIS_PAN_WINGMAN : JOY_AXIS_PAN;
    int tiltAxis = mWingman ? JOY_AXIS_TILT_WINGMAN : JOY_AXIS_TILT;

    if ( l2Pressed && r2Pressed ) {
        mShooting = true;
        if( mJoy.getAxis(JOY_AXIS_THRUST).changed() )
            mShootingStrength = 0.5 - (mJoy.getAxis( JOY_AXIS_THRUST ).getValue() / (2.0 * JOY_AXIS_MAX));

        assert(mShootingStrength >= 0.0);
        assert(mShootingStrength <= 1.0);

        mRamaxx->getActuators()->setPosition(Actuators::ACTUATOR_EXT, (int) (1000 + mShootingStrength * 2000));

    } else if(mShooting) {
        mShooting = false;
        mRamaxx->getActuators()->setPosition(Actuators::ACTUATOR_EXT, (int) (1000));
    }

    Uint pc = mJoy.getButton( JOY_BUTTON_RELATIVE_PTZ ).getPressCount();
    if (  pc != 0 ) {
        mRelativePTZ = !mRelativePTZ;
    }

    // Move the camera?
    if ( l1Pressed ) {
        if(mRelativePTZ) {
            double pan = mRamaxx->getPanRad();
            double tilt = mRamaxx->getTiltRad();
            pan  += ( -(double)mJoy.getAxis( panAxis ).getValue()) / (double)JOY_AXIS_MAX * 0.1;
            tilt += (double)mJoy.getAxis( tiltAxis ).getValue() / (double)JOY_AXIS_MAX * 0.1;
            mRamaxx->setPanRad( pan );
            mRamaxx->setTiltRad( tilt );

        } else if ( mJoy.getAxis( panAxis ).changed() || mJoy.getAxis( tiltAxis ).changed()) {
            double angleDeg = ( -(double)mJoy.getAxis( panAxis ).getValue()) / (double)JOY_AXIS_MAX * 90;
            mRamaxx->setPanRad( angleDeg / 180.0 * M_PI );
            //mRamaxx->getRtzController()->setRollRad( M_PI*angleDeg/180.0 );
            angleDeg = (double)mJoy.getAxis( tiltAxis ).getValue() / (double)JOY_AXIS_MAX * 45;
            mRamaxx->setTiltRad( angleDeg / 180.0 * M_PI );
            //mRamaxx->getRtzController()->setTiltRad( M_PI*angleDeg/180.0 );
        }
    }

    // Check steer
    bool modifierPressed = r1Pressed || r2Pressed || l1Pressed;
    if ( !modifierPressed && mJoy.getAxis( steerAxis ).changed()) {
        double steer = (double)mJoy.getAxis( steerAxis ).getValue() / (double)JOY_AXIS_MAX;
        steer *= -25.0;
        mRamaxx->setSteerDegrees( steer );
    }

    // Check thrust
    if ( mJoy.getAxis( JOY_AXIS_THRUST ).changed()) {
        float thrust = -(float)mJoy.getAxis( JOY_AXIS_THRUST ).getValue() / (float)JOY_AXIS_MAX;
        // Speed button pressed?
        if ( mJoy.getButton( mWingman ? JOY_BUTTON_L2_WINGMAN : JOY_BUTTON_L2 ).isPressed()) {
                if ( mJoy.getButton( mWingman ? JOY_BUTTON_L1_WINGMAN : JOY_BUTTON_L1 ).isPressed()) {
                        // calibration mode
                        mRamaxx->deactivateSpeedController();
                        mRamaxx->getActuators()->setSpeed(thrust > 0.3 ? 3250 : (thrust < -0.3 ? 1250 : 2250));
                }
                else { // speed boost
                    mRamaxx->setSpeed( mSpeedBoost * thrust );
                }
        }
        else { // normal mode
                mRamaxx->setSpeed(thrust);
        }
    }

    // Check steer mode
    Uint pressCount = mJoy.getButton( JOY_BUTTON_STEERMODE ).getPressCount();
    if (  pressCount != 0 ) {
        switchSteerMode( pressCount );
    }

    // Disable speed/steer automatic mode if steer or thrust changed
    if ( mRamaxx->getCmdMode() == AUTO
        && ( mJoy.getAxis( steerAxis ).changed() || mJoy.getAxis( JOY_AXIS_THRUST ).changed())) {
        mRamaxx->setCmdMode( MANUAL_EXCEPT_PTZ );
        mJoy.resetAxes();
    }
    // Check automatic mode button
    pressCount = mJoy.getButton( JOY_BUTTON_AUTOMATICMODE ).getPressCount();
    if ( pressCount == 1 || ( pressCount % 2 ) != 0 ) {
        if ( mRamaxx->getCmdMode() != MANUAL && !l1Pressed ) {
            // Switch to manual command mode
            mRamaxx->setCmdMode( MANUAL );
        } else if ( l1Pressed && !( r1Pressed  || r2Pressed )) {
            // Switch to manaul except ptz mode
            mRamaxx->setCmdMode( MANUAL_EXCEPT_PTZ );
            mJoy.resetAxes();
        } else {
            // Switch to auto mode
            mRamaxx->setCmdMode( AUTO );
            mJoy.resetAxes();
        }
    }
    mJoy.resetButtons();

    return true;
}

void JoystickCarDriver::getJoystickState(unsigned int& buttons, vector<float>& axes)
{
  mJoy.getState(buttons,axes);
}

void JoystickCarDriver::switchSteerMode( Uint steps ) {
    RobotSteerMode mode = mRamaxx->getSteerMode();
    switch ( mode ) {
    case FRONT:
        mode = BOTH;
        break;
    case BOTH:
        mode = REAR;
        break;
    case REAR:
        mode = PARALLEL;
        break;
    default:
        mode = FRONT;
        break;
    }
    mRamaxx->setSteerMode(mode);
}

