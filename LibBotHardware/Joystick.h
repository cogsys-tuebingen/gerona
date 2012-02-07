/*
 * Joystick.h
 *
 *  Created on: Dec 4, 2009
 *      Author: marks
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

///////////////////////////////////////////////////////////////////////////////
// PROTOTYPES
///////////////////////////////////////////////////////////////////////////////

class JoystickCarDriver;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <time.h>
#include <vector>

// SDL
#include <SDL/SDL.h>

// Workspace
#include "Misc.h"

// Project
#include "Ramaxx.h"




///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

#define JOY_AXIS_ZERO_THRESHOLD     10000   // All values smaller than this value
                                            // will be treated as zero if the axis
                                            // modifies the trim of a servo.

#define JOY_BUTTON_AUTOMATICMODE    0       // Button 1 (A on WingMan controller)
#define JOY_BUTTON_RELATIVE_PTZ     1       // Button 2
#define JOY_BUTTON_STEERMODE        2       // Button 3 (C on WingMan controller)
#define JOY_BUTTON_L1               4       // Button 4 (L1 on WingMan controller)
#define JOY_BUTTON_L1_WINGMAN       6
#define JOY_BUTTON_L2               6       // Button 7 (L2 on WingMan controller)
#define JOY_BUTTON_L2_WINGMAN       9
#define JOY_BUTTON_R1               5       // Button 6 (R1 on WingMan controller)
#define JOY_BUTTON_R1_WINGMAN       7
#define JOY_BUTTON_R2               7       // Button 8 (R2 on WingMan controller)
#define JOY_BUTTON_R2_WINGMAN       10
#define JOY_BUTTON_ENABLE           8       // Button 9 (Start on WingMan controller)
#define JOY_BUTTON_ENABLE_WINGMAN   8

#define JOY_AXIS_MAX                32768.0 // Maximum value of an axis
#define JOY_AXIS_THRUST             1       // Left stick forward-backward
#define JOY_AXIS_STEER              2       // Right stick left-right
#define JOY_AXIS_STEER_WINGMAN      3
#define JOY_AXIS_PAN                2
#define JOY_AXIS_PAN_WINGMAN        3
#define JOY_AXIS_TILT               3       // Right stick forward-backward
#define JOY_AXIS_TILT_WINGMAN       4

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Represents a joystick button. Debounces the readout values from the joystick.
 */
class JoyButton {
public:
    /**
     * Contructor.
     *
     * @param debounce Debounce time in sec.
     */
    JoyButton( double debounce = 0.02 );

    /**
     * Sets the readout joystick value for this button.
     *
     * @param The readout value. True if the button is pressed.
     */
    void setSignal( bool pressed );

    /**
     * Returns the debounced button state.
     *
     * @return True if the button is pressed.
     */
    bool isPressed() const { return mPressed; }

    /**
     * Returns the number of keystrokes since the last reset.
     *
     * @return The number of keystrokes since the last reset.
     */
    Uint getPressCount() const { return mPressCount; }

    /**
     * Resets the number of recognized keystrokes.
     */
    void reset() { mPressCount = 0; }

private:
    /** The state of the button. True if its pressed. */
    bool mPressed;

    /** Debounce time in msec. */
    double mDebounce;

    /** Time of the last alteration of the button state. */
    timeval mLastChange;

    /** Number of keystrokes since the last reset. */
    Uint mPressCount;
};

/**
 * Represents a joystick axis.
 */
class JoyAxis {
public:
    /**
     * Sets the value of the axis.
     *
     * @param signal The value.
     */
    void setSignal( int signal );

    /**
     * Returns the current position of the axis.
     *
     * @return The position of the axis.
     */
    int getValue() const { return mValue; }

    /**
     * Returns true if the value has changed since the last reset.
     */
    bool changed() const { return mValue != mPrevValue; }

    /**
     * Resets the axis.
     */
    void reset() { mPrevValue = mValue; }

private:
    /** The current value of the axis. */
    int mValue;
    /** Previous value of the axis. */
    int mPrevValue;
};

/**
 * Reads the input signal from a joystick device.
 */
class Joystick
{
public:
    /**
     * Default constructor.
     */
    Joystick();

    /**
     * Initializes the joystick.
     *
     * @return False if no joysticks are present.
     */
    bool initialize();

    /**
     * @return True if a joystick device is present.
     */
    bool good() const { return mJoy != NULL; }

    /**
     * Reads the joystick input data.
     *
     * @return False if an error occurred.
     */
    bool update();

    /**
     * Resets all buttons.
     */
    void resetButtons();

    /**
     * Resets all axes.
     */
    void resetAxes();

    /**
     * Returns the number of buttons.
     *
     * @return The number of buttons.
     */
    Uint getNumButtons() const { return mButtons.size(); }

    /**
     * Returns the number of axes.
     *
     * @return The number of axes.
     */
    Uint getNumAxes() const { return mAxes.size(); }

    /**
     * Returns button i.
     *
     * @param i The index of the button.
     *
     * @return Button number i.
     */
    JoyButton& getButton( Uint i ) { return mButtons[i]; }

    /**
     * Returns axis i.
     *
     * @param i The index of the axis.
     *
     * @return Joystick axis i.
     */
    JoyAxis& getAxis( Uint i ) { return mAxes[i]; }

    void getState(unsigned int& buttons, vector<float>& axes);
private:
    /** The joystick device. */
    SDL_Joystick * mJoy;
    /** The buttons. */
    vector<JoyButton> mButtons;
    /** The joystick axes. */
    vector<JoyAxis> mAxes;

    unsigned int mButtonsState;
    vector<float> mAxesState;
};

/**
 * Monitors the input data from a joystick and issues the corresponding commands
 * (e.g drive forward, steer left).
 */
class JoystickCarDriver {
public:
    /**
     * Constructor.
     *
     * @param ramaxx The driver object.
     */
    JoystickCarDriver( Ramaxx &ramaxx);

    /**
     * Updates the joystick and interpretes the input data.
     */
    bool update();


    /**
     * Set this flag true if the joystick is a WingMan controller.
     *
     * @param wingman True if the joystick is a WingMan controller. False
     *      if its the default Logitech controller.
     */
    void setWingMan( bool wingman ) { mWingman = wingman; }

    void setSpeedBoost (double factor) {mSpeedBoost = factor;}

    void getJoystickState(unsigned int& buttons, vector<float>& axes);
private:
    /**
     * Switches the steer mode.
     *
     * @param steps Number of steps.
     */
    void switchSteerMode( Uint steps );

    /**
     * Sets the manual mode flag.
     *
     * @param mode True to enable the manual mode.
     */
    void setManualMode( bool mode );

    /** The joystick. */
    Joystick mJoy;


    /** The robot driver object. */
    Ramaxx * mRamaxx;

    /** True if the joystick is a WingMan controller. */
    bool mWingman;
    /** True if the joystick is enabled (button 10 was pressed). */
    bool mEnabled;

    /**
	boost factor when "speed-button" on joystick is pressed
	*/
    double mSpeedBoost;

    /** Move the PTZ relative or absolute? */
    bool mRelativePTZ;

    /** Shooting strength */
    double mShootingStrength;
    /** Shooting?*/
    bool mShooting;
};

#endif /* JOYSTICK_H_ */
