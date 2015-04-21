#pragma once
#include <Arduino.h>

// Class to handle button presses with debounce.

// Callback is called when button press is detected or long press is detected.
typedef void (*TButtonPressCallback)();

class CButtonWatcher {
public:
	// callbacks can be zero
	CButtonWatcher( TButtonPressCallback pressCallback, TButtonPressCallback longPressCallbac );

	// call this every time the pin state changes and time to time in a loop
	void UpdateButtonState( bool isPressed );
	
private:
	TButtonPressCallback pressCallback;
	TButtonPressCallback longPressCallbac;

	volatile bool prevState; // actual state of button pin
	volatile bool isButtonPressed; // do WE consider button pressed
	volatile bool isLongPressDetected;
	volatile long timeDown;
	volatile long timeUp;

	inline void onPress();
	inline void onRelease();
	inline void onSameState();
};
