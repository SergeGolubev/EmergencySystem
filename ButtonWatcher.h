#pragma once
#include <Arduino.h>

// Class to handle button presses with debounce.

class CButtonWatcher {
public:
	// watcher sets report flags when detects press or long press
	// user can then check flag state, do some actions, reset flag etc.
	CButtonWatcher( volatile bool* pressFlag, volatile bool* longPressFlag );

	// call this every time the pin state changes and time to time in a loop
	void UpdateButtonState( bool isPressed );
	
private:
	volatile bool* pressFlag;
	volatile bool* longPressFlag;

	volatile bool prevState; // actual state of button pin
	volatile bool isButtonPressed; // do WE consider button pressed
	volatile bool isLongPressDetected;
	volatile long timeDown;
	volatile long timeUp;

	inline void onPress();
	inline void onRelease();
	inline void onSameState();
};
