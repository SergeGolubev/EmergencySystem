#pragma once
#include <Arduino.h>

// Class to control LED (on, off, blink)

class CLedController {
public:
	CLedController( uint8_t ledPin );

	// fuctions to control LED state
	void TurnOn();
	void TurnOff();
	void BlinkConstantly( int intervalMillis );
	void BlinkShort( int intervalMillis, int blinkCount );

	// call this in loop, interval depends on desired blink resolution
	void UpdateState( unsigned long millis );

private:
	uint8_t ledPin;
	int blinkInterval; // zero if state is On or Off
	int shortBlinkChangeCount;
	int shortBlinkInterval;
	unsigned long prevStateChangeMillis;
};