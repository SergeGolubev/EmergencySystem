#include "LedController.h"

CLedController::CLedController( uint8_t _ledPin ) :
	ledPin( _ledPin ),
	blinkInterval( 0 ),
	shortBlinkChangeCount( 0 ),
	shortBlinkInterval( 0 ),
	prevStateChangeMillis( 0 )
{
}

void CLedController::TurnOn()
{
	digitalWrite( ledPin, HIGH );
	blinkInterval = 0;
	shortBlinkChangeCount = 0;
}

void CLedController::TurnOff()
{
	digitalWrite( ledPin, LOW );
	blinkInterval = 0;
	shortBlinkChangeCount = 0;
}

void CLedController::Switch()
{
	digitalWrite( ledPin, !digitalRead( ledPin ) );
}

void CLedController::BlinkConstantly( int intervalMillis )
{
	blinkInterval = intervalMillis;
	shortBlinkChangeCount = 0;
}

void CLedController::BlinkShort( int intervalMillis, int blinkCount )
{
	shortBlinkInterval = intervalMillis;
	shortBlinkChangeCount = 2 * blinkCount;
}

void CLedController::UpdateState( unsigned long millis )
{
	if( shortBlinkChangeCount != 0 ) {
		if( millis - prevStateChangeMillis > shortBlinkInterval ) {
			digitalWrite( ledPin, !digitalRead( ledPin ) );
			prevStateChangeMillis = millis;
			shortBlinkChangeCount--;
		}
	} else if( blinkInterval != 0 ) {
		if( millis - prevStateChangeMillis > blinkInterval ) {
			digitalWrite( ledPin, !digitalRead( ledPin ) );
			prevStateChangeMillis = millis;
		}
	}
}