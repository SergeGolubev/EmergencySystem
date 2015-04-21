#include "ButtonWatcher.h"

// Class to handle button presses with debounce.

static const long DebounceTimeout = 100; // 100 ms timeout
static const long LongPressTimeout = 3000; // 3 s press is a long press

CButtonWatcher::CButtonWatcher( TButtonPressCallback _pressCallback, TButtonPressCallback _longPressCallbac ) :
	pressCallback( _pressCallback ),
	longPressCallbac( _longPressCallbac ),
	prevState( false ),
	isButtonPressed( false ),
	isLongPressDetected( false ),
	timeDown( 0 ),
	timeUp( 0 )
{
}

void CButtonWatcher::UpdateButtonState( bool isPressed )
{
	if( isPressed != prevState ) {
		if( isPressed ) {
			onPress();
		} else {
			onRelease();
		}
	} else {
		onSameState();
	}

	prevState = isPressed;
}

void CButtonWatcher::onPress()
{
	if( !isButtonPressed ) {
		isButtonPressed = true;
		timeDown = millis();
		if( pressCallback != 0 ) {
			pressCallback();
		}
	}
}

void CButtonWatcher::onRelease()
{
	timeUp = millis();
}

void CButtonWatcher::onSameState()
{
	long time = millis();
	if( isButtonPressed // we consider button is pressed
		&& !prevState  // but actual button is released
		&& ( time - timeUp ) > DebounceTimeout ) // for long enough time
	{
		// consider button released
		isButtonPressed = false;
		isLongPressDetected = false;
	} else if( isButtonPressed // we consider button is pressed
		&& !isLongPressDetected // and we didn't catch long press yet
		&& ( time - timeDown ) > LongPressTimeout // and button is pressed long enough
		&& longPressCallbac != 0 ) // and we have callback
	{
		// catch long press
		longPressCallbac();
		isLongPressDetected = true;
	}
}