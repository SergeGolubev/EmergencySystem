#include "ButtonWatcher.h"

// Class to handle button presses with debounce.

static const long DebounceTimeout = 100; // 100 ms timeout
static const long LongPressTimeout = 3000; // 3 s press is a long press

CButtonWatcher::CButtonWatcher( volatile bool* _pressFlag, volatile bool* _longPressFlag ) :
	pressFlag( _pressFlag ),
	longPressFlag( _longPressFlag ),
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
		if( pressFlag != 0 ) {
			*pressFlag = true;
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
		&& longPressFlag != 0 ) // and we have callback
	{
		// catch long press
		*longPressFlag = true;
		isLongPressDetected = true;
	}
}