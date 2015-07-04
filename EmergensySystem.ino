#include <util/atomic.h>
#include <EEPROM.h>
#include "ButtonWatcher.h"
#include "LedController.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ESC input
#define ESC_IN_PIN 8 // ESC inpit pin
#define ESC_IN_PIN_REG PINB // pin 8 is ATmega pin B0 in port B
#define ESC_IN_PIN_FLAG PINB0 // pin B0

#define ESC_IN_UPDATE_INTERVAL 20 // 20 ms

// Heartbeat input
#define HEARTBEAT_IN_PIN 5
#define HEARTBEAT_IN_PIN_REG PIND
#define HEARTBEAT_IN_PIN_FLAG PIND5
#define HEARTBEAT_TIMEOUT 100 // max timeout between heartbeat pulses in milliseconds
#define HEARTBEAT_CAPTURE_TIMEOUT 1000 // in milliseconds
#define HEARTBEAT_UPDATE_INTERVAL HEARTBEAT_TIMEOUT / 4

// ESC outpit
#define ESC_OUT_PIN 9 // ESC output pin (this is OC1A ATmega pin)
#define ESC_OUT_REG OCR1A // register to write output

// Servo output
#define SRV_OUT_PIN 10 // servo output pin (this is OC1B ATmega pin)
#define SRV_OUT_REG OCR1B

// TOP value for timer1, used for servo input and output
// 16 MHz with prescaler 8 give 0,5 mks tacts
// 20 ms servo update period - 40000 tacts
#define TIMER1_TOP 40000

// Servo control button
#define BUTTON_SRV_PIN 3
#define BUTTON_SRV_REG PIND
#define BUTTON_SRV_FLAG PIND3

// Arm/Disarm button
#define BUTTON_ARM_PIN 4
#define BUTTON_ARM_REG PIND
#define BUTTON_ARM_FLAG PIND4

// Button state update interval
#define BUTTON_UPDATE_INTERVAL 50 // 50 ms

// Max/Min values for servo input and output in misroseconds
#define SERVO_MAX 2100
#define SERVO_MIN 900

// Servo state LED
#define SRV_LED_PIN 6
CLedController servoLedController( SRV_LED_PIN );

// Arm/Disarm LED
#define ARM_LED_PIN 7
CLedController armLedController( ARM_LED_PIN );

// On-board LED
#define BOARD_LED_PIN 13
CLedController boardLedController( BOARD_LED_PIN );

// LED state update interval
#define LED_UPDATE_INTERVAL 10 // 10 ms

// EEPROM addresses
#define ESC_MIN_ADDRESS 0 // min ECS out value
#define SERVO_OPEN_ADDRESS sizeof( int ) // servo open position
#define SERVO_CLOSE_ADDRESS 2 * sizeof( int ) // servo close position

// servo and ESC calibration values
unsigned int servoOpenPos = SERVO_MIN;
unsigned int servoClosePos = SERVO_MAX;
unsigned int escMinPos = SERVO_MIN;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned int readEEPROM( int address )
{
	byte hi = EEPROM.read( address );
	byte lo = EEPROM.read( address + 1 );
	return word( hi, lo );
}

void writeEEPROM( int address, unsigned int value )
{
	EEPROM.write( address, highByte( value ) );
	EEPROM.write( address + 1, lowByte( value ) );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline unsigned int restrictServoValue( unsigned int servoValue )
{
	return min( max( servoValue, SERVO_MIN ), SERVO_MAX );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Operation state

enum TState {
	S_DISARM_OPEN,
	S_DISARM_CLOSE,
	S_ARMED,
	S_FIRED,
	S_CAL_ESC,
	S_CAL_OPEN,
	S_CAL_CLOSE
};

TState state = S_DISARM_OPEN;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Output PWM control

// Set output ESC signal is microseconds
void setEscOut( unsigned int escValue )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		// resolution is 0,5 mks so multiply by 2
		ESC_OUT_REG = 2 * restrictServoValue( escValue );
	}
}

// Set output servo signal is microseconds
void setServoOut( unsigned int servoValue )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		// resolution is 0,5 mks so multiply by 2
		SRV_OUT_REG = 2 * restrictServoValue( servoValue );
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ESC input processing

// timer overflow count since ESC pilse started
volatile uint8_t overflowCount = 0; 
// read esc input value in microseconds
volatile unsigned int escInput = SERVO_MIN; 

// Hande ESC input signal change (if any)
inline void processEscInput()
{
	// read counter value as soon as possible
	const unsigned int counter = TCNT1;

	// prev state as bit flag in pin register
	static uint8_t prevState = 0;

	// current state as bit flag
	uint8_t state = ( ESC_IN_PIN_REG & _BV( ESC_IN_PIN_FLAG ) );
	if( !( state ^ prevState ) ) {
		// if no state schange - nothing to do
		return;
	}
	prevState = state;

	static unsigned int pulseStart = counter;
	
	if( state ) { // change from LOW to HIGH
		pulseStart = counter;
		overflowCount = 0;
	} else { // change from HIGH to LOW
		// calc pulse lenght
		unsigned int pulseLen = 0xFFFF;
		// there is possibility that counter overflows in this ISR before counter value is read
		// in this case overflowCount will not be incremented
		// so check the values of counter and pulseStart
		if( overflowCount == 0 && counter > pulseStart ) {
			// overflow didn't happen
			pulseLen = counter - pulseStart;
		} else if( overflowCount == 1 && counter < pulseStart ) {
			// single overflow happened and pulse is not too long
			pulseLen = ( TIMER1_TOP - pulseStart ) + counter;
		} // if more than one overflow happened - pulse is too long
		
		// if pulse is neither too long nor to short - update input value
		if( pulseLen < SERVO_MAX * 4 && pulseLen > SERVO_MIN ) {
			// restrict the value anyway
			escInput = restrictServoValue( pulseLen / 2 );
		}		
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Heartbeat input processing

volatile bool hasHeartbeat = false;

inline void onHeatbeatCapture()
{
	boardLedController.TurnOn();
}

inline void onHeartbeatLost()
{
	boardLedController.TurnOff();
	if( state == S_ARMED ) {
		setEscOut( escMinPos ); // block motor
		setServoOut( servoOpenPos ); // fire parachute
		armLedController.BlinkConstantly( 400 );
		servoLedController.TurnOff();
		state = S_FIRED;
	}	
}

inline void processHeartbeat()
{
	unsigned long currentMillis = millis();
	static unsigned long prevPulseMillis = 0; // time of previous heartbeat pulse
	static unsigned long pulsesStartMillis = 0; // time of first pulse

	// check if heartbeat is lost
	if( prevPulseMillis != 0 && currentMillis - prevPulseMillis > HEARTBEAT_TIMEOUT ) {
		// too long time passed from prev pulse
		hasHeartbeat = false;
		prevPulseMillis = 0;
		pulsesStartMillis = 0;
	}

	// check state change
	// pullup is enabled -> deafult state is HIGH
	static uint8_t prevState = _BV( HEARTBEAT_IN_PIN_FLAG );
	uint8_t state = ( HEARTBEAT_IN_PIN_REG & _BV( HEARTBEAT_IN_PIN_FLAG ) );
	if( !( state ^ prevState ) ) {
		// if no state schange - nothing else to do
		return;
	}
	// signal is changed on heartbeat pin
	prevState = state; // remember prev state
	prevPulseMillis = currentMillis; // remember pulse time

	if( !hasHeartbeat && pulsesStartMillis != 0 && currentMillis - pulsesStartMillis > HEARTBEAT_CAPTURE_TIMEOUT ) {
		// have heartbeat pulses for long enough time
		hasHeartbeat = true;
	}

	// remember time of first pulse in sequence
	if( pulsesStartMillis == 0 ) {
		pulsesStartMillis = currentMillis;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Button processing

volatile bool armButtonPress = false;
volatile bool armButtonLongPress = false;
static CButtonWatcher armDisarmButtonWatcher( &armButtonPress, &armButtonLongPress );

volatile bool servoButtonPress = false;
volatile bool servoButtonLongPress = false;
static CButtonWatcher servoButtonWatcher( &servoButtonPress, &servoButtonLongPress );

inline void onArmButtonPress()
{
	// we can arm only from S_DISARM_CLOSE if heartbeat signal is present
	// if this condition is not met - blink LED quickly as a warning
	if( state != S_ARMED && state != S_FIRED && ( state != S_DISARM_CLOSE || !hasHeartbeat ) ) {
		armLedController.BlinkShort( 100, 5 );
	}
}

inline void onArmButtonLongPress()
{
	
	switch( state ) {
		case S_DISARM_CLOSE:
			// arm if servo is closed and we have heartbeat
			if( hasHeartbeat ) {
				armLedController.TurnOn();
				state = S_ARMED;
			}
			break;
		case S_ARMED: 
			// disarm to S_DISARM_CLOSE
			setEscOut( escMinPos );
			armLedController.TurnOff();
			state = S_DISARM_CLOSE;
			break;
		case S_FIRED:
			// disarm to S_DISARM_OPEN
			setEscOut( escMinPos );
			armLedController.TurnOff();
			state = S_DISARM_OPEN;
			break;
	}
}

inline void onServoButtonPress()
{
	switch( state ) {
		case S_DISARM_OPEN:
			// close servo
			setServoOut( servoClosePos );
			servoLedController.TurnOn();
			state = S_DISARM_CLOSE;
			break;
			
		case S_DISARM_CLOSE:
			// open servo
			setServoOut( servoOpenPos );
			servoLedController.TurnOff();
			state = S_DISARM_OPEN;
			break;
		case S_CAL_ESC:
			// remember esc min pos and go to servo open pos calibration
			servoLedController.BlinkShort( 100, 5 );
			escMinPos = escInput;
			writeEEPROM( ESC_MIN_ADDRESS, escMinPos );
			state = S_CAL_OPEN;
			break;

		case S_CAL_OPEN:
			// remember servo open pos
			servoLedController.BlinkShort( 100, 5 );
			servoOpenPos = escInput;
			writeEEPROM( SERVO_OPEN_ADDRESS, servoOpenPos );
			state = S_CAL_CLOSE;
			break;

		case S_CAL_CLOSE: 
			// remember servo close pos
			servoLedController.TurnOn();
			servoLedController.BlinkShort( 100, 5 );
			servoClosePos = escInput;
			writeEEPROM( SERVO_CLOSE_ADDRESS, servoClosePos );
			state = S_DISARM_CLOSE;
			break;			
	}
}

inline void onServoButtonLongPress()
{
	if( state == S_DISARM_OPEN || state == S_DISARM_CLOSE ) {
		// switch to ESC min pos calibration mode
		servoLedController.BlinkConstantly( 800 );
		servoLedController.BlinkShort( 100, 5 );
		state = S_CAL_ESC;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pass captured ESC input to ESC output or Servo output

void passThroughEscInput()
{
	// if armed or in ESC min pos calibration - pass ESC input to ESC output
	if( state == S_ARMED || state == S_CAL_ESC ) {
		setEscOut( escInput );
	}

	// calibration - pass ESC input to Servo output
	if( state == S_CAL_OPEN || state == S_CAL_CLOSE ) {
		setServoOut( escInput );
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupts

// Pin change interrupt 0 routine
ISR( PCINT0_vect )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
	{
		processEscInput();
	}
}

// Pin change interrupt 2 routine
ISR( PCINT2_vect )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
	{
		// process heartbeat signal
		processHeartbeat();
		// process button presses
		armDisarmButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_ARM_REG, BUTTON_ARM_FLAG ) );
		servoButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_SRV_REG, BUTTON_SRV_FLAG ) );
	}
}

// Timer overflow interrupt routine
ISR( TIMER1_OVF_vect )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		if( overflowCount < 2 ) {
			overflowCount++;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup

// Setup 16-bit timer\counter1 for ESC\servo output
void setupTimer1()
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		// Fast PWM mode, TOP is ICR1
		// enable output from OCR1A and OCR1B
		// prescaler 1/8
		TCCR1A = _BV( WGM11 ) | _BV( COM1A1 ) |_BV( COM1B1 );
		TCCR1B = _BV( CS11 ) | _BV( WGM12 ) | _BV( WGM13 );

		// set TOP value
		ICR1 = TIMER1_TOP;

		ESC_OUT_REG = escMinPos * 2;
		SRV_OUT_REG = servoOpenPos * 2;

		// enable timer overflow interrupt
		// this timer counter is also used to measure ESC input pulses
		// so we need to know when counter overflows
		TIMSK1 = _BV( TOIE1 );
	}
}

// Set pin change interrupt on ESC input pin and button pins
void setupPinChangeInterrupt()
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
	{
		// digital pin 8 (PCINT0) corresponds to pin change interrupt 0
		PCICR |= _BV( PCIE0 ); // enable pin change interrupt 0
		PCMSK0 |= _BV( PCINT0 ); // enable interrupt on PCINT0 pin (Esc input)
		
		// digital pins 3 (PCINT19), 4 (PCINT20), 5 (PCINT21) correspond to pin change interrupt 2
		PCICR |= _BV( PCIE2 ); // enable pin change interrupt 2
		PCMSK2 |= _BV( PCINT19 ); // enable interrupt on PCINT19 pin (Servo button)
		PCMSK2 |= _BV( PCINT20 ); // enable interrupt on PCINT20 pin (Arm/Disarm button)
		PCMSK2 |= _BV( PCINT21 ); // enable interrupt on PCINT21 pin (Heatbeat input)
	}
}

// Main setup
void setup() 
{
	pinMode( ESC_IN_PIN, INPUT );
	pinMode( HEARTBEAT_IN_PIN, INPUT_PULLUP );
	pinMode( ESC_OUT_PIN, OUTPUT );
	digitalWrite( ESC_OUT_PIN, LOW );
	pinMode( SRV_OUT_PIN, OUTPUT );
	digitalWrite( SRV_OUT_PIN, LOW );

	pinMode( BUTTON_ARM_PIN, INPUT_PULLUP );
	pinMode( BUTTON_SRV_PIN, INPUT_PULLUP );

	pinMode( SRV_LED_PIN, OUTPUT );
	pinMode( ARM_LED_PIN, OUTPUT );
	pinMode( BOARD_LED_PIN, OUTPUT );
	
	// read saved EEPROM values
	escMinPos = restrictServoValue( readEEPROM( ESC_MIN_ADDRESS ) );
	servoOpenPos = restrictServoValue( readEEPROM( SERVO_OPEN_ADDRESS ) );
	servoClosePos = restrictServoValue( readEEPROM( SERVO_CLOSE_ADDRESS ) );

	setupTimer1();
	setupPinChangeInterrupt();

	//Serial.begin( 9600 );
	//Serial.print( "Finished setup\r\n" );
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop

void loop()
{
	unsigned long currentMillis = millis();

	// Update LED state
	static unsigned long prevLedUpdate = currentMillis;
	if( currentMillis - prevLedUpdate >= LED_UPDATE_INTERVAL ) {
		servoLedController.UpdateState( currentMillis );
		armLedController.UpdateState( currentMillis );
		boardLedController.UpdateState( currentMillis );
		prevLedUpdate = currentMillis;
	}

	// Update heartbeat state
	static unsigned long prevHeartbeatUpdate = currentMillis;
	static bool prevHasHeartbeat = false;
	if( currentMillis - prevHeartbeatUpdate >= HEARTBEAT_UPDATE_INTERVAL ) {
		prevHeartbeatUpdate= currentMillis;
		ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
		{
			processHeartbeat();
		}
		if( !prevHasHeartbeat && hasHeartbeat ) {
			onHeatbeatCapture();
		}
		if( prevHasHeartbeat && !hasHeartbeat ) {
			onHeartbeatLost();
		}
		prevHasHeartbeat = hasHeartbeat;
	}

	// Update and pass through ESC input
	static unsigned long prevEscInUpdate = currentMillis;
	if( currentMillis - prevEscInUpdate >= ESC_IN_UPDATE_INTERVAL ) {
		passThroughEscInput();
		prevEscInUpdate = currentMillis;
	}

	// Update button state
	static unsigned long prevButtonUpdate = currentMillis;
	if( currentMillis - prevButtonUpdate >= BUTTON_UPDATE_INTERVAL ) {
		prevButtonUpdate = currentMillis;
		ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
		{
			// process button presses
			armDisarmButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_ARM_REG, BUTTON_ARM_FLAG ) );
			servoButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_SRV_REG, BUTTON_SRV_FLAG ) );
		}

		// Process button presses
		if( armButtonPress ) {
			//Serial.print( "Arm press.\r\n" );
			onArmButtonPress();
			armButtonPress = false;
		}
		if( armButtonLongPress ) {
			//Serial.print( "Arm long press.\r\n" );
			onArmButtonLongPress();
			armButtonLongPress = false;
		}
		if( servoButtonPress ) {
			//Serial.print( "Servo press.\r\n" );
			onServoButtonPress();
			servoButtonPress = false;
		}
		if( servoButtonLongPress ) {
			//Serial.print( "Servo long press.\r\n" );
			onServoButtonLongPress();
			servoButtonLongPress = false;
		}
	}
}
