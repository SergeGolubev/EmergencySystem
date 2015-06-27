#include <util/atomic.h>
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

#define LED_PIN 13
CLedController ledController( LED_PIN );

// LED state update interval
// We dont need to blink faster than 10 times a second, so 50 ms is enough
#define LED_UPDATE_INTERVAL 50 // 50 ms

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Button processing

volatile bool armButtonPress = false;
volatile bool armButtonLongPress = false;
static CButtonWatcher armDisarmButtonWatcher( &armButtonPress, &armButtonLongPress );

volatile bool servoButtonPress = false;
volatile bool servoButtonLongPress = false;
static CButtonWatcher servoButtonWatcher( &servoButtonPress, &servoButtonLongPress );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Output PWM control

// Set output ESC signal is microseconds
void setEscOut( unsigned int escValue )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		// resolution is 0,5 mks so multiply by 2
		ESC_OUT_REG = 2 * min( max( escValue, SERVO_MIN ), SERVO_MAX );
	}
}

// Set output servo signal is microseconds
void setServoOut( unsigned int servoValue )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		// resolution is 0,5 mks so multiply by 2
		SRV_OUT_REG = 2 * min( max( servoValue, SERVO_MIN ), SERVO_MAX );
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
			escInput = min( max( pulseLen / 2, SERVO_MIN ), SERVO_MAX );
		}		
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Heartbeat input processing

volatile bool hasHeartbeat = false;

inline void onHeatbeatCapture()
{
	ledController.TurnOn();
	setServoOut( SERVO_MAX );
}

inline void onHeartbeatLost()
{
	ledController.BlinkConstantly( 250 );
	setServoOut( SERVO_MIN );
}

inline void processHeartbeat()
{
	unsigned long currentMillis = millis();
	static unsigned long prevPulseMillis = 0; // time of previous heartbeat pulse
	static unsigned long pulsesStartMillis = 0; // time of first pulse

	// check if heartbeat is lost
	if( prevPulseMillis != 0 && currentMillis - prevPulseMillis > HEARTBEAT_TIMEOUT ) {
		// too long time passed from prev pulse
		if( hasHeartbeat ) {
			hasHeartbeat = false;
			onHeartbeatLost();
		}
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
		// report heartbeat capture
		hasHeartbeat = true;
		onHeatbeatCapture();
	}

	// remember time of first pulse in sequence
	if( pulsesStartMillis == 0 ) {
		pulsesStartMillis = currentMillis;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pass captured ESC input to ESC output or Servo output

void passThroughEscInput()
{
	if( hasHeartbeat ) {
		setEscOut( escInput );
	} else {
		setEscOut( SERVO_MIN );
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

		OCR1A = SERVO_MIN * 2;
		OCR1B = SERVO_MIN * 2;

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
	pinMode( LED_PIN, OUTPUT );

	setupTimer1();
	setupPinChangeInterrupt();

	Serial.begin( 9600 );
	Serial.print( "Finished setup\r\n" );
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
		ledController.UpdateState( currentMillis );
		prevLedUpdate = currentMillis;
	}

	// Update heartbeat state
	static unsigned long prevHeartbeatUpdate = currentMillis;
	if( currentMillis - prevHeartbeatUpdate >= HEARTBEAT_UPDATE_INTERVAL ) {
		prevHeartbeatUpdate= currentMillis;
		ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
		{
			processHeartbeat();
		}
	}

	// Update and pass through ESC input
	static unsigned long prevEscInUpdate = currentMillis;
	if( currentMillis - prevEscInUpdate >= ESC_IN_UPDATE_INTERVAL ) {
		passThroughEscInput();
		prevEscInUpdate = currentMillis;
	}

	// update button state
	static unsigned long prevButtonUpdate = currentMillis;
	if( currentMillis - prevButtonUpdate >= BUTTON_UPDATE_INTERVAL ) {
		prevButtonUpdate = currentMillis;
		ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
		{
			// process button presses
			armDisarmButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_ARM_REG, BUTTON_ARM_FLAG ) );
			servoButtonWatcher.UpdateButtonState( !bit_is_set( BUTTON_SRV_REG, BUTTON_SRV_FLAG ) );
		}

		if( armButtonPress ) {
			Serial.print( "Arm press.\r\n" );
			armLedController.Switch();
			armButtonPress = false;
		}
		if( armButtonLongPress ) {
			Serial.print( "Arm long press.\r\n" );
			armLedController.BlinkConstantly( 500 );
			armButtonLongPress = false;
		}

		if( servoButtonPress ) {
			Serial.print( "Servo press.\r\n" );
			servoLedController.Switch();
			servoButtonPress = false;
		}
		if( servoButtonLongPress ) {
			Serial.print( "Servo long press.\r\n" );
			servoLedController.BlinkConstantly( 300 );
			servoButtonLongPress = false;
		}
	}
}
