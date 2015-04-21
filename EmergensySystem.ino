#include <util/atomic.h>
#include "ButtonWatcher.h"

// Пины
#define SERVO_IN_PIN 8 // входной сигнал сервы
#define SERVO_OUT_PIN 10 // выход сервы
#define LED_PIN 13 // светодиод

#define BUTTON_PIN 4 // кнопка
#define BUTTON_PIN_REG PIND
#define BUTTON_PIN_FLAG PIND4
 

int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
long interval = 1000;           // interval at which to blink (milliseconds)

const int ReadInterval = 50;
unsigned long prevRead = 0;

int buttonState = LOW;

volatile unsigned int servoPulseStart = 0;
volatile bool hasOverflow = false;

volatile int maxOverflow = 0;
volatile int overflowCount = 0;

volatile unsigned int servoInput = 0; // в микросекундах

#define SERVO_MAX 2000
#define SERVO_MIN 1000

volatile bool reportPress = false;
volatile bool reportLongPress = false;

void onButtonPress()
{
	reportPress= true;
}

void onButtonLongPress()
{
	reportLongPress = true;
}

static CButtonWatcher buttonWatcher( onButtonPress, onButtonLongPress );

void setServoOut( unsigned int servoValue )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		OCR1B = 2 * min( max( servoValue, SERVO_MIN ), SERVO_MAX );
	}
}

void setupTimer1()
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		//Fast PWM mode, TOP - OCR1A
		//capture raising edge
		//prescaler 1/8
		TCCR1A = _BV( WGM10 ) | _BV( WGM11 ) | _BV( COM1B1 );
		TCCR1B = _BV( ICES1 ) | _BV( CS11 ) | _BV( WGM12 ) | _BV( WGM13 );

		// 16мгц с делителем 8 дают один такт в 0,5 мкс
		// нам нужен период 20 мс, т.е. 40000 тактов
		OCR1A = 40000;

		OCR1B = SERVO_MIN * 2;

		//enable capture interupt
		TIMSK1 = _BV( ICIE1 ) | _BV( TOIE1 );
	}
}

// прерывание на изменение сигнала на входе 8 (ICP1)
ISR( TIMER1_CAPT_vect )
{
	// все переменные, которые используются в превываниях быстро копируем во
	// временные переменные атомарной операцией, и потом спокойно обрабатываем
	bool ovf;
	unsigned int counter;
	bool isPulseStart;
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		ovf = hasOverflow;
		counter = ICR1;
		hasOverflow = false;
		overflowCount = 0;
		isPulseStart = bit_is_set( TCCR1B, ICES1 );
		TCCR1B ^= _BV( ICES1 ); // теперь ждем другой фронт
	}

	if( isPulseStart ) {
		// передний фронт импульса, запоминаем значение счетчика
		servoPulseStart = counter;
	} else {
		// задний фронт, считаем длину импульса
		unsigned int pulseLen = 0xFFFF;
		if( !ovf ) {
			// не было переполнения
			pulseLen = ( counter - servoPulseStart );
		} else {
			if( counter < servoPulseStart ) {
				pulseLen = ( 0xFFFF - servoPulseStart ) + counter;
			}
		}
		
		if( pulseLen > SERVO_MAX * 4 || pulseLen < SERVO_MIN ) {
			// слишком короткий или длинный импульс пропускаем
			return;
		}
		
		servoInput = min( max( pulseLen / 2, SERVO_MIN ), SERVO_MAX );
	}
}

ISR( TIMER1_OVF_vect )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
	{
		overflowCount++;
		maxOverflow = max( maxOverflow, overflowCount );
		if( hasOverflow ) {
			// если случилось больше одного переполнения - начинаем снова ждать передний фронт
			TCCR1B |= _BV( ICES1 );
			hasOverflow = false;		
		} else {
			hasOverflow = true;
		}
	}
}

void setupPinChangeInterrupt()
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
	{
		// digital pin 4 is PCINT20 pin and correspond to pin change interrupt 2
		PCICR |= _BV( PCIE2 ); // enable pin change interrupt 2
		PCMSK2 |= _BV( PCINT20 ); // enable interrupt on PCINT20 pin
	}
}

// pin change interrupt 2 routine
ISR( PCINT2_vect )
{
	ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
	{
		bool isButtonPressed = bit_is_set( BUTTON_PIN_REG, BUTTON_PIN_FLAG );
		buttonWatcher.UpdateButtonState( isButtonPressed );
	}
}

void setup() 
{
	pinMode( LED_PIN, OUTPUT );
	pinMode( SERVO_OUT_PIN, OUTPUT );
	pinMode( BUTTON_PIN, INPUT );
	pinMode( SERVO_IN_PIN, INPUT );
	
	setupTimer1();
	setupPinChangeInterrupt();

	Serial.begin( 9600 );
	Serial.print( "Finished setup\r\n" );
}

void loop()
{
	// here is where you'd put code that needs to be running all the time.

	// check to see if it's time to blink the LED; that is, if the
	// difference between the current time and last time you blinked
	// the LED is bigger than the interval at which you want to
	// blink the LED.
	unsigned long currentMillis = millis();

	if(currentMillis - previousMillis >= interval) {
	// save the last time you blinked the LED 
	previousMillis = currentMillis;   

	// if the LED is off turn it on and vice-versa:
	if (ledState == LOW)
	  ledState = HIGH;
	else
	  ledState = LOW;

	// set the LED with the ledState of the variable:
	digitalWrite( LED_PIN, ledState );
	}

	if( currentMillis - prevRead >= ReadInterval ) {
		ATOMIC_BLOCK( ATOMIC_RESTORESTATE ) 
		{
			bool isButtonPressed = bit_is_set( BUTTON_PIN_REG, BUTTON_PIN_FLAG );
			buttonWatcher.UpdateButtonState( isButtonPressed );
		}

		if( reportPress ) {
			Serial.print( "Press!\r\n" );
			reportPress = false;
		}
		if( reportLongPress ) {
			Serial.print( "Long press!\r\n" );
			reportLongPress = false;
		}

		prevRead = currentMillis;

		/*buttonState = digitalRead( BUTTON_PIN );
		prevRead = currentMillis;

		static int prevState = LOW;	
		if( buttonState == HIGH && prevState == LOW ) {
			unsigned int sinp = 0;
			int ov = 0;
			ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
			{
			sinp = servoInput;
			ov = overflowCount;
			}
			setServoOut( sinp );
			Serial.print( "Servo input: " );
			Serial.print( sinp );
			Serial.print( "\r\n" );
			Serial.print( ov );
			Serial.print( "\r\n" );
		}
		prevState = buttonState;*/
	}
}
