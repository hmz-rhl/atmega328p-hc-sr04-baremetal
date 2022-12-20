/*
 * main.c
 *	code to control a fan with interrupt and pwm signal
 *  Created on: 7 nov. 2022
 *      Author: rahal
 */
// PWM -> D3
// TRIG -> D2
// ECHO -> D4


//PB13 is the built-in LED

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define STEP 15

#define DELAY 122
#define MAX_DISTANCE 40.0
#define SOUND_SPEED 34029

// memo
// D4 pcint20 PD4


// global variable
int pwm = 0;

volatile uint64_t temps_10us = 0;

volatile uint8_t triggered = 0;

void init()
{
	//          ...    pin 4 as Input for echo, 3 as output for pwm and 2 as output for trigger
	DDRD = 0b00001100;


	// built-in led
	DDRB |= 1<<PB5;
	PORTB |= 1<<5;

	//PORTD = 1<< 3;
// -------------------FOR ECHO PIN INTERRUPT

	PCICR |= 0b00000100; // enabling PCIE2 interrupt
	PCMSK2 |= (1<<4);
// --------------------
	TCCR0A |= 0b00000010;			// CTC mode and normal port operation
	TCCR0B |= 0b10000010;			// enable clock with prescaler = 8

	OCR0A = 9;
	TIMSK0 |= 0b000000010;  		// enable Output compare A interrupts


// --------------------FOR PWM INTERRUPT--- TIMER2

	TCCR2A |= 0b00100011;			// fast PWM mode
	TCCR2B |= 0b00000001;			// Prescaler = 1

	OCR2B = 0;
	TIMSK2 |= 0b00000001; 			// enable Overflow interrupts
// --------------------


	SREG |= 0x80;					// Global Interrupt Enable
}

// Interrupt Service Routine for ECHO pin
//-------------------------------------------

ISR(PCINT2_vect)
{

	static unsigned char state = 0, old_state = 0;
	static float distance = 0;
	static unsigned char pwm =0;

	old_state = state;
	state = (PIND & 0b00010000);

	// rising edge
	if(state && !old_state){

		// il faut activer le timer ici
		temps_10us = 0;

	}
	//falling edge
	else if(!state && old_state){
		// on set le pwm et on relance un pulse
		// DEBUG
		//PINB |= 1<<PB5;
		triggered = 0;
		distance = ((float)(temps_10us * SOUND_SPEED))/200000.0;
		// if distance measured is over max_distance we set to the pwm to its maximum value
		if(distance > MAX_DISTANCE){
			OCR2B = 255;
		}
		// if distance measured is over min_distance we set to the pwm to its minimum value
		else if (distance < 5){
			OCR2B = 0;
		}

		else{
			pwm = (unsigned char)((distance / MAX_DISTANCE)* 255);
			OCR2B = 0xFF&pwm;
		}
		_delay_us(100);
	}
	else{
		triggered = 0;
	}
}

// just to reset flags
ISR(TIMER2_OVF_vect)
{
}

ISR(TIMER2_COMPB_vect)
{
}


// Interrupt Service Routine for 10micros interrupt (timer0)
ISR(TIMER0_COMPA_vect){
	temps_10us++; // count each 10 micros
	TCNT0 = 0;		// reseting the counter
}

// main program
//--------------

int main(void)
{
	// initialize Timers/Counters and associated interrupts

	init();

	while(1){
			// LED ON
		// trigger pin 2 = PD2
		if(!triggered){
			// generating the pulse
			PORTD |= (1<<PD2);
			_delay_us(10);
			PORTD &= ~(1<<PD2);
			triggered = 1;
		}

	}

	return 0;
}
