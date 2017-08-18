#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "LCD.h"
#include "DS18B20.h"
#include "uart.h"

// variables //
float current_temp;
float integral_sum;
float set_point = 70.0;
float output = 100;
uint8_t kp = 50;
float ki = 2.5;
uint8_t kd = 0;
uint8_t i_limit = 2;
uint8_t cycle_counter = 0;
uint8_t running = 0;

/*
2 states for sous vide (states are triggered by toggle button):
Off state - uses a potentiometer and ADC to write out setpoint temperatures to LCD screen
On  state - PWM water heater outputs, 
			reads ds18b20 temp sensor and uses PI control loop to reach chosen temperature setpoint, 
			LCD screen reads out current water temperature
 */

void pwm_init() {	
	//set OC1A pin as output
	DDRD |= (1<<PD5);

	//set registers to Fast PWM mode, prescaler = 8 (2Hz cycle)
	TCCR1A |= (1<<COM1A1)|(0<<COM1A0)|(1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11);
	ICR1 = 62500;

	//turn on PWM at 0% duty, 62500 = 100%
	OCR1A = 0;

	TCNT1 = 0;

	//enable TOV1 interrupt 
	TIMSK |= (1<<TOIE1);
}

void button_init() {
	MCUCR |= (1<<ISC01) | (1<<ISC00);
	DDRB |= 1 << PB0;
	GICR |= (1<<INT0);
}

void adc_init() {
	//initialization for 10bit ADC, right shifted, interrupt enabled
	ADCSRA |= 1<<ADPS2;
	ADMUX |= (1<<REFS0) | (1<<REFS1);
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	ADCSRA |= 1<<ADSC;
	
}

/* Interrupt Service Routines */

//PID control loop timer ISR
ISR(TIMER1_OVF_vect) {
	if(running == 1) {
		cycle_counter++;	
	}

	// PID ON EVERY 10 seconds	
	if(cycle_counter >= 20) {
		cycle_counter = 0;
		current_temp = ds18b20_gettemp();
		lcd_send_position_int(13, 1, (int)current_temp, 4);
	
		// proportional control w/o integral
		if(abs(set_point - current_temp) > i_limit) {
			output = kp*(set_point - current_temp);
		}
		// add integral term if w/ in 2C
		else {	
			integral_sum += (set_point - current_temp);
			output = kp*(set_point - current_temp) + ki*integral_sum;
		}
		// set output cap at 100% duty cycle
		if(output > 100) {
			output = 100;
		}
		OCR1A = (int)(output * 625);
	}	
}

//start/stop button hardware ISR 
ISR(INT0_vect) {
	//disable the interrupt for button debouncing
	GICR |= (0<<INT0);
	//start PID
	if(running == 0) {
		running = 1;
		PORTB |= (1 << PB0);
		ADCSRA &= ~(1<<ADIE);
		lcd_send_position_string(1, 1, "Current Temp:");
		lcd_send_position_string(13, 1, " ");

	}
	//stop PID
	else if(running == 1) {
		running = 0;
		OCR1A = 0;
		PORTB &= ~(1 << PB0);
		ADCSRA |= (1<<ADIE);
		lcd_send_position_string(1, 1, "Set Temp:");
	}
	//button debouncing
	_delay_us(1000);
	GICR |= (1<<INT0);
}

// ADC ISR used for choosing temperature set point 
ISR(ADC_vect) {
	uint8_t low = ADCL;
	uint16_t result = ADCH<<8 | low;
	lcd_send_position_int(13, 1, result, 4);

	ADCSRA |= 1<<ADSC; 
}

/* main loop*/
int main(void) {	
	button_init();
	usart_init(25);
	pwm_init();
	adc_init();
	lcd_init();
	lcd_send_position_string(1, 1, "Set Temp:");

	//enable global interrupt
	sei();

	while(1) {
	}
}
