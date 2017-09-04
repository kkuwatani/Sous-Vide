//import peripherals and libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h"
#include "DS18B20.h"
#include "uart.h"
#include "pwm.h"
#include "adc.h"
#include "button.h"

//define control loop macros and clock
#define F_CPU   1000000UL
#define KP      50
#define KI      2.5
#define KD      0
#define KI_LIMIT 2

// declare variables //
float current_temp;
float integral_sum;
float set_point;
float output = 100;
uint8_t cycle_counter = 0;
uint8_t running = 0;

/*-----------------------------------------------------------------------------------------------
2 states for sous vide (states are triggered by toggle button):

SET_TEMP state      - uses a potentiometer and ADC to write out setpoint temperatures to LCD screen

CONTROL_TEMP  state - PWM water heater outputs,
			          reads ds18b20 temp sensor and uses PI control loop to reach chosen temperature setpoint,
			          LCD screen reads out current water temperature
-----------------------------------------------------------------------------------------------*/

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
		if(abs(set_point - current_temp) > KI_LIMIT) {
			output = KP*(set_point - current_temp);
		}
		// add integral term if w/ in 2C
		else {	
			integral_sum += (set_point - current_temp);
			output = KP*(set_point - current_temp) + KI*integral_sum;
		}
		// set output cap at 100% duty cycle
		if(output > 100) {
			output = 100;
		}
		pwm_set(int(output));
	}	
}

//start/stop button hardware ISR 
ISR(INT0_vect) {
	//disable the interrupt for button debouncing
	button_interrupt_disable();
	//start PID
	if(running == 0) {
		running = 1;
		running_led_enable();
		adc_interrupt_disable();
		lcd_send_position_string(1, 1, "Current Temp:");
		lcd_send_position_string(13, 1, " ");

	}
	//stop PID
	else if(running == 1) {
		running = 0;
		pwm_set(0);
		running_led_disable();
		adc_interrupt_enable();
		lcd_send_position_string(1, 1, "Set Temp:");
	}
	//button debouncing
	_delay_us(50);
	button_interrupt_enable();
}

// ADC ISR used for choosing temperature set point 
ISR(ADC_vect) {
	adc_receive();
	lcd_send_position_int(13, 1, result, 4);
    set_point = result;
    adc_start_conversion();
}

/* main function*/
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
