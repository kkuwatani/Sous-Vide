#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "LCD.h"
#include "DS18B20.h"
#include "uart.h"

/*
//ds18b20 Macros//
//setup connection
#define DS18B20_PORT 	PORTC
#define DS18B20_DDR 	DDRC
#define DS18B20_PIN 	PINC
#define DS18B20_DQ 		PC0
//commands
#define DS18B20_CMD_CONVERTTEMP 	0x44
#define DS18B20_CMD_RSCRATCHPAD 	0xbe
#define DS18B20_CMD_WSCRATCHPAD 	0x4e
#define DS18B20_CMD_CPYSCRATCHPAD 	0x48
#define DS18B20_CMD_RECEEPROM 		0xb8
#define DS18B20_CMD_RPWRSUPPLY 		0xb4
#define DS18B20_CMD_SEARCHROM 		0xf0
#define DS18B20_CMD_READROM 		0x33
#define DS18B20_CMD_MATCHROM 		0x55
#define DS18B20_CMD_SKIPROM 		0xcc
#define DS18B20_CMD_ALARMSEARCH 	0xec
#define DS18B20_STOPINTERRUPTONREAD 0
*/

//functions

/* UART Functions */
/*
void usart_init(uint16_t ubrr_value)
{
  //Set Frame Format
  //  Asynchronous mode
  //  No Parity
  //  1 StopBit
  //  char size 8
  

   //Set Baud rate
   UBRRL = ubrr_value;
   UBRRH = (ubrr_value>>8);

   UCSRC=(1<<URSEL)|(3<<UCSZ0);

   //Enable The receiver and transmitter
   UCSRB=(1<<RXEN)|(1<<TXEN);
}

void usart_transmit (unsigned int data)
{
	//Wait until the Transmitter is ready
	while (! (UCSRA & (1 << UDRE)) );

	//Make the 9th bit 0 for the moment, If the 9th bit of the data is a 1, Set the TXB8 bit to 1
	UCSRB &=~(1 << TXB8); 
	if (data & 0x0100) 
		UCSRB |= (1 << TXB8); 

	//Get that data outa !
	UDR = data;
}
*/

/*ds18b20 communication functions*/
/*
//ds18b20 init
uint8_t ds18b20_reset() {
	uint8_t i;

	//low for 480us
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(480);

	//release line and wait for 60uS
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
	_delay_us(60);

	//get value and wait 420us
	i = (DS18B20_PIN & (1<<DS18B20_DQ));
	_delay_us(420);

	//return the read value, 0=ok, 1=error
	return i;
}

 //write one bit 
void ds18b20_writebit(uint8_t bit){
	//low for 1uS
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(1);

	//if we want to write 1, release the line (if not will keep low)
	if(bit)
		DS18B20_DDR &= ~(1<<DS18B20_DQ); //input

	//wait 60uS and release the line
	_delay_us(60);
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
}

//read one bit
uint8_t ds18b20_readbit(void){
	uint8_t bit=0;

	//low for 1uS
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(1);

	//release line and wait for 14uS
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
	_delay_us(14);

	//read the value
	if(DS18B20_PIN & (1<<DS18B20_DQ))
		bit=1;

	//wait 45uS and return read value
	_delay_us(45);

	return bit;
}


//write one byte
void ds18b20_writebyte(uint8_t byte){
	uint8_t i=8;
	while(i--){
		ds18b20_writebit(byte&1);
		byte >>= 1;
	}
}

//read one byte
uint8_t ds18b20_readbyte(void){
	uint8_t i=8, n=0;
	while(i--){
		n >>= 1;
		n |= (ds18b20_readbit()<<7);
	}
	return n;
}

//get temperature 
float ds18b20_gettemp() {
	uint8_t temperature_l;
	uint8_t temperature_h;
	float temperature_full = 0;

	#if DS18B20_STOPINTERRUPTONREAD == 1
	cli();
	#endif

	ds18b20_reset(); //reset
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
	ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP); //start temperature conversion

	while(!ds18b20_readbit()); //wait until conversion is complete

	ds18b20_reset(); //reset
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
	ds18b20_writebyte(DS18B20_CMD_RSCRATCHPAD); //read scratchpad

	//read 2 byte from scratchpad
	temperature_l = ds18b20_readbyte();
	temperature_h = ds18b20_readbyte();

	#if DS18B20_STOPINTERRUPTONREAD == 1
	sei();
	#endif
	
	//send data to computer using UART
	usart_transmit(temperature_l);
	usart_transmit(temperature_h);

	//convert the 12 bit value obtained
	temperature_full = ( ( temperature_h << 8 ) + temperature_l ) * 0.0625;

	return temperature_full;
}
*/

// variables //
float current_temp;
float integral_sum;
float set_point = 70.0;
float output = 100;
uint8_t kp = 50;
uint8_t ki = 2.5;
uint8_t kd = 0;
uint8_t i_limit = 2;
uint8_t cycle_counter = 0;
uint8_t running = 0;

void pwm_init() 
{	
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

void button_init()
{
	MCUCR |= (1<<ISC01) | (1<<ISC00);
	DDRB |= 1 << PB0;
	GICR |= (1<<INT0);
}

void adc_init()
{
	//initialization for 10bit ADC, right shifted, interrupt enabled
	ADCSRA |= 1<<ADPS2;
	ADMUX |= (1<<REFS0) | (1<<REFS1);
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	ADCSRA |= 1<<ADSC;
	
}

/* Interrupt Service Routines */

//PID control loop timer ISR
ISR(TIMER1_OVF_vect)
{
	if(running == 1) 
	{
		cycle_counter++;	
	}

	// PID ON EVERY 10 seconds	
	if(cycle_counter >= 20)
	{
		cycle_counter = 0;

		current_temp = ds18b20_gettemp();
		lcd_send_position_int(13, 1, (int)current_temp, 4);
	
		// proportional control w/o integral
		if(abs(set_point - current_temp) > i_limit)
		{
			output = kp*(set_point - current_temp);
		}
		// add integral term if w/ in 2C
		else
		{	
			integral_sum += (set_point - current_temp);
			output = kp*(set_point - current_temp) + ki*integral_sum;
		}
		// set output cap at 100% duty cycle
		if(output > 100)
		{
			output = 100;
		}
		OCR1A = (int)(output * 625);
	}	
}

//start/stop button hardware ISR
ISR(INT0_vect)
{
	//disable the interrupt for button debouncing
	GICR |= (0<<INT0);
	//start PID
	if(running == 0)
	{
		running = 1;
		PORTB |= (1 << PB0);
		ADCSRA &= ~(1<<ADIE);
		lcd_send_position_string(1, 1, "Current Temp:");
		lcd_send_position_string(13, 1, " ");

	}
	//stop PID
	else if(running == 1)
	{
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

ISR(ADC_vect)
{
	uint8_t low = ADCL;
	uint16_t result = ADCH<<8 | low;
	lcd_send_position_int(13, 1, result, 4);

	ADCSRA |= 1<<ADSC; 
}

/* main code*/
int main(void)
{	
	button_init();
	usart_init(25);
	pwm_init();
	adc_init();
	lcd_init();
	lcd_send_position_string(1, 1, "Set Temp:");

	//enable global interrupt
	sei();

	while(1)
	{
	}
}
