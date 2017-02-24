#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define DATA_REGISTER 		PORTD
#define DATA_REGISTER_DDR 	DDRD
#define CTRL_REGISTER 		PORTA
#define CTRL_REGISTER_DDR 	DDRA
#define ENABLE 				1
#define RW 					2
#define RS 					3

// functions
char column_position[4] = {0, 64, 20, 84}; //may need to change these for specific LCD display
void lcd_wait(void);
void lcd_enable_disable(void);
void lcd_send_command(unsigned char command);
void lcd_send_char(unsigned char character);
void lcd_send_string(char *char_string);
void lcd_go_to_coordinates(uint8_t x, uint8_t y);
void lcd_send_position_string(uint8_t x, uint8_t y, char *char_string);
void lcd_send_position_int(uint8_t x, uint8_t y, int number, char digits);
void lcd_init(void);


void lcd_wait()
{
	//checks if lcd is busy and waits til its free
	DATA_REGISTER_DDR = 0;
	CTRL_REGISTER |= 1<<RW;
	CTRL_REGISTER &= ~1<<RS;

	while (DATA_REGISTER >= 0x80)
	{
		lcd_enable_disable();
	}

	DATA_REGISTER_DDR = 0xFF;
}

void lcd_enable_disable()
{
	// flips enable bit to refresh lcd
	CTRL_REGISTER |= 1<<ENABLE;
	asm volatile ("nop");
	asm volatile ("nop");
	CTRL_REGISTER &= ~1<<ENABLE;
}

void lcd_send_command(unsigned char command)
{
	lcd_wait();
	DATA_REGISTER = command;
	CTRL_REGISTER &= ~ ((1<<RW)|(1<<RS));
	lcd_enable_disable();
	DATA_REGISTER = 0;
}

void lcd_send_char(unsigned char character)
{
	lcd_wait();
	DATA_REGISTER = character;
	CTRL_REGISTER &= ~ (1<<RW);
	CTRL_REGISTER |= 1<<RS;
	lcd_enable_disable();
	DATA_REGISTER = 0;
}

void lcd_send_string(char *char_string)
{
	while(*char_string > 0)
	{
		lcd_send_char(*char_string++);
	}
}

void lcd_go_to_coordinates(uint8_t x, uint8_t y)
{
	lcd_send_command(0x80 + column_position[y-1] + (x-1));
}

void lcd_send_position_string(uint8_t x, uint8_t y, char *char_string)
{
	lcd_go_to_coordinates(x, y);
	lcd_send_string(char_string);
}

void lcd_send_position_int(uint8_t x, uint8_t y, int number, char digits)
{
	char display_string[digits];
	itoa(number, display_string, 10);
	lcd_send_position_string(x, y, display_string); lcd_send_string(" ");
}

void lcd_init()
{
	CTRL_REGISTER_DDR |= 1<<ENABLE | 1<<RW | 1<<RS;
	_delay_ms(15);
	lcd_send_command(0x01);
	_delay_ms(2);
	lcd_send_command(0x38);
	_delay_us(50);
	lcd_send_command(0b00001110);
	_delay_us(50);
}
