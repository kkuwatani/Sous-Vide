//ds18b20 macros

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

// ds18b20  functions

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
void ds18b20_writebit(uint8_t bit) {
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
uint8_t ds18b20_readbit(void) {
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
void ds18b20_writebyte(uint8_t byte) {
	uint8_t i=8;
	while(i--) {
		ds18b20_writebit(byte&1);
		byte >>= 1;
	}
}

//read one byte
uint8_t ds18b20_readbyte(void) {
	uint8_t i=8, n=0;
	while(i--) {
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

	ds18b20_reset(); 
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); 
	ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP); 

	while(!ds18b20_readbit()); //wait until conversion is complete

	ds18b20_reset(); 
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); 
	ds18b20_writebyte(DS18B20_CMD_RSCRATCHPAD); 

	//read 2 byte from scratchpad
	temperature_l = ds18b20_readbyte();
	temperature_h = ds18b20_readbyte();
	
	//send data to computer using UART for debugging and plotting data
	//usart_transmit(temperature_l);
	//usart_transmit(temperature_h);

	//convert the 12 bit value obtained
	temperature_full = ( ( temperature_h << 8 ) + temperature_l ) * 0.0625;

	return temperature_full;
}