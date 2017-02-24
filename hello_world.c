#include<avr/io.h>
#include<util/delay.h>


void setPinHigh(volatile *port, int pin) 
{
	*port |= (1<<pin);
}

void setPinLow(volatile *port, int pin) 
{
	*port &= ~(1<<pin);
}

void changePinState(volatile *port, int pin)
{
	*port ^= (1<<pin);
}

int main(void)
{
	DDRB = 0xff;
	
	while(1)
	{

// Method 1 uses two different bit masks for setting pin high and low 
//		setPinHigh(&PORTB, 0);
//		_delay_ms(500);
//		setPinLow(&PORTB, 0);
//		_delay_ms(500);

// Method 2 uses a single XOR bit mask for setting pin high AND low
		changePinState(&PORTB, 0);
		_delay_ms(250);
	}
    return 0;
}
