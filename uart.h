//UART Functions 

void usart_init(uint16_t ubrr_value)
{
  /*Set Frame Format
   >> Asynchronous mode
   >> No Parity
   >> 1 StopBit
   >> char size 8
   */

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

	//Put data in buffer to be transmitted 
	UDR = data;
}
